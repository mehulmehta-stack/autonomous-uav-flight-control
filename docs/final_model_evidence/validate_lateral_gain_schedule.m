%% VALIDATE LATERAL GAIN-SCHEDULED CONTROLLER (Lat_GainSched)
% ------------------------------------------------------------
% Same methodology as validate_gain_schedule_v3.m, applied to the
% lateral axis. Disturbance is a 10 deg initial bank angle (not a
% guidance-commanded turn -- L1_Guidance's waypoint logic was not
% modified, avoiding the risk of misunderstanding an unfamiliar
% subsystem), at all three trim airspeeds.
%
% State output order (per c172_simulink.xml):
%   1 u, 2 v, 3 w, 4 p, 5 q, 6 r, 7 h, 8 long, 9 lat, 10 phi, 11 theta, 12 psi
% NOTE: this is a DIFFERENT ordering from the offline JSBSim 13-state
% linearization vector used to design K_lat_pts -- do not conflate them.
%
% Control output order:
%   1 throttle-pos-norm, 2 left-aileron-pos-rad, 3 right-aileron-pos-rad,
%   4 elevator-pos-norm, 5 rudder-pos-norm, ...
% NOTE: aileron here is POSITION IN RADIANS, not a normalized command --
% there is no normalized aileron command signal on any output port.
% Converted to degrees below for readability; do not compare directly
% to the elevator plot's normalized (-1 to 1) scale.
%
% GPS_Validity is left at its default (outage active t=25-35s) -- same
% as the longitudinal validation, this run does not override it. If you
% want a clean run without that transient, see the note in
% validate_gain_schedule_v3.m about the GPS override, but be sure to
% revert it afterward and document either choice in the README.
% ------------------------------------------------------------

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withoutNoise_GnSch.slx';
script_files = struct('low','c172_cruise_8K_FINAL_FG_low_roll', ...
                       'cruise','c172_cruise_8K_FINAL_FG_cruise_roll', ...
                       'high','c172_cruise_8K_FINAL_FG_high_roll');

Tsim = 40;

if ~isfile(mfile)
    error('Model file not found: %s', mfile);
end

load_system(mfile);
[~, mdl, ~] = fileparts(mfile);

sfBlocks = find_system(mdl, 'BlockType', 'S-Function', 'FunctionName', 'JSBSim_SFunction');
if isempty(sfBlocks)
    error('Could not find the JSBSim S-Function block in %s', mdl);
end
sfBlockPath = sfBlocks{1};
origParams = get_param(sfBlockPath, 'Parameters');

case_names = fieldnames(script_files);
results = struct();

for c = 1:numel(case_names)
    cname = case_names{c};
    script_name = script_files.(cname);

    newParams = swap_ic_field(origParams, script_name);
    set_param(sfBlockPath, 'Parameters', newParams);
    fprintf('  S-Function Parameters now: %s\n', newParams);
    fprintf('Running %s roll-disturbance case (script: %s)...\n', cname, script_name);

    simOut = sim(mdl, 'StopTime', num2str(Tsim));

    stateData = get_logged_var(simOut, 'val_state');
    controlData = get_logged_var(simOut, 'val_control');

    results.(cname).t = stateData.time;
    results.(cname).phi = rad2deg(stateData.signals.values(:,10));
    results.(cname).p = stateData.signals.values(:,4);
    results.(cname).aileron_deg = rad2deg(controlData.signals.values(:,2)); % left aileron position
end

set_param(sfBlockPath, 'Parameters', origParams);
close_system(mdl, 0);

%% Plot
figure('Position', [100 100 900 750]);
colors = struct('low','m','cruise','k','high','c');

subplot(3,1,1);
hold on;
for c = 1:numel(case_names)
    cname = case_names{c};
    plot(results.(cname).t, results.(cname).phi, colors.(cname), 'LineWidth', 1.5);
end
yline(0, 'r--');
grid on; xlabel('Time (s)'); ylabel('Bank angle \phi (deg)');
title('Lateral gain-scheduled recovery from 10 deg initial bank, 3 airspeeds');
legend(case_names, 'Location', 'best');

subplot(3,1,2);
hold on;
for c = 1:numel(case_names)
    cname = case_names{c};
    plot(results.(cname).t, results.(cname).p, colors.(cname), 'LineWidth', 1.5);
end
grid on; xlabel('Time (s)'); ylabel('Roll rate p (rad/s)');
title('Roll rate');
legend(case_names, 'Location', 'best');

subplot(3,1,3);
hold on;
for c = 1:numel(case_names)
    cname = case_names{c};
    plot(results.(cname).t, results.(cname).aileron_deg, colors.(cname), 'LineWidth', 1.5);
end
grid on; xlabel('Time (s)'); ylabel('Left aileron position (deg)');
title('Aileron position -- geometric limits approx [-20, +15] deg, check for pinning');
legend(case_names, 'Location', 'best');

saveas(gcf, 'LateralGainSchedule_Validation.png');

%% Numeric summary
fid = fopen('LateralGainSchedule_Validation_Summary.txt', 'w');
fprintf(fid, 'LATERAL GAIN-SCHEDULED CONTROLLER VALIDATION\n');
fprintf(fid, '=============================================\n');
fprintf(fid, 'Disturbance: 10 deg initial bank angle at t=0, 3 trim airspeeds.\n');
fprintf(fid, 'GPS_Validity left at default (outage active t=25-35s) -- not isolated\n');
fprintf(fid, 'for this run; a transient may appear in that window, unrelated to\n');
fprintf(fid, 'the roll disturbance itself.\n\n');

for c = 1:numel(case_names)
    cname = case_names{c};
    phi = results.(cname).phi;
    ail = results.(cname).aileron_deg;
    tail = round(0.1 * numel(phi));
    final_phi = mean(phi(end-tail+1:end));
    max_ail = max(abs(ail));
    fprintf(fid, '%-7s | final phi = %7.4f deg | max |aileron| = %6.2f deg\n', ...
        cname, final_phi, max_ail);
end
fclose(fid);

fprintf('\nSaved: LateralGainSchedule_Validation.png, LateralGainSchedule_Validation_Summary.txt\n');

%% Helpers (same as validate_gain_schedule_v3.m)
function newParams = swap_ic_field(origParams, script_basename)
    parts = strsplit(origParams, ',');
    if numel(parts) < 4
        error('Unexpected S-Function Parameters format: %s', origParams);
    end
    parts{4} = ['''scripts/' script_basename ''''];
    newParams = strjoin(parts, ',');
end

function data = get_logged_var(simOut, varname)
    try
        data = evalin('base', varname);
        return;
    catch
    end
    try
        data = simOut.get(varname);
        return;
    catch
    end
    try
        data = simOut.(varname);
        return;
    catch
    end
    error('Could not find logged variable "%s".', varname);
end
