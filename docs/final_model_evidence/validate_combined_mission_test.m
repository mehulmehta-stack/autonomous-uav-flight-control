%% COMBINED SYSTEM TEST: Envelope-Excursion Speed + Real Waypoint Mission
% Same stress_low / stress_high IC files as before. Now run long enough
% (800s, matching your existing Day-7 integrated-run precedent) for L1
% guidance to actually command turns toward the 4 waypoints, so the
% throttle-transient roll/yaw coupling found in the 40s test can be
% checked against REAL commanded bank angles, not just a wings-level
% disturbance.

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withoutNoise_GnSch.slx';
ic_files = struct('stress_low','c172_cruise_8K_FINAL_FG_stress_low', ...
                   'cruise','c172_cruise_8K_FINAL_FG', ...
                   'stress_high','c172_cruise_8K_FINAL_FG_stress_high');

Tsim = 800;   % full mission-length run, not the 40s recovery-only window

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

ic_names = fieldnames(ic_files);
results = struct();

for c = 1:numel(ic_names)
    cname = ic_names{c};
    script_name = ic_files.(cname);

    newParams = swap_ic_field(origParams, script_name);
    set_param(sfBlockPath, 'Parameters', newParams);
    fprintf('Running FULL MISSION from %s IC (script: %s, Tsim=%ds)...\n', cname, script_name, Tsim);

    tic;
    simOut = sim(mdl, 'StopTime', num2str(Tsim));
    fprintf('  Wall-clock time: %.1f s\n', toc);

    stateData = get_logged_var(simOut, 'val_state');
    controlData = get_logged_var(simOut, 'val_control');

    results.(cname).t = stateData.time;
    results.(cname).u = stateData.signals.values(:,1);
    results.(cname).theta = stateData.signals.values(:,11);
    results.(cname).phi = stateData.signals.values(:,10);
    results.(cname).psi = stateData.signals.values(:,12);
    results.(cname).p = stateData.signals.values(:,4);
    results.(cname).lon = stateData.signals.values(:,8);   % ground track
    results.(cname).lat = stateData.signals.values(:,9);
    results.(cname).elevator = controlData.signals.values(:,4);
    results.(cname).aileron = controlData.signals.values(:,2);
    results.(cname).rudder = controlData.signals.values(:,5);
end

set_param(sfBlockPath, 'Parameters', origParams);
close_system(mdl, 0);

%% Plot 1: Ground track -- confirms real turns actually happened
figure('Position', [100 100 900 800]);
colors = {'m','k','c'};

subplot(3,2,[1 2]);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).lon, results.(ic_names{c}).lat, colors{c}, 'LineWidth', 1.5);
end
grid on; axis equal; xlabel('Longitude (deg)'); ylabel('Latitude (deg)');
title('Ground track -- confirms L1 guidance actually commanded turns');
legend(ic_names, 'Location', 'best');

subplot(3,2,3);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, results.(ic_names{c}).u, colors{c}, 'LineWidth', 1.5);
end
yline(184.999,'k--'); grid on; ylabel('u (ft/s)'); title('Airspeed');

subplot(3,2,4);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, rad2deg(results.(ic_names{c}).phi), colors{c}, 'LineWidth', 1.5);
end
grid on; ylabel('\phi (deg)'); title('Bank angle -- now includes REAL commanded turns');

subplot(3,2,5);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, results.(ic_names{c}).aileron*180/pi, colors{c}, 'LineWidth', 1.5);
end
grid on; xlabel('Time (s)'); ylabel('Aileron (deg)'); title('Aileron command -- distinguishes commanded turns from disturbance');

subplot(3,2,6);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, results.(ic_names{c}).elevator, colors{c}, 'LineWidth', 1.5);
end
yline(1,'r:'); yline(-1,'r:');
grid on; xlabel('Time (s)'); ylabel('Elevator (norm)'); title('Elevator -- check saturation across full mission');

saveas(gcf, 'GainSchedule_CombinedMission_Test.png');

%% Numeric summary
fid = fopen('GainSchedule_CombinedMission_Summary.txt', 'w');
fprintf(fid, 'COMBINED SYSTEM TEST -- FULL WAYPOINT MISSION FROM ENVELOPE-EXCURSION SPEEDS\n');
fprintf(fid, '=============================================================================\n');
fprintf(fid, 'Tsim = %d s. Tests whether the throttle-transient roll/yaw coupling found\n', Tsim);
fprintf(fid, 'in the 40s wings-level test (REQ-012) compounds with genuine L1-commanded\n');
fprintf(fid, 'turns, or is small enough to be absorbed within normal mission behavior.\n\n');
for c = 1:numel(ic_names)
    cname = ic_names{c};
    u = results.(cname).u; elev = results.(cname).elevator; phi = results.(cname).phi;
    fprintf(fid, '%-12s | max |elevator| = %.4f | max |phi| = %6.2f deg | final u = %8.3f ft/s\n', ...
        cname, max(abs(elev)), max(abs(rad2deg(phi))), u(end));
end
fclose(fid);

fprintf('\nSaved: GainSchedule_CombinedMission_Test.png, GainSchedule_CombinedMission_Summary.txt\n');
fprintf('Check the ground-track plot first -- if it is a straight line, no turn was\n');
fprintf('actually commanded and this run does not test what it is meant to.\n');

%% Helpers (unchanged)
function data = get_logged_var(simOut, varname)
    try, data = evalin('base', varname); return; catch, end
    try, data = simOut.get(varname); return; catch, end
    try, data = simOut.(varname); return; catch, end
    error('Could not find logged variable "%s".', varname);
end

function newParams = swap_ic_field(origParams, script_basename)
    parts = strsplit(origParams, ',');
    if numel(parts) < 4
        error('Unexpected S-Function Parameters format: %s', origParams);
    end
    parts{4} = ['''scripts/' script_basename ''''];
    newParams = strjoin(parts, ',');
end