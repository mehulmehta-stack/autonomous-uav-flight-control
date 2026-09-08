%% VALIDATE GAIN-SCHEDULED CONTROLLER AT ENVELOPE-EXCURSION SPEEDS (v2)
% Same as before, now also logging psi (val_state col 12) and rudder
% (val_control col 5) to diagnose the unexpected bank-angle coupling
% seen in stress_low (7.5 deg peak phi with zero lateral command).
%
% v2b: added REQ-011 elevator-margin reporting (data was already
% logged, just never printed) and forced a white figure background --
% the previous black background + a 'k' (black) line for the cruise
% case were making that trace invisible in the exported PNG.

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx';
ic_files = struct('stress_low','c172_cruise_8K_FINAL_FG_stress_low', ...
                   'cruise','c172_cruise_8K_FINAL_FG', ...
                   'stress_high','c172_cruise_8K_FINAL_FG_stress_high');

Tsim = 100;

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
    fprintf('Running scheduled model from %s IC (script: %s)...\n', cname, script_name);

    simOut = sim(mdl, 'StopTime', num2str(Tsim));

    stateData = get_logged_var(simOut, 'val_state');
    controlData = get_logged_var(simOut, 'val_control');

    results.(cname).t = stateData.time;
    results.(cname).u = stateData.signals.values(:,1);
    results.(cname).theta = stateData.signals.values(:,11);
    results.(cname).phi = stateData.signals.values(:,10);
    results.(cname).psi = stateData.signals.values(:,12);   % heading
    results.(cname).p = stateData.signals.values(:,4);
    results.(cname).elevator = controlData.signals.values(:,4);
    results.(cname).aileron = controlData.signals.values(:,2);
    results.(cname).rudder = controlData.signals.values(:,5);
end

set_param(sfBlockPath, 'Parameters', origParams);
close_system(mdl, 0);

%% Plot -- focused on the coupling diagnosis
figure('Position', [100 100 950 850]);
set(gcf, 'Color', 'white');   % force white background regardless of MATLAB theme
colors = {'m', [0 0.5 0], 'c'};   % dark green replaces pure black -- 'k' was
                                   % invisible against a black-themed export

subplot(3,1,1);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, rad2deg(results.(ic_names{c}).phi), 'Color', colors{c}, 'LineWidth', 1.5);
end
grid on; ylabel('\phi (deg)');
title('Bank angle (repeated for reference)');
legend(ic_names, 'Location', 'best');

subplot(3,1,2);
hold on;
for c = 1:numel(ic_names)
    psi0 = results.(ic_names{c}).psi(1);
    plot(results.(ic_names{c}).t, rad2deg(results.(ic_names{c}).psi - psi0), 'Color', colors{c}, 'LineWidth', 1.5);
end
grid on; ylabel('\Delta\psi (deg)');
title('Heading deviation from initial -- correlated with \phi excursion? (P-factor/torque signature)');

subplot(3,1,3);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, rad2deg(results.(ic_names{c}).rudder), 'Color', colors{c}, 'LineWidth', 1.5);
end
grid on; xlabel('Time (s)'); ylabel('Rudder (deg)');
title('Rudder position -- note FCS holds rudder fixed unless something is driving it');

set(findall(gcf, 'Type', 'axes'), 'Color', 'white');   % axes background is
                                                         % separate from the
                                                         % figure's background --
                                                         % the previous fix only
                                                         % set the figure's
saveas(gcf, 'GainSchedule_Stress_Coupling_Check.png');
fprintf('\nSaved: GainSchedule_Stress_Coupling_Check.png\n');
fprintf('If Delta-psi tracks phi in time for stress_low, propeller effects are the\n');
fprintf('likely driver. If psi stays flat while phi moves, look elsewhere (aero\n');
fprintf('cross-derivatives or an L1/EKF interaction) before writing a cause into docs.\n');

fprintf('\nREQ-011 elevator margin at envelope extremes:\n');
for c = 1:numel(ic_names)
    cname = ic_names{c};
    fprintf('  %-11s max |elevator| = %.4f\n', cname, max(abs(results.(cname).elevator)));
end

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
