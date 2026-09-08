%% VALIDATE GAIN-SCHEDULED CONTROLLER ACROSS OFF-NOMINAL TRIM CONDITIONS
% ------------------------------------------------------------
% Requires you to have manually added two To Workspace blocks in
% Final_JSB_4wp_LQG_withoutNoise_GnSch.slx, tapping the JSBSim S-Function's
% State (port 1) and Control (port 2) outputs:
%   - Variable name: val_state    | Save format: Structure With Time
%   - Variable name: val_control  | Save format: Structure With Time
% Save the model with those blocks in place before running this script.
%
% This validates ONLY the gain-scheduled model (no fixed-gain baseline
% is available anymore). It proves: the scheduled controller recovers
% cleanly from off-nominal starting conditions (100 / 220 ft/s) toward
% the 185 ft/s cruise reference, without actuator saturation or
% sustained oscillation. It does NOT claim improvement over the old
% fixed-gain controller, since that model no longer exists to compare
% against -- do not caption it that way on GitHub.
% ------------------------------------------------------------

clear; clc; close all;
bdclose('all');   % guarantee no stale model state from a previous crashed run

mfile = 'Final_JSB_4wp_LQG_withoutNoise_GnSch.slx';
ic_files = struct('low','c172_cruise_8K_FINAL_FG_low', ...
                   'cruise','c172_cruise_8K_FINAL_FG', ...
                   'high','c172_cruise_8K_FINAL_FG_high');

Tsim = 40;  % match the JSBSim run-script's own native duration (40.0082s)

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

if isempty(find_system(mdl, 'Name', 'val_state')) && isempty(find_system(mdl, 'VariableName', 'val_state'))
    % (best-effort check only; if this doesn't catch it, sim() below will
    % just fail loudly with "unrecognized variable" and you'll know the
    % manual blocks aren't wired yet)
end

ic_names = fieldnames(ic_files);
results = struct();

for c = 1:numel(ic_names)
    cname = ic_names{c};
    script_name = ic_files.(cname);

    newParams = swap_ic_field(origParams, script_name);
    set_param(sfBlockPath, 'Parameters', newParams);
    fprintf('  S-Function Parameters now: %s\n', newParams);
    fprintf('Running scheduled model from %s IC (script: %s)...\n', cname, script_name);

    simOut = sim(mdl, 'StopTime', num2str(Tsim));

    stateData = get_logged_var(simOut, 'val_state');
    controlData = get_logged_var(simOut, 'val_control');

    results.(cname).t = stateData.time;
    results.(cname).u = stateData.signals.values(:,1);
    results.(cname).theta = stateData.signals.values(:,11);
    results.(cname).elevator = controlData.signals.values(:,4);
end

set_param(sfBlockPath, 'Parameters', origParams);
close_system(mdl, 0);

%% Plot -- all three starting conditions overlaid
figure('Position', [100 100 900 650]);

subplot(2,1,1);
colors = {'m','k','c'};
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, results.(ic_names{c}).u, colors{c}, 'LineWidth', 1.5);
end
yline(184.999, 'k--');
grid on; xlabel('Time (s)'); ylabel('True airspeed u (ft/s)');
title('Gain-scheduled controller: airspeed recovery from 3 starting conditions');
legend([ic_names; {'cruise trim'}], 'Location', 'best');

subplot(2,1,2);
hold on;
for c = 1:numel(ic_names)
    plot(results.(ic_names{c}).t, results.(ic_names{c}).elevator, colors{c}, 'LineWidth', 1.5);
end
grid on; xlabel('Time (s)'); ylabel('Elevator position (norm)');
title('Elevator command -- watch for saturation (\pm1) or sustained oscillation');
legend(ic_names, 'Location', 'best');

saveas(gcf, 'GainSchedule_Validation.png');

%% Numeric summary
fid = fopen('GainSchedule_Validation_Summary.txt', 'w');
fprintf(fid, 'GAIN-SCHEDULED CONTROLLER VALIDATION\n');
fprintf(fid, '=====================================\n');
fprintf(fid, 'Single model, three starting conditions. Proves recovery behavior\n');
fprintf(fid, 'across the tested envelope, not improvement over a prior baseline.\n\n');
for c = 1:numel(ic_names)
    cname = ic_names{c};
    u = results.(cname).u;
    elev = results.(cname).elevator;
    tail = round(0.1 * numel(u));
    final_u = mean(u(end-tail+1:end));
    max_elev = max(abs(elev));
    fprintf(fid, '%-6s IC | final u = %8.3f ft/s | max |elevator| = %.4f\n', cname, final_u, max_elev);
end
fclose(fid);

fprintf('\nSaved: GainSchedule_Validation.png, GainSchedule_Validation_Summary.txt\n');

%% Helper: fetch a To-Workspace-logged variable regardless of whether this
% MATLAB/model routes it to the base workspace or into the sim() output
% object -- avoids guessing which mode applies on a given machine.
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
    error(['Could not find logged variable "%s" in the base workspace or ' ...
           'in the sim() output object. Open the model and confirm the ' ...
           'To Workspace block''s Variable Name is exactly "%s" and that ' ...
           'it is actually connected (click the block, check its input ' ...
           'port has a solid line, not just proximity to one).'], varname, varname);
end

%% Helper: replace the 4th (script) field of the S-Function Parameters
% string. Swapping field 5 (initfile) does NOTHING for this model,
% because c172_cruise_8K_FINAL_FG.xml has its own hardcoded
% <use initialize="c172p_cruise_init"/> line that overrides it. The
% only way to change the starting condition is to point at a different
% script file that has a different hardcoded <use> line.
function newParams = swap_ic_field(origParams, script_basename)
    parts = strsplit(origParams, ',');
    if numel(parts) < 4
        error('Unexpected S-Function Parameters format (expected 6 comma-separated fields): %s', origParams);
    end
    parts{4} = ['''scripts/' script_basename ''''];
    newParams = strjoin(parts, ',');
end