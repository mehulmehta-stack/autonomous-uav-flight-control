%% REQ-015 — FINAL MODEL ALTITUDE-DISTURBANCE RECOVERY (MIL)
%
% Same test as the checkpoint's SIL_robustness_stress_test.md: aircraft
% starts at 3000 ft, controller commands 4000 ft, +1000 ft initial error.
% Difference here: runs the final gain-scheduled model directly (MIL),
% not the checkpoint's generated code (SIL). State that difference
% explicitly when this gets folded into requirements.md.
%
% Uses files that ALREADY EXIST -- c172p_cruise_init_stress.xml and
% c172_cruise_8K_FINAL_FG_STRESS.xml, both confirmed by direct inspection.
% generate_altitude_stress_ic.py is NOT needed and should not be run --
% it would only create a redundant duplicate of what's already here.

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx';
Tsim = 40;   % matches this run-script's own native <run end="40"> -- confirmed
             % from c172_cruise_8K_FINAL_FG_STRESS.xml, same convention
             % validate_gain_schedule_v3.m already established for the
             % airspeed-stress cases. Do not extend this without checking
             % whether the S-Function actually honors a longer Simulink
             % StopTime past the script's own configured end -- unverified.

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

script_name = 'c172_cruise_8K_FINAL_FG_STRESS';  % confirmed real file, not the
                                                   % placeholder name from before -
                                                   % points at c172p_cruise_init_stress
                                                   % via its own <use initialize=...>
newParams = swap_ic_field(origParams, script_name);
set_param(sfBlockPath, 'Parameters', newParams);
fprintf('Running altitude-disturbance case (script: %s)...\n', script_name);

simOut = sim(mdl, 'StopTime', num2str(Tsim));

set_param(sfBlockPath, 'Parameters', origParams);  % restore before any error path

stateData = get_logged_var(simOut, 'val_state');
controlData = get_logged_var(simOut, 'val_control');

t = stateData.time;
h_ft = stateData.signals.values(:,7);      % ft, per c172_simulink.xml state order
elevator = controlData.signals.values(:,4);

close_system(mdl, 0);

%% Checks, same convention as SIL_robustness_stress_test.md
% Note: the checkpoint version didn't fully settle until ~40-45s, and this
% run is capped at 40s (see the Tsim comment above) -- report what actually
% happened rather than assuming "settled" if it's still converging at t=40.
target_ft = 4000;
initial_error_ft = target_ft - h_ft(1);

near_target_idx = find(abs(h_ft - target_ft) < 5, 1, 'first');  % first time within 5 ft
if isempty(near_target_idx)
    t_settle = NaN;
    fprintf('Altitude did not come within 5 ft of the 4000 ft command within %ds --\n', Tsim);
    fprintf('report the final error and trend below rather than treating this as a failure;\n');
    fprintf('the checkpoint version took ~40-45s to fully settle too.\n');
else
    t_settle = t(near_target_idx);
end

% Trend over the last quarter of the run -- converging or not, regardless
% of whether it crossed the 5 ft band yet.
tail_idx = t >= 0.75*Tsim;
trend_slope = polyfit(t(tail_idx), h_ft(tail_idx), 1);  % ft/s, negative if descending toward target from above
fprintf('Altitude trend over final %.0fs: %.4f ft/s (toward 4000 ft if same sign as -(h(end)-4000))\n', ...
    0.25*Tsim, trend_slope(1));

max_elev = max(abs(elevator));
elev_saturated = max_elev >= 0.999;

fprintf('\n============================================================\n');
fprintf('REQ-015 — FINAL MODEL (MIL), altitude-disturbance recovery\n');
fprintf('============================================================\n');
fprintf('Initial altitude: %.2f ft (target error ~%.0f ft)\n', h_ft(1), initial_error_ft);
fprintf('Commanded altitude: %d ft\n', target_ft);
fprintf('Time to within 5 ft of command: %.2f s\n', t_settle);
fprintf('Max |elevator|: %.4f (%s)\n', max_elev, tern(elev_saturated,'saturated at some point','no full saturation'));
fprintf('Final altitude (t=%ds): %.2f ft (deviation from command: %.2f ft)\n', ...
    Tsim, h_ft(end), h_ft(end)-target_ft);

figure('Position',[100 100 900 700]);
subplot(2,1,1);
plot(t, h_ft, 'LineWidth', 1.3); hold on;
yline(target_ft, 'k--');
grid on; ylabel('Altitude (ft)'); title('REQ-015: Altitude recovery, final gain-scheduled model (MIL)');

subplot(2,1,2);
plot(t, elevator, 'LineWidth', 1.3); hold on;
yline(1,'r:'); yline(-1,'r:');
grid on; xlabel('Time (s)'); ylabel('Elevator (norm)'); title('Elevator command -- check saturation and duration');

saveas(gcf, 'REQ015_final_model_altitude_recovery.png');
save('REQ015_final_model_results.mat', 't','h_ft','elevator','target_ft', ...
    'initial_error_ft','t_settle','max_elev','elev_saturated','trend_slope');

fprintf('\nSaved: REQ015_final_model_altitude_recovery.png, REQ015_final_model_results.mat\n');
fprintf('Paste the block above back and I''ll fold it into requirements.md.\n');

function s = tern(cond,a,b)
    if cond, s = a; else, s = b; end
end

function newParams = swap_ic_field(origParams, script_basename)
    parts = strsplit(origParams, ',');
    if numel(parts) < 4
        error('Unexpected S-Function Parameters format: %s', origParams);
    end
    parts{4} = ['''scripts/' script_basename ''''];
    newParams = strjoin(parts, ',');
end

function data = get_logged_var(simOut, varname)
    try, data = evalin('base', varname); return; catch, end
    try, data = simOut.get(varname); return; catch, end
    try, data = simOut.(varname); return; catch, end
    error('Could not find logged variable "%s".', varname);
end
