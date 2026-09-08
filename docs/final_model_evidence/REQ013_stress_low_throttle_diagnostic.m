%% REQ-013 DIAGNOSTIC -- is stress_low's failure to recover airspeed a
% physical thrust limit (throttle pinned near max) or a control-authority
% gap (throttle never really tries)? Runs stress_low only, shorter
% window than the full 800s mission, to get a fast, direct answer.

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx';
Tsim = 200;   % first 200s is plenty to see whether throttle is trying

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

newParams = swap_ic_field(origParams, 'c172_cruise_8K_FINAL_FG_stress_low');
set_param(sfBlockPath, 'Parameters', newParams);
fprintf('Running stress_low, %ds, logging throttle + airspeed...\n', Tsim);

simOut = sim(mdl, 'StopTime', num2str(Tsim));

stateData = get_logged_var(simOut, 'val_state');
controlData = get_logged_var(simOut, 'val_control');

set_param(sfBlockPath, 'Parameters', origParams);
close_system(mdl, 0);

t = stateData.time;
u = stateData.signals.values(:,1);
% [Guessing]: column 1 assumed to be Throttle_cmd, based on the pattern
% elsewhere (elevator=4, aileron=2, rudder=5 in this same val_control
% vector) -- not independently confirmed for this specific port. If the
% throttle plot below looks wrong (e.g. bounded outside [0,1], or matches
% a control you already know is a different channel), check the actual
% port order on the JSBSim S-Function block's Control output before
% trusting this result.
throttle = controlData.signals.values(:,1);

figure('Position', [100 100 900 650]);

subplot(2,1,1);
plot(t, u, 'm', 'LineWidth', 1.5);
yline(184.999, 'k--');
grid on; ylabel('True airspeed u (ft/s)');
title('stress\_low: does airspeed ever climb toward cruise?');

subplot(2,1,2);
plot(t, throttle, 'Color', [0 0.5 0], 'LineWidth', 1.5);
yline(1.0, 'r:');
grid on; xlabel('Time (s)'); ylabel('Throttle (norm)');
title('Throttle command -- pinned near 1.0 = physical limit; well below = control gap');
ylim([0 1.05]);

saveas(gcf, 'REQ013_stress_low_throttle_diagnostic.png');

fprintf('\nThrottle: min=%.4f  max=%.4f  mean(last 50%%)=%.4f\n', ...
    min(throttle), max(throttle), mean(throttle(t > Tsim/2)));
fprintf('Airspeed: start=%.2f  end=%.2f  (target 184.999)\n', u(1), u(end));

if mean(throttle(t > Tsim/2)) > 0.95
    fprintf('\n--> Throttle is pinned near max. Likely a real thrust/drag limit\n');
    fprintf('    at this speed+altitude, not a control-law gap.\n');
else
    fprintf('\n--> Throttle is NOT near max despite the airspeed error. Worth\n');
    fprintf('    checking the throttle PI loop''s gain/authority directly.\n');
end

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
