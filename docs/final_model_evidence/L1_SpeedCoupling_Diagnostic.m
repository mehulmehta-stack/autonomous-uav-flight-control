%% DIAGNOSTIC: Log speed, eta, phi_cmd around each waypoint transition
% Tests whether the WP2/WP4 S-wiggle correlates with elevated ground
% speed at those specific corners (a_cmd scales with V^2 in L1_Guidance).
% Add three To Workspace blocks tapped directly on the L1_Guidance
% MATLAB Function block's outputs (a_cmd, eta, phi_cmd) plus one more
% tapping its N/E inputs -- OR, simpler: add a 4th signal to your
% existing val_state log if speed isn't already derivable from it.
%
% This assumes you add a MATLAB Function output port exposing
% 'speed' (sqrt(VN^2+VE^2)) and 'waypoint_index' as debug outputs --
% two extra output arguments on L1_Guidance, wired to two new
% To Workspace blocks: val_L1speed, val_L1wpidx.

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withoutNoise_GnSch.slx';
Tsim = 1200;

load_system(mfile);
[~, mdl, ~] = fileparts(mfile);

simOut = sim(mdl, 'StopTime', num2str(Tsim));

stateData = get_logged_var(simOut, 'val_state');
speedData = get_logged_var(simOut, 'val_L1speed');
wpData    = get_logged_var(simOut, 'val_L1wpidx');

t = stateData.time;
phi = stateData.signals.values(:,10);
speed = speedData.signals.values(:,1);
wpidx = wpData.signals.values(:,1);

% Detect waypoint-switch events (wpidx changes)
switch_idx = find(diff(wpidx) ~= 0) + 1;

figure('Position',[100 100 900 700]);
subplot(3,1,1);
plot(t, rad2deg(phi), 'm'); grid on; hold on;
for k = 1:numel(switch_idx)
    xline(t(switch_idx(k)), 'k:');
end
ylabel('\phi (deg)'); title('Bank angle with waypoint-switch markers');

subplot(3,1,2);
plot(t, speed*3.28084, 'c'); grid on; hold on;   % m/s -> ft/s
for k = 1:numel(switch_idx)
    xline(t(switch_idx(k)), 'k:');
end
ylabel('Speed (ft/s)'); title('Ground speed -- check if elevated at WP2/WP4 switches specifically');

subplot(3,1,3);
plot(t, wpidx, 'g'); grid on;
xlabel('Time (s)'); ylabel('Active waypoint index (1-4)');
title('Waypoint index -- identifies WHICH switch is which');

saveas(gcf, 'L1_SpeedCoupling_Diagnostic.png');

fprintf('\nAt each switch event, print speed just before/after:\n');
for k = 1:numel(switch_idx)
    idx = switch_idx(k);
    fprintf('  t=%.1fs  wp %d->%d  speed before=%.2f ft/s  speed after=%.2f ft/s\n', ...
        t(idx), wpidx(idx-1), wpidx(idx), speed(idx-1)*3.28084, speed(idx)*3.28084);
end

function data = get_logged_var(simOut, varname)
    try, data = evalin('base', varname); return; catch, end
    try, data = simOut.get(varname); return; catch, end
    try, data = simOut.(varname); return; catch, end
    error('Could not find logged variable "%s". Did you add the debug output ports and To Workspace blocks?', varname);
end