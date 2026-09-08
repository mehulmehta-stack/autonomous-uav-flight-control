%% L1 ETA DISCONTINUITY CHECK -- all 8 switch events, zoomed, side by side
clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withoutNoise_GnSch.slx';
Tsim = 1200;

load_system(mfile);
[~, mdl, ~] = fileparts(mfile);
simOut = sim(mdl, 'StopTime', num2str(Tsim));

stateData = get_logged_var(simOut, 'val_state');
wpData    = get_logged_var(simOut, 'val_L1wpidx');
etaData   = get_logged_var(simOut, 'val_L1eta');
phiCData  = get_logged_var(simOut, 'val_L1phicmd');

t = stateData.time;
wpidx = wpData.signals.values(:,1);
eta_log = etaData.signals.values(:,1);
phicmd_log = phiCData.signals.values(:,1);

switch_idx = find(diff(wpidx) ~= 0) + 1;
window = 15; % seconds each side of the switch

figure('Position',[50 50 1400 800]);
for k = 1:numel(switch_idx)
    t_sw = t(switch_idx(k));
    mask = t >= (t_sw - window) & t <= (t_sw + window);

    subplot(2, ceil(numel(switch_idx)/2), k);
    plot(t(mask) - t_sw, rad2deg(eta_log(mask)), 'y', 'LineWidth', 1.2); hold on;
    plot(t(mask) - t_sw, rad2deg(phicmd_log(mask)), 'c', 'LineWidth', 1);
    xline(0, 'w:');
    grid on;
    title(sprintf('wp%d->%d @ t=%.0fs', wpidx(switch_idx(k)-1), wpidx(switch_idx(k)), t_sw));
    if k == 1
        legend('\eta (deg)','\phi_{cmd} (deg)','Location','best');
    end
    xlabel('t - t_{switch} (s)');
end
sgtitle('L1 guidance angle and bank command, zoomed around every waypoint switch');
saveas(gcf, 'L1_Eta_Discontinuity_Check.png');

fprintf('\nLook for: a sharp jump or reversal in eta right at t=0 in some\n');
fprintf('panels but not others. That panel is your S-wiggle corner.\n');
fprintf('A smooth, single-sign eta transition in all 8 panels means the\n');
fprintf('guidance law itself is clean, and the wiggle is downstream --\n');
fprintf('in the DLQR gain response to phi_cmd, not in L1.\n');

function data = get_logged_var(simOut, varname)
    try, data = evalin('base', varname); return; catch, end
    try, data = simOut.get(varname); return; catch, end
    try, data = simOut.(varname); return; catch, end
    error('Could not find logged variable "%s".', varname);
end