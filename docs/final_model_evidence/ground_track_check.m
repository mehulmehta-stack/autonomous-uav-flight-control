%% GROUND TRACK CHECK -- reuses val_state already in the workspace,
% does NOT re-run the simulation. val_state columns 8/9 are
% position/long-gc-deg and position/lat-gc-deg per c172_simulink.xml.

lon = val_state.signals.values(:,8);
lat = val_state.signals.values(:,9);

figure('Position',[100 100 900 700]);
plot(lon, lat, 'r', 'LineWidth', 1);
axis equal; grid on;
xlabel('Longitude (deg)'); ylabel('Latitude (deg)');
title('Ground track, corrected L1 code (phi_max=17 deg) -- compare against the earlier S-wiggle plot');
saveas(gcf, 'L1_GroundTrack_AfterFix.png');

fprintf('Saved: L1_GroundTrack_AfterFix.png\n');
fprintf('Compare this directly against your earlier ground-track plot.\n');
fprintf('Same crossing-loop shape at the same two corners = phi_max fix did not\n');
fprintf('change it, keep investigating. Gone or visibly smaller = the phi_max\n');
fprintf('correction incidentally fixed it, and speed was never the mechanism.\n');