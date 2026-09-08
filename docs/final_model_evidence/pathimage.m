%% GROUND TRACK WITH WAYPOINTS MARKED
% Converts JSBSim's raw lat/lon truth output (val_state cols 8/9) into
% local North/East meters, referenced to the same origin your trim IC
% uses (47.0 N, -122.0 E, from c172p_cruise_init.xml). This puts the
% flown path in the EXACT SAME frame as your L1_Guidance waypoints
% (WP1-WP4 are defined in local N/E meters), so markers line up exactly
% with no lat/lon distortion.
%
% Assumes val_state is already in the base workspace from your last run
% (evalin('base',...) picks it up without re-simulating). If you've
% cleared the workspace, re-run your sim first.

clear lon lat N E

lon = val_state.signals.values(:,8);   % position/long-gc-deg
lat = val_state.signals.values(:,9);   % position/lat-gc-deg

% Origin -- matches c172p_cruise_init.xml exactly
lat0 = 47.0;
lon0 = -122.0;
R_earth = 6378137;  % WGS84 equatorial radius, meters

N = (lat - lat0) * (pi/180) * R_earth;
E = (lon - lon0) * (pi/180) * R_earth * cosd(lat0);

% Waypoints, exactly as defined in L1_Guidance.m
WP1 = [-4500, -4500];
WP2 = [    0, -9000];
WP3 = [ 4500, -4500];
WP4 = [    0,     0];
WP = [WP4; WP1; WP2; WP3; WP4];  % closed loop, same segment order as L1_Guidance

figure('Position',[100 100 900 800]);
hold on; grid on; axis equal;

% Ideal square path (straight-line reference, what a perfect controller
% with zero turn radius would fly)
plot(WP(:,2), WP(:,1), 'w--', 'LineWidth', 1);

% Actual flown path
plot(E, N, 'r', 'LineWidth', 1);

% Waypoint markers, labeled
wp_list = {WP1, WP2, WP3, WP4};
wp_names = {'WP1','WP2','WP3','WP4'};
for k = 1:4
    plot(wp_list{k}(2), wp_list{k}(1), 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
    text(wp_list{k}(2)+150, wp_list{k}(1)+150, wp_names{k}, 'Color', 'y', 'FontWeight', 'bold');
end

xlabel('East (m)'); ylabel('North (m)');
title('Flown path (red) vs. ideal square (white dashed), waypoints marked');
legend('Ideal square path', 'Flown path', 'Waypoints', 'Location', 'best');

saveas(gcf, 'GroundTrack_WithWaypoints.png');
fprintf('Saved: GroundTrack_WithWaypoints.png\n');