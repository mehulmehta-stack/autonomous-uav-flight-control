clc;
%% PLOT FLOWN PATH VS IDEAL 4-WAYPOINT REFERENCE
N = squeeze(ekf_N_log.signals.values);
E = squeeze(ekf_E_log.signals.values);
t = squeeze(ekf_N_log.time);

% Ideal path from L1_Guidance.m, closed back to WP1 for plotting
WP_N = [-4500,     0,  4500,     0, -4500];  % WP1, WP2, WP3, WP4, close loop
WP_E = [-4500, -9000, -4500,     0, -4500];

figure; hold on; axis equal; grid on;
plot(E, N, 'Color', [0.6 0 0.3], 'LineWidth', 1.5, 'DisplayName', 'Flown path (EKF)');
plot(WP_E, WP_N, 'b--o', 'LineWidth', 1.2, 'MarkerSize', 8, ...
'MarkerFaceColor', 'y', 'DisplayName', 'Ideal 4-waypoint path');
labels = {'WP1','WP2','WP3','WP4'};
text(WP_E(1:4) + 150, WP_N(1:4) + 150, labels, 'FontWeight', 'bold', 'FontSize', 10);
xlabel('East (m)'); ylabel('North (m)');
title('Flown path vs. 4-waypoint reference (phi\_max = 16 deg)');
legend('Location', 'best');

figure; hold on; axis equal; grid on;
scatter(E, N, 4, (1:length(E))', 'filled');  % color = time progression
colormap(jet); colorbar;
plot(WP_E, WP_N, 'k--o', 'LineWidth', 1.2, 'MarkerSize', 8, 'MarkerFaceColor', 'y');
labels = {'WP1','WP2','WP3','WP4'};
text(WP_E(1:4)+150, WP_N(1:4)+150, labels, 'FontWeight','bold');
xlabel('East (m)'); ylabel('North (m)');
title('Flown path colored by time -- converging or repeating?');

%% EKF POSITION ERROR VS GROUND TRUTH (Synthetic_GPS, noiseless)
raw = gps_truth_log.signals.values;
sz = size(raw);
Tlen = numel(N);

if sz(1) == 3 && sz(end) == Tlen
    gps_truth = reshape(raw, 3, Tlen).';      % 3 x ... x Tlen  -> Tlen x 3
elseif sz(end) == 3 && sz(1) == Tlen
    gps_truth = reshape(raw, Tlen, 3);        % Tlen x ... x 3  -> Tlen x 3
else
    error(['gps_truth_log size %s does not match ekf log length %d in either ' ...
           'dimension -- did you rerun the sim with a different Tsim between ' ...
           'logging the two signals?'], mat2str(sz), Tlen);
end

truth_N = gps_truth(:,1);
truth_E = gps_truth(:,2);

pos_error = sqrt((N - truth_N).^2 + (E - truth_E).^2);

fprintf('Position error, full run: mean=%.2f m, max=%.2f m\n', ...
    mean(pos_error), max(pos_error));

outage = t >= 25 & t < 35;
fprintf('Position error during GPS outage (t=25-35s): mean=%.2f m, max=%.2f m\n', ...
    mean(pos_error(outage)), max(pos_error(outage)));

figure; hold on; grid on;
plot(t, pos_error, 'LineWidth', 1.2);
xline(25, 'r--'); xline(35, 'r--');
xlabel('Time (s)'); ylabel('Position error (m)');
title('EKF position error vs. ground truth (GPS outage window marked)');

gps_valid_log_check = squeeze(gps_valid_log.signals.values);  % if you've logged it; if not, add a To Workspace tap on GPS_Validity's output now
fprintf('gps_valid during outage window: min=%.0f, max=%.0f, mean=%.3f\n', ...
    min(gps_valid_log_check(outage)), max(gps_valid_log_check(outage)), mean(gps_valid_log_check(outage)));