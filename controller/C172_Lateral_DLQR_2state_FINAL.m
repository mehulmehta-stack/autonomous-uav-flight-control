%% C172 LATERAL 2-STATE DISCRETE LQR
% ------------------------------------------------------------
% Purpose:
%   First proper lateral/roll controller for the C172.
%
% Architecture:
%
%   phi_cmd
%      |
%      v
%   e = [phi - phi_cmd; p]
%      |
%      v
%   delta_a = -K*e
%      |
%      v
%   C172 lateral roll dynamics
%
% State order:
%   x_lat = [phi; p]
%
%   phi = bank angle [rad]
%   p   = roll rate [rad/s]
%
% Input:
%   delta_a = aileron command perturbation
%
% The A/B entries below are taken directly from the verified
% native JSBSim 13-state linearization c172_lin.sce.
%
% Native JSBSim input ordering:
%   1 = throttle
%   2 = aileron
%   3 = elevator
%   4 = rudder
%
% Therefore B(:,2) is the aileron column.
%
% Sample time:
%   0.008333 s
%
% This script does NOT modify the existing Simulink model.
% First validate the lateral controller independently.
% ------------------------------------------------------------

%bdclose('all');
%clearvars;
%clear;
%clc;
%close all;


%% -----------------------------------------------------------
% 1. Exact lateral states extracted from JSBSim native model
% ------------------------------------------------------------

% Full native lateral state subset:
%
%   x_lat = [phi; p]
%
% Continuous-time A_lat:
A_lat = [ ...
     1.7646113159e-11,   1.0000000000;
     7.0692729448e-07,  -6.9719508589 ];

% Continuous-time B_lat for JSBSim input #2 = aileron:
B_lat = [ ...
     0.0000000000;
     10.322889649 ];

Ts = 0.008333;

%% -----------------------------------------------------------
% 2. Operating/reference definitions
% -----------------------------------------------------------

phi_trim = 0.0;
p_trim   = 0.0;

% LQR design limits used for Bryson weighting.
% These are design scales, not actuator limits.

max_phi     = deg2rad(10.0);   % 10 deg bank
max_p       = 0.50;            % rad/s
max_aileron = 0.40;            % normalized aileron perturbation

Q = diag([ ...
    1/max_phi^2, ...
    1/max_p^2]);

R = 1/max_aileron^2;

%% -----------------------------------------------------------
% 3. Discretize exactly as for the longitudinal DLQR
% -----------------------------------------------------------

sys_c = ss(A_lat, B_lat, eye(2), zeros(2,1));

sys_d = c2d(sys_c, Ts, 'zoh');

[Ad, Bd, ~, ~] = ssdata(sys_d);

%% -----------------------------------------------------------
% 4. Controllability check
% -----------------------------------------------------------

Co = ctrb(Ad, Bd);
controllability_rank = rank(Co);

fprintf('\n============================================================\n');
fprintf('C172 LATERAL 2-STATE DLQR\n');
fprintf('============================================================\n');

fprintf('State order:\n');
fprintf('  x_lat = [phi; p]\n\n');

fprintf('Input:\n');
fprintf('  delta_a = aileron command perturbation\n\n');

fprintf('Sample time: %.6f s\n\n', Ts);

disp('A_lat =');
disp(A_lat);

disp('B_lat =');
disp(B_lat);

disp('Q =');
disp(Q);

disp('R =');
disp(R);

disp('Ad =');
disp(Ad);

disp('Bd =');
disp(Bd);

fprintf('Controllability rank = %d / 2\n', controllability_rank);

if controllability_rank ~= 2
    error('Lateral [phi,p] system is NOT controllable.');
end

%% -----------------------------------------------------------
% 5. Discrete LQR
% -----------------------------------------------------------

K_lat = dlqr(Ad, Bd, Q, R);

Acl = Ad - Bd*K_lat;

closed_loop_poles = eig(Acl);

disp('K_lat =');
disp(K_lat);

disp('Closed-loop poles =');
disp(closed_loop_poles);

disp('Closed-loop pole magnitudes =');
disp(abs(closed_loop_poles));

if any(abs(closed_loop_poles) >= 1)
    error('Closed-loop lateral DLQR is NOT discrete-time stable.');
end

%% -----------------------------------------------------------
% 6. Independent +10 deg roll-command test
% -----------------------------------------------------------

Tsim = 20.0;
N = round(Tsim/Ts) + 1;

t = (0:N-1)' * Ts;

phi_cmd = zeros(N,1);

% Step to +5 deg at 2 seconds.
phi_cmd(t >= 2.0) = deg2rad(5.0);

x = zeros(2,1);

x_log = zeros(N,2);
u_log = zeros(N,1);

for k = 1:N

    % Reference state:
    x_ref = [phi_cmd(k); 0];

    % Tracking error:
    e = x - x_ref;

    % LQR control law:
    delta_a = -K_lat * e;

    % Aileron perturbation saturation.
    delta_a = min(max(delta_a, -max_aileron), max_aileron);

    % Log current state and command.
    x_log(k,:) = x.';
    u_log(k) = delta_a;

    % Plant propagation.
    if k < N
        x = Ad*x + Bd*delta_a;
    end
end

phi = x_log(:,1);
p   = x_log(:,2);

%% -----------------------------------------------------------
% 7. Basic validation numbers
% -----------------------------------------------------------

step_start = find(t >= 2.0, 1, 'first');
final_window = t >= 15.0;

phi_final_deg = rad2deg(mean(phi(final_window)));
phi_cmd_final_deg = rad2deg(mean(phi_cmd(final_window)));

phi_error_final_deg = phi_final_deg - phi_cmd_final_deg;

fprintf('\nIndependent +5 deg command test:\n');
fprintf('  Commanded bank      : %.3f deg\n', phi_cmd_final_deg);
fprintf('  Final mean bank     : %.3f deg\n', phi_final_deg);
fprintf('  Final bank error    : %.3f deg\n', phi_error_final_deg);
fprintf('  Maximum |p|         : %.6f rad/s\n', max(abs(p)));
fprintf('  Maximum |delta_a|   : %.6f normalized\n', max(abs(u_log)));

%% -----------------------------------------------------------
% 8. Save controller data
% -----------------------------------------------------------

save('C172_Lateral_DLQR_2state_FINAL.mat', ...
    'A_lat', 'B_lat', 'Ad', 'Bd', ...
    'K_lat', 'Q', 'R', 'Ts', ...
    'phi_trim', 'p_trim', ...
    'max_phi', 'max_p', 'max_aileron');

disp('Saved: C172_Lateral_DLQR_2state_FINAL.mat');

%% -----------------------------------------------------------
% 9. Save plots
% -----------------------------------------------------------

plot_dir = fullfile(pwd, 'C172_Lateral_DLQR_Plots');

if ~exist(plot_dir, 'dir')
    mkdir(plot_dir);
end

figure('Name','C172 Lateral DLQR - Bank Tracking');

plot(t, rad2deg(phi), 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(phi_cmd), '--', 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Bank angle \phi (deg)');
title('C172 Lateral DLQR — Bank Tracking');
legend('\phi actual', '\phi command', 'Location', 'best');

saveas(gcf, fullfile(plot_dir, ...
    'C172_Lateral_DLQR_BankTracking.png'));

figure('Name','C172 Lateral DLQR - Roll Rate');

plot(t, p, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Roll rate p (rad/s)');
title('C172 Lateral DLQR — Roll Rate');

saveas(gcf, fullfile(plot_dir, ...
    'C172_Lateral_DLQR_RollRate.png'));

figure('Name','C172 Lateral DLQR - Aileron');

plot(t, u_log, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('\delta_a (normalized)');
title('C172 Lateral DLQR — Aileron Command');

saveas(gcf, fullfile(plot_dir, ...
    'C172_Lateral_DLQR_Aileron.png'));

fprintf('\nPlots saved to:\n  %s\n', plot_dir);

fprintf('\n============================================================\n');
fprintf('LATERAL DLQR DESIGN COMPLETE\n');
fprintf('============================================================\n');
