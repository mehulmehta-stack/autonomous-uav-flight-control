%clear; clc;
%
% C172 3-POINT GAIN-SCHEDULED DLQR DESIGN
% ------------------------------------------------------------
% Trim points: 100 / 185 / 220 ft/s TRUE airspeed, 4000 ft altitude.
% A/B extracted from c172_lin_<case>.sce, rows/cols {1,2,3,4} = 0-indexed
% JSBSim native 13-state vector [u alpha theta q rpm v phi p psi ... lat lon alt].
% Longitudinal state order: [delta_u; delta_alpha; delta_theta; q] -- do not permute.
% Lateral state order: [phi; p] -- do not permute.
%
% Q/R weighting is IDENTICAL across all three points (same Bryson scales
% as the original single-point design) -- this is required for the
% gain interpolation across trim points to be meaningful. Do not tune
% per-point unless you have a specific reason and document it.
% ------------------------------------------------------------

Ts = 0.008333;

V_trim_pts = [100, 185, 220];   % ft/s TRUE airspeed, scheduling variable

% ==============================================================
% LONGITUDINAL A/B per trim point
% ==============================================================

A_lon = cell(1,3);
B_lon = cell(1,3);

% --- Case 1: low (100 ft/s) ---
A_lon{1} = [ ...
 -0.070495702305,  13.171050576,  -32.166876084,   0.0061619926727;
 -0.0062471991126, -0.96892757889,  7.3941777128e-07, 0.89720675370;
  1.1003543172e-17, 2.4757968098e-17, -6.2422005548e-17, 1.0000000000;
 -0.0039290907591, -13.233925700,  -7.5340323066e-06, -3.1495963291];
B_lon{1} = [0.9214360865; -0.0651830846; 0; -6.0982175198];

% --- Case 2: cruise (185 ft/s) ---
A_lon{2} = [ ...
 -0.060668365254,  3.2740147743, -32.166876080, -0.016844606210;
 -0.0018549608048, -3.0828659556,  3.7794398674e-07, 0.95743620415;
  0,               -6.5014449028e-19, 4.1702735010e-18, 1.0000000000;
  0.0080872887805, -35.968235270, -1.1114495531e-05, -5.7322819188];
B_lon{2} = [-2.590475857; -0.0990552493; 0; -11.83014053];

% --- Case 3: high (220 ft/s) ---
A_lon{3} = [ ...
 -0.066790512645,  4.3677058533, -32.166876078, -0.018198902500;
 -0.0013172179068, -3.5888924513,  3.1518144240e-07, 0.95743461572;
 -1.6120362080e-20, -1.7430137292e-19, -8.8772773807e-20, 1.0000000000;
  0.010287482951,  -49.977598635, -1.3213607099e-05, -6.8021315815];
B_lon{3} = [-3.6633880541; -0.1177958256; 0; -16.526247885];

% ==============================================================
% LATERAL A/B per trim point
% ==============================================================

A_lat = cell(1,3);
B_lat = cell(1,3);

A_lat{1} = [-2.9162230356e-09, 1.0000000000; 4.2496427673e-07, -3.7950658560];
B_lat{1} = [0; 3.0161535436];

A_lat{2} = [1.7646113159e-11, 1.0000000000; 7.0692729448e-07, -6.9719508589];
B_lat{2} = [0; 10.322889649];

A_lat{3} = [5.4029408262e-13, 1.0000000000; 8.3187290389e-07, -8.2893727157];
B_lat{3} = [0; 14.598546472];

% ==============================================================
% Weighting -- identical to the original single-point design
% ==============================================================

max_du = 10.0;
max_alpha = 0.12;
max_theta = 0.05;
max_q = 0.15;
max_elevator = 0.40;

Q_lon = diag([1/max_du^2, 1/max_alpha^2, 1/max_theta^2, 1/max_q^2]);
R_lon = 1/max_elevator^2;

max_phi = deg2rad(10.0);
max_p = 0.50;
max_aileron = 0.40;

Q_lat = diag([1/max_phi^2, 1/max_p^2]);
R_lat = 1/max_aileron^2;

% ==============================================================
% Design loop
% ==============================================================

K_lon_pts = zeros(3,4);
K_lat_pts = zeros(3,2);

case_names = {'low (100 ft/s)', 'cruise (185 ft/s)', 'high (220 ft/s)'};

fprintf('============================================================\n');
fprintf('3-POINT GAIN-SCHEDULED DLQR DESIGN\n');
fprintf('============================================================\n');

for i = 1:3
    fprintf('\n--- %s ---\n', case_names{i});

    % Longitudinal
    sys_c = ss(A_lon{i}, B_lon{i}, eye(4), zeros(4,1));
    sys_d = c2d(sys_c, Ts, 'zoh');
    [Ad, Bd, ~, ~] = ssdata(sys_d);

    rank_lon = rank(ctrb(Ad, Bd));
    fprintf('  Longitudinal controllability rank = %d / 4\n', rank_lon);
    if rank_lon ~= 4
        error('Longitudinal system NOT controllable at %s', case_names{i});
    end

    K = dlqr(Ad, Bd, Q_lon, R_lon);
    poles = eig(Ad - Bd*K);
    fprintf('  Longitudinal closed-loop pole magnitudes: ');
    fprintf('%.4f ', abs(poles)); fprintf('\n');
    if any(abs(poles) >= 1)
        error('Longitudinal DLQR NOT discrete-time stable at %s', case_names{i});
    end
    K_lon_pts(i,:) = K;

    % Lateral
    sys_c_lat = ss(A_lat{i}, B_lat{i}, eye(2), zeros(2,1));
    sys_d_lat = c2d(sys_c_lat, Ts, 'zoh');
    [Ad_lat, Bd_lat, ~, ~] = ssdata(sys_d_lat);

    rank_lat = rank(ctrb(Ad_lat, Bd_lat));
    fprintf('  Lateral controllability rank = %d / 2\n', rank_lat);
    if rank_lat ~= 2
        error('Lateral system NOT controllable at %s', case_names{i});
    end

    K_lat = dlqr(Ad_lat, Bd_lat, Q_lat, R_lat);
    poles_lat = eig(Ad_lat - Bd_lat*K_lat);
    fprintf('  Lateral closed-loop pole magnitudes: ');
    fprintf('%.4f ', abs(poles_lat)); fprintf('\n');
    if any(abs(poles_lat) >= 1)
        error('Lateral DLQR NOT discrete-time stable at %s', case_names{i});
    end
    K_lat_pts(i,:) = K_lat;
end

fprintf('\n============================================================\n');
fprintf('RESULTS\n');
fprintf('============================================================\n');
disp('V_trim_pts ='); disp(V_trim_pts);
disp('K_lon_pts (rows = trim points, cols = [du, dalpha, dtheta, q]) =');
disp(K_lon_pts);
disp('K_lat_pts (rows = trim points, cols = [phi, p]) =');
disp(K_lat_pts);

% Sanity check: middle row of K_lon_pts / K_lat_pts should match your
% original single-point cruise K within a few percent (it uses the
% identical Q/R and near-identical A/B). If it's off by more than the
% original single-point K itself was off from a same-trim rerun, stop
% and investigate before trusting the schedule.

save('C172_GainSchedule_3pt.mat', ...
    'V_trim_pts', 'K_lon_pts', 'K_lat_pts', ...
    'A_lon', 'B_lon', 'A_lat', 'B_lat', ...
    'Q_lon', 'R_lon', 'Q_lat', 'R_lat', 'Ts');

fprintf('\nSaved: C172_GainSchedule_3pt.mat\n');
fprintf('============================================================\n');
