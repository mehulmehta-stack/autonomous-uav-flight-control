% States: x = [Alpha (rad), q (rad/s), Theta (rad)]

% Cessna 172 Plant (Radians)
A = [-2.0 1.0 0.0; -5.0 -2.0 0.0; 0.0 1.0 0.0];
B = [-0.1; -10.0; 0.0];
C = [0 0 1];
D = 0;
Ts = 0.008333;

% Discretize for JSBSim (Using your exact 1/120 sample time)
Ts = 0.008333;
sys_d = c2d(ss(A, B, C, D), Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_d);

p_lqr = 50;
Qd = p_lqr * Cd' * Cd; % High penalty on Pitch Error (Theta) to force strict tracking
Rd = 15;                % Lower input penalty so the jet can aggressively use the elevator

% Calculate the K Matrix
K = dlqr(Ad, Bd, Qd, Rd);

disp(K);
