%% ALTERNATIVE 4: STATE-SPACE (LQR)
% Concept: Abandoning transfer functions. We define internal states and minimize a cost function.

% Redefine the plant using State-Space matrices (A, B, C, D)
A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = 0;
t = 0:0.01:10;

% Set up LQR weighting matrices
p_lqr = 50; 
Q = p_lqr * C' * C; % Penalize output error
R = 1;              % Penalize control effort

% Compute optimal state-feedback gain matrix K
K_ss = lqr(A, B, Q, R);

% Since this is full-state feedback, we need a precompensator (Nbar) 
% to fix the steady-state tracking error. (Calculated via tutorial logic).
Nbar_ss = 7.0711;

% Build the closed-loop state-space system
sys_cl_ss = ss(A - B*K_ss, B*Nbar_ss, C, D);

figure;
step(0.2 * sys_cl_ss, t);
title('Alternative 4: State-Space (LQR) Step Response');
ylabel('Pitch Angle (rad)');
grid on;