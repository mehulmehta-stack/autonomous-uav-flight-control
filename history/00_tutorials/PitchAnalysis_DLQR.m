% Redefine the plant using State-Space matrices (A, B, C, D)
A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = 0;
t = 0:0.01:10;
sys_ss = ss(A, B, C, D);

%% Discrete rscale — same idea, discrete-time steady-state equations
function Nbar = rscale_d(Ad,Bd,Cd,Dd,K)
    n = size(Ad,1);
    z = [zeros(n,1); 1];
    N = [eye(n)-Ad, -Bd; Cd, Dd] \ z;
    Nx = N(1:n); Nu = N(n+1);
    Nbar = Nu + K*Nx;
end

Ts = 0.01;
sys_d = c2d(sys_ss, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_d);

p_lqr = 50;
Qd = p_lqr * Cd' * Cd;
Rd = 1;
Kd_gain = dlqr(Ad, Bd, Qd, Rd);
Nbar_d = rscale_d(Ad,Bd,Cd,Dd,Kd_gain);   % = 6.9555, computed not guessed

sys_cl_d = ss(Ad - Bd*Kd_gain, Bd*Nbar_d, Cd, Dd, Ts);
figure; step(0.2*sys_cl_d, t); title('Discrete LQR - properly computed Nbar');
ylabel('Pitch Angle (rad)'); grid on;


% --- OUTER LOOP KINEMATICS ---
% Your state-space matrices model aerodynamics, but they do not contain the forward airspeed of the aircraft. 
% To calculate altitude from pitch, you must assume a constant forward velocity. 
% Add this to the bottom of your existing script:

% [Guessing] I am assigning 100 m/s (approx 200 knots) as a standard 
% cruising speed for a generic commercial aircraft model. 
V = 100; 

% Initialize placeholder PID gains so Simulink does not throw an error
Kp = 0; 
Ki = 0; 
Kd = 0;