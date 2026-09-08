function x_hat = Navigation_EKF(imu, gps, gps_valid, dt)
%#codegen
%
% 12-State INS/GPS/Gyro EKF
%
% State:
% x = [N E D VN VE VD phi theta psi p q r]'
%
% IMU:
% imu = [ax ay az p q r]'
%
% GPS:
% gps = [N E D]'
%
% p,q,r are estimated states using the gyro measurements.

GRAVITY = 9.80665;

% =========================================================
% PERSISTENT STATE
% =========================================================

persistent x P initialized

if isempty(initialized)

    x = zeros(12,1);

    % Position from first GPS measurement
    x(1) = gps(1);
    x(2) = gps(2);
    x(3) = gps(3);

    % Initial aircraft velocity
    V0 = 56.4;
    psi0 = deg2rad(225.0);

    x(4) = V0*cos(psi0);
    x(5) = V0*sin(psi0);
    x(6) = 0.0;

    % Initial attitude
    x(7) = 0.0;
    x(8) = 0.0026556;
    x(9) = psi0;

    % Initial body rates from first gyro measurement
    x(10) = imu(4);
    x(11) = imu(5);
    x(12) = imu(6);

    P = diag([ ...
        5.0^2;
        5.0^2;
        5.0^2;
        2.0^2;
        2.0^2;
        2.0^2;
        deg2rad(5.0)^2;
        deg2rad(5.0)^2;
        deg2rad(8.0)^2;
        0.01^2;
        0.01^2;
        0.01^2 ]);

    initialized = true;
end


% =========================================================
% IMU
% =========================================================

ax = imu(1);
ay = imu(2);
az = imu(3);

p = imu(4);
q = imu(5);
r = imu(6);


% =========================================================
% STATE
% =========================================================

phi   = x(7);
theta = x(8);
psi   = x(9);


% =========================================================
% BODY -> NED ROTATION
% =========================================================

cp = cos(phi);
sp = sin(phi);

ct = cos(theta);
st = sin(theta);

cy = cos(psi);
sy = sin(psi);

C_bn = [ ...
    ct*cy, ...
    sp*st*cy - cp*sy, ...
    cp*st*cy + sp*sy;

    ct*sy, ...
    sp*st*sy + cp*cy, ...
    cp*st*sy - sp*cy;

    -st, ...
    sp*ct, ...
    cp*ct ];


% =========================================================
% SPECIFIC FORCE -> NED ACCELERATION
% =========================================================

f_body = [ax; ay; az];

gravity_ned = [0; 0; GRAVITY];

acceleration_ned = C_bn*f_body + gravity_ned;


% =========================================================
% ATTITUDE RATES
% =========================================================

tan_theta = tan(theta);

phi_dot = ...
    p + sp*tan_theta*q + cp*tan_theta*r;

theta_dot = ...
    cp*q - sp*r;

psi_dot = ...
    (sp*q + cp*r)/ct;


% =========================================================
% NONLINEAR STATE DERIVATIVE
% =========================================================

f = zeros(12,1);

f(1) = x(4);
f(2) = x(5);
f(3) = x(6);

f(4) = acceleration_ned(1);
f(5) = acceleration_ned(2);
f(6) = acceleration_ned(3);

f(7) = phi_dot;
f(8) = theta_dot;
f(9) = psi_dot;

% Body rates:
% random-walk model; gyro measurements correct them
f(10) = 0.0;
f(11) = 0.0;
f(12) = 0.0;


% =========================================================
% STATE PREDICTION
% =========================================================

x_pred = x + f*dt;

x_pred(7:9) = ...
    mod(x_pred(7:9) + pi,2*pi) - pi;


% =========================================================
% CONTINUOUS JACOBIAN
% =========================================================

tt = tan(theta);
sec2 = 1.0/(ct*ct);

Fc = zeros(12,12);

% Position
Fc(1,4) = 1.0;
Fc(2,5) = 1.0;
Fc(3,6) = 1.0;


% North acceleration

daN_dphi = ...
    (cp*st*cy + sp*sy)*ay ...
    + (-sp*st*cy + cp*sy)*az;

daN_dtheta = ...
    -st*cy*ax ...
    + sp*ct*cy*ay ...
    + cp*ct*cy*az;

daN_dpsi = ...
    -ct*sy*ax ...
    + (-sp*st*sy - cp*cy)*ay ...
    + (-cp*st*sy + sp*cy)*az;


% East acceleration

daE_dphi = ...
    (cp*st*sy - sp*cy)*ay ...
    + (-sp*st*sy - cp*cy)*az;

daE_dtheta = ...
    -st*sy*ax ...
    + sp*ct*sy*ay ...
    + cp*ct*sy*az;

daE_dpsi = ...
    ct*cy*ax ...
    + (sp*st*cy - cp*sy)*ay ...
    + (cp*st*cy + sp*sy)*az;


% Down acceleration

daD_dphi = ...
    cp*ct*ay ...
    - sp*ct*az;

daD_dtheta = ...
    -ct*ax ...
    - sp*st*ay ...
    - cp*st*az;


Fc(4,7) = daN_dphi;
Fc(4,8) = daN_dtheta;
Fc(4,9) = daN_dpsi;

Fc(5,7) = daE_dphi;
Fc(5,8) = daE_dtheta;
Fc(5,9) = daE_dpsi;

Fc(6,7) = daD_dphi;
Fc(6,8) = daD_dtheta;


% Attitude derivatives

Fc(7,7) = ...
    cp*tt*q - sp*tt*r;

Fc(7,8) = ...
    (sp*q + cp*r)*sec2;

Fc(8,7) = ...
    -sp*q - cp*r;

Fc(9,7) = ...
    (cp*q - sp*r)/ct;

Fc(9,8) = ...
    (sp*q + cp*r)*sec2/ct;


% =========================================================
% DISCRETE TRANSITION
% =========================================================

Fd = eye(12) + Fc*dt;


% =========================================================
% PROCESS NOISE
% =========================================================

Q = diag([ ...
    0.001^2;
    0.001^2;
    0.001^2;
    0.04^2;
    0.04^2;
    0.04^2;
    deg2rad(0.03)^2;
    deg2rad(0.03)^2;
    deg2rad(0.03)^2;
    0.005^2;
    0.005^2;
    0.005^2 ]);


% =========================================================
% COVARIANCE PREDICTION
% =========================================================

P_pred = Fd*P*Fd' + Q;

P_pred = 0.5*(P_pred + P_pred');


% =========================================================
% MEASUREMENT MODEL
%
% z = [GPS_N GPS_E GPS_D gyro_p gyro_q gyro_r]'
% =========================================================

H = zeros(6,12);

H(1,1) = 1.0;
H(2,2) = 1.0;
H(3,3) = 1.0;

H(4,10) = 1.0;
H(5,11) = 1.0;
H(6,12) = 1.0;


z = [ ...
    gps(1);
    gps(2);
    gps(3);
    p;
    q;
    r ];


z_pred = H*x_pred;

innovation = z - z_pred;


% =========================================================
% MEASUREMENT NOISE
% =========================================================

R = diag([ ...
    2.0^2;
    2.0^2;
    3.0^2;
    0.002^2;
    0.002^2;
    0.002^2 ]);


% =========================================================
% GPS VALIDITY
%
% If GPS is unavailable, ignore the first 3 measurements.
% Gyro measurements remain active.
% =========================================================

if gps_valid == 0

    H_use = H(4:6,:);
    z_use = z(4:6);
    z_pred_use = z_pred(4:6);
    R_use = R(4:6,4:6);

    innovation_use = z_use - z_pred_use;

    S = H_use*P_pred*H_use' + R_use;

    K = P_pred*H_use'/S;

    x = x_pred + K*innovation_use;

    I12 = eye(12);
    A = I12 - K*H_use;

    P = A*P_pred*A' + K*R_use*K';

else

    S = H*P_pred*H' + R;

    K = P_pred*H'/S;

    x = x_pred + K*innovation;

    I12 = eye(12);
    A = I12 - K*H;

    P = A*P_pred*A' + K*R*K';

end


% =========================================================
% COVARIANCE SYMMETRY
% =========================================================

P = 0.5*(P + P');


% =========================================================
% ANGLE WRAP
% =========================================================

x(7:9) = ...
    mod(x(7:9) + pi,2*pi) - pi;


% =========================================================
% OUTPUT
% =========================================================

x_hat = x;

end