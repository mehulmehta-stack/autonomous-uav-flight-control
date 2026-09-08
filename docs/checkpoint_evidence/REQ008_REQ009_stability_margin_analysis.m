%% REQ-008 / REQ-009
% C172P 4-State DLQR Stability Margin Analysis
%
% Requirements:
% REQ-008: Closed-loop phase margin > 30 deg
% REQ-009: Closed-loop gain margin > 6 dB
%
% State order:
% x = [delta_u; delta_alpha; delta_theta; q]
%
% Controller:
% delta_e = -K*x
%
% For the single-input state-feedback system, the return ratio is:
%
% L(z) = K * (zI - Ad)^(-1) * Bd
%
% The MATLAB state-space representation below creates exactly this
% SISO return-ratio system.

clear;
clc;
close all;

%% ============================================================
% 1. Continuous-time aircraft model
% =============================================================

A = [ ...
 -0.060668363940, 3.2740147717, -32.166876080, -0.016844605405;
 -0.0027464567573, -3.0828659556, 3.7794562027e-07, 0.95743620415;
 0, -2.0384228900e-18, 5.9219042629e-18, 1.0000000000;
 0.0099808985828, -35.9682352700, -1.1114723170e-05, -5.7322819189];

B = [ ...
 -2.5904758571;
 -0.099055249294;
 0;
 -11.8301405300];

%% ============================================================
% 2. Controller sample time
% =============================================================

Ts = 0.008333;       % seconds
Fs = 1/Ts;           % controller frequency

fprintf('Sample time: %.9f s\n', Ts);
fprintf('Sample frequency: %.3f Hz\n\n', Fs);

%% ============================================================
% 3. Discretize aircraft model
% =============================================================

sys_c = ss(A,B,eye(4),zeros(4,1));

sys_d = c2d(sys_c,Ts,'zoh');

[Ad,Bd,~,~] = ssdata(sys_d);

%% ============================================================
% 4. Reconstruct DLQR controller
% =============================================================

max_du      = 10.0;
max_alpha   = 0.12;
max_theta   = 0.05;
max_q       = 0.15;
max_elevator = 0.40;

Q = diag([ ...
    1/max_du^2, ...
    1/max_alpha^2, ...
    1/max_theta^2, ...
    1/max_q^2]);

R = 1/max_elevator^2;

K = dlqr(Ad,Bd,Q,R);

fprintf('DLQR gain K:\n');
disp(K);

%% ============================================================
% 5. Nominal closed-loop stability
% =============================================================

Acl = Ad - Bd*K;

poles = eig(Acl);

fprintf('\nClosed-loop poles:\n');
disp(poles);

fprintf('Pole magnitudes:\n');
disp(abs(poles));

if all(abs(poles) < 1)
    fprintf('Nominal discrete-time stability: PASS\n');
else
    fprintf('Nominal discrete-time stability: FAIL\n');
end

%% ============================================================
% 6. Construct state-feedback return ratio
% =============================================================
%
% L(z) = K * (zI-Ad)^(-1) * Bd
%
% This is represented as:
%
% input  = elevator perturbation
% output = K*x
%
% with:
%
% x(k+1) = Ad*x(k) + Bd*u(k)
% y(k)   = K*x(k)

L = ss(Ad,Bd,K,0,Ts);

%% ============================================================
% 7. Gain and phase margins
% =============================================================

[Gm,Pm,Wcg,Wcp] = margin(L);

fprintf('\n=============================================\n');
fprintf('STABILITY MARGIN RESULTS\n');
fprintf('=============================================\n');

fprintf('Gain Margin (absolute): %g\n',Gm);

if isinf(Gm)
    fprintf('Gain Margin: Infinity dB\n');
else
    Gm_dB = 20*log10(Gm);
    fprintf('Gain Margin: %.6f dB\n',Gm_dB);
end

fprintf('Phase Margin: %.6f deg\n',Pm);

fprintf('Gain crossover frequency: %.6f rad/s\n',Wcp);
fprintf('Phase crossover frequency: %.6f rad/s\n',Wcg);

%% ============================================================
% 8. Convert gain margin to dB
% =============================================================

if isinf(Gm)
    Gm_dB = Inf;
else
    Gm_dB = 20*log10(Gm);
end

%% ============================================================
% 9. Requirement checks
% =============================================================

REQ008_limit = 30;      % degrees
REQ009_limit = 6;       % dB

REQ008_PASS = Pm > REQ008_limit;
REQ009_PASS = Gm_dB > REQ009_limit;

fprintf('\n=============================================\n');
fprintf('REQUIREMENT VERIFICATION\n');
fprintf('=============================================\n');

fprintf('REQ-008: Phase Margin = %.6f deg > %.2f deg\n', ...
    Pm, REQ008_limit);

if REQ008_PASS
    fprintf('REQ-008 RESULT: PASS\n');
else
    fprintf('REQ-008 RESULT: FAIL\n');
end

fprintf('\nREQ-009: Gain Margin = %.6f dB > %.2f dB\n', ...
    Gm_dB, REQ009_limit);

if REQ009_PASS
    fprintf('REQ-009 RESULT: PASS\n');
else
    fprintf('REQ-009 RESULT: FAIL\n');
end

%% ============================================================
% 10. Plot margin diagram
% =============================================================

figure;

margin(L);
grid on;

title('C172P DLQR State-Feedback Stability Margins');

%% ============================================================
% 11. Save analysis data
% =============================================================

save('REQ008_REQ009_stability_margin_results.mat', ...
    'A','B','Ad','Bd','K','Q','R','Ts', ...
    'Acl','poles','Gm','Gm_dB','Pm','Wcg','Wcp');

fprintf('\nResults saved to:\n');
fprintf('REQ008_REQ009_stability_margin_results.mat\n');