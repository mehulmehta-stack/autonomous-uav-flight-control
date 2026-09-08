%% REQ-007 — 30% Model Parameter Uncertainty Analysis
%
% Requirement:
%   REQ-007: Controller stable with 30% model parameter uncertainty
%
% Verification method:
%   Analysis
%
% Method:
%   1. Start from the nominal continuous-time C172P model A, B.
%   2. Randomly perturb every physically meaningful non-zero A/B
%      coefficient independently within +/-30%.
%   3. Discretize each perturbed model using the nominal sample time.
%   4. Apply the SAME nominal DLQR gain K.
%   5. Calculate the discrete-time closed-loop poles.
%   6. A trial is stable if every pole satisfies |lambda| < 1.
%   7. Repeat for 10,000 trials.
%
% Important:
%   This is a Monte Carlo robustness analysis, not a formal proof
%   of robust stability for every possible uncertainty realization. 
%   It is a defined numerical robustness analysis. That's the correct level of claim for the portfolio.
%
% State order:
%   x = [delta_u; delta_alpha; delta_theta; q]
%
% Controller:
%   delta_e = -K*x

clear;
clc;
close all;

%% 1. Nominal continuous-time aircraft model

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

%% 2. Controller sample time

Ts = 0.008333;     % seconds

fprintf('=============================================\n');
fprintf('REQ-007: 30%% MODEL UNCERTAINTY ANALYSIS\n');
fprintf('=============================================\n\n');

fprintf('Sample time: %.9f s\n', Ts);

%% ============================================================
% 3. DLQR weighting matrices
% =============================================================

max_du        = 10.0;
max_alpha     = 0.12;
max_theta     = 0.05;
max_q         = 0.15;
max_elevator  = 0.40;

Q = diag([ ...
    1/max_du^2, ...
    1/max_alpha^2, ...
    1/max_theta^2, ...
    1/max_q^2]);

R = 1/max_elevator^2;

%% ============================================================
% 4. Nominal discrete model
% =============================================================

sys_c = ss(A,B,eye(4),zeros(4,1));

sys_d = c2d(sys_c,Ts,'zoh');

[Ad,Bd,~,~] = ssdata(sys_d);

%% ============================================================
% 5. Calculate nominal DLQR controller
% =============================================================

[K,~,~] = dlqr(Ad,Bd,Q,R);

fprintf('\nNominal DLQR gain K:\n');
disp(K);

%% ============================================================
% 6. Nominal closed-loop poles
% =============================================================

Acl_nominal = Ad - Bd*K;

poles_nominal = eig(Acl_nominal);

pole_mag_nominal = abs(poles_nominal);

fprintf('Nominal closed-loop poles:\n');
disp(poles_nominal);

fprintf('Nominal pole magnitudes:\n');
disp(pole_mag_nominal);

fprintf('Nominal maximum pole magnitude: %.12f\n', ...
    max(pole_mag_nominal));

if all(pole_mag_nominal < 1)
    fprintf('Nominal stability: PASS\n');
else
    fprintf('Nominal stability: FAIL\n');
end

%% ============================================================
% 7. Monte Carlo settings
% =============================================================

N = 10000;

uncertainty = 0.30;

% Reproducible random sequence
rng(2026);

fprintf('\nMonte Carlo trials: %d\n', N);
fprintf('Coefficient uncertainty: +/- %.0f%%\n', ...
    uncertainty*100);

%% ============================================================
% 8. Identify meaningful non-zero coefficients
% ============================================================
%
% Extremely small values such as 1e-18 in the model are numerical
% round-off terms rather than meaningful aerodynamic derivatives.
%
% We therefore use a threshold so that numerical zero terms are not
% treated as physical model parameters.

threshold = 1e-12;

A_mask = abs(A) > threshold;
B_mask = abs(B) > threshold;

fprintf('\nPerturbed A coefficients: %d\n', nnz(A_mask));
fprintf('Perturbed B coefficients: %d\n', nnz(B_mask));

%% ============================================================
% 9. Allocate result storage
% =============================================================

max_pole_magnitude = zeros(N,1);

unstable_trials = false(N,1);

worst_trial = 0;

%% ============================================================
% 10. Monte Carlo uncertainty experiment
% =============================================================

fprintf('\nRunning Monte Carlo analysis...\n');

for i = 1:N

    % ---------------------------------------------------------
    % Create perturbed A matrix
    % ---------------------------------------------------------

    A_uncertain = A;

    random_A = 1 + uncertainty * (2*rand(size(A)) - 1);

    A_uncertain(A_mask) = ...
        A(A_mask) .* random_A(A_mask);

    % ---------------------------------------------------------
    % Create perturbed B matrix
    % ---------------------------------------------------------

    B_uncertain = B;

    random_B = 1 + uncertainty * (2*rand(size(B)) - 1);

    B_uncertain(B_mask) = ...
        B(B_mask) .* random_B(B_mask);

    % ---------------------------------------------------------
    % Discretize perturbed aircraft model
    % ---------------------------------------------------------

    sys_uncertain_c = ss( ...
        A_uncertain, ...
        B_uncertain, ...
        eye(4), ...
        zeros(4,1));

    sys_uncertain_d = c2d( ...
        sys_uncertain_c, ...
        Ts, ...
        'zoh');

    [Ad_uncertain,Bd_uncertain,~,~] = ...
        ssdata(sys_uncertain_d);

    % ---------------------------------------------------------
    % Apply SAME nominal controller
    % ---------------------------------------------------------

    Acl_uncertain = ...
        Ad_uncertain - Bd_uncertain*K;

    % ---------------------------------------------------------
    % Calculate closed-loop poles
    % ---------------------------------------------------------

    poles_uncertain = eig(Acl_uncertain);

    pole_magnitudes = abs(poles_uncertain);

    max_pole_magnitude(i) = ...
        max(pole_magnitudes);

    % ---------------------------------------------------------
    % Stability test
    % ---------------------------------------------------------

    unstable_trials(i) = ...
        any(pole_magnitudes >= 1);

end

%% ============================================================
% 11. Calculate final statistics
% =============================================================

unstable_count = sum(unstable_trials);

stable_count = N - unstable_count;

worst_pole = max(max_pole_magnitude);

best_pole = min(max_pole_magnitude);

mean_max_pole = mean(max_pole_magnitude);

%% ============================================================
% 12. Find worst-case trial
% =============================================================

[worst_pole, worst_trial] = ...
    max(max_pole_magnitude);

%% ============================================================
% 13. Display results
% =============================================================

fprintf('\n=============================================\n');
fprintf('REQ-007 RESULTS\n');
fprintf('=============================================\n');

fprintf('Total trials:             %d\n', N);
fprintf('Stable trials:            %d\n', stable_count);
fprintf('Unstable trials:          %d\n', unstable_count);

fprintf('\nMinimum maximum-pole magnitude: %.12f\n', ...
    best_pole);

fprintf('Mean maximum-pole magnitude:    %.12f\n', ...
    mean_max_pole);

fprintf('Worst observed pole magnitude:   %.12f\n', ...
    worst_pole);

fprintf('Worst-case trial number:         %d\n', ...
    worst_trial);

%% ============================================================
% 14. Requirement decision
% =============================================================

if unstable_count == 0

    REQ007_RESULT = "PASS";

else

    REQ007_RESULT = "FAIL";

end

fprintf('\n=============================================\n');
fprintf('REQ-007 DECISION: %s\n', REQ007_RESULT);
fprintf('=============================================\n');

%% ============================================================
% 15. Plot distribution of worst pole magnitude
% =============================================================

figure;

histogram(max_pole_magnitude,50);

hold on;

xline(1,'r--','LineWidth',2);

grid on;

xlabel('Maximum closed-loop pole magnitude');

ylabel('Number of trials');

title('REQ-007: Closed-Loop Pole Magnitude Under ±30% Model Uncertainty');

legend( ...
    'Monte Carlo trials', ...
    'Stability boundary |pole| = 1', ...
    'Location','best');

%% ============================================================
% 16. Plot worst pole magnitude over trial number
% =============================================================

figure;

plot(1:N,max_pole_magnitude,'LineWidth',1);

hold on;

yline(1,'r--','LineWidth',2);

grid on;

xlabel('Monte Carlo trial');

ylabel('Maximum closed-loop pole magnitude');

title('REQ-007: Pole Magnitude Across Uncertainty Trials');

legend( ...
    'Maximum pole magnitude', ...
    'Stability boundary |pole| = 1', ...
    'Location','best');

%% ============================================================
% 17. Save numerical results
% =============================================================

save( ...
    'REQ007_30pct_uncertainty_results.mat', ...
    'A','B','Ad','Bd','K','Q','R','Ts', ...
    'N','uncertainty','threshold', ...
    'max_pole_magnitude', ...
    'unstable_trials', ...
    'unstable_count', ...
    'stable_count', ...
    'worst_pole', ...
    'worst_trial', ...
    'mean_max_pole', ...
    'REQ007_RESULT');

fprintf('\nResults saved to:\n');
fprintf('REQ007_30pct_uncertainty_results.mat\n');