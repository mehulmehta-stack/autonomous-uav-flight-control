clc;

fprintf('\n============================================================\n');
fprintf('DAY 9 — CONVERTING RAW p/q FEEDBACK TO EKF ESTIMATES\n');
fprintf('============================================================\n\n');

src = 'Final_JSB_4wp_IMU_withNoise';
out = 'Final_JSB_4wp_LQG';

load_system(src);

fprintf('Loaded: %s.slx\n',src);


%% =========================================================
% 1. UPDATE NAVIGATION_EKF
% ==========================================================

fprintf('\nUpdating Navigation_EKF to 12 states...\n');

rt = sfroot;

chart = rt.find( ...
    '-isa','Stateflow.EMChart', ...
    'Path',[src '/Navigation_EKF']);

if isempty(chart)
    error('Navigation_EKF MATLAB Function block was not found.');
end

chart.Script = fileread('Navigation_EKF_LQG.m');

fprintf('Navigation_EKF updated.\n');


%% =========================================================
% 2. CHANGE EKF DEMUX FROM 9 -> 12 OUTPUTS
% ==========================================================

fprintf('\nExpanding Demux5: 9 -> 12 outputs...\n');

set_param([src '/Demux5'],'Outputs','12');

fprintf('Demux5 expanded.\n');


%% =========================================================
% 3. REMOVE RAW p FROM LATERAL LQR
% ==========================================================

fprintf('\nReplacing lateral raw p feedback...\n');

delete_line(src,'Demux1/4','Mux/2');

add_line(src, ...
    'Demux5/10', ...
    'Mux/2', ...
    'autorouting','on');

fprintf('Lateral: p_raw -> p_hat.\n');


%% =========================================================
% 4. REMOVE RAW q FROM LONGITUDINAL LQR
% ==========================================================

fprintf('\nReplacing longitudinal raw q feedback...\n');

delete_line(src,'Demux1/5','Mux1/4');

add_line(src, ...
    'Demux5/11', ...
    'Mux1/4', ...
    'autorouting','on');

fprintf('Longitudinal: q_raw -> q_hat.\n');


%% =========================================================
% 5. SAVE AS NEW MODEL
% ==========================================================

fprintf('\nSaving new model...\n');

save_system(src,[out '.slx']);

fprintf('\n============================================================\n');
fprintf('DAY 9 LQG ARCHITECTURE UPDATE COMPLETE\n');
fprintf('============================================================\n\n');

fprintf('Original model preserved:\n');
fprintf('  %s.slx\n\n',src);

fprintf('New model:\n');
fprintf('  %s.slx\n\n',out);

fprintf('Final feedback architecture:\n');
fprintf('  Lateral      : phi_hat + p_hat\n');
fprintf('  Longitudinal : existing states + theta_hat + q_hat\n');
fprintf('  Raw p/q      : REMOVED from LQR feedback\n');
fprintf('============================================================\n');