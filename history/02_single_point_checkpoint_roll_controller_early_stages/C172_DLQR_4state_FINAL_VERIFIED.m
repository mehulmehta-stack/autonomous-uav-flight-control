%clear; clc;

% C172 FINAL 4-STATE DLQR
% State order: x = [delta_u; delta_alpha; delta_theta; q]
% Do not permute states 3 and 4.

A = [ ...
 -0.060668363940, 3.2740147717, -32.166876080, -0.016844605405;
 -0.0027464567573, -3.0828659556, 3.7794562027e-07, 0.95743620415;
 0, -2.0384228900e-18, 5.9219042629e-18, 1.0000000000;
 0.0099808985828, -35.9682352700, -1.1114723170e-05, -5.7322819189];

B = [-2.5904758571; -0.099055249294; 0; -11.8301405300];
Ts = 0.008333;

u_trim = 185;
alpha_trim = 0.0026556289;
theta_trim = 0.0026556289;
q_trim = 0.0;

max_du = 10.0;
max_alpha = 0.12;
max_theta = 0.05;
max_q = 0.15;
max_elevator = 0.40;

Q = diag([1/max_du^2, 1/max_alpha^2, 1/max_theta^2, 1/max_q^2]);
R = 1/max_elevator^2;

sys_c = ss(A,B,eye(4),zeros(4,1));
sys_d = c2d(sys_c,Ts,'zoh');
[Ad,Bd,~,~] = ssdata(sys_d);

K = dlqr(Ad,Bd,Q,R);
Acl = Ad - Bd*K;
poles = eig(Acl);

disp('Q ='); disp(Q);
disp('R ='); disp(R);
disp('Ad ='); disp(Ad);
disp('Bd ='); disp(Bd);
disp('K ='); disp(K);
disp('Closed-loop poles ='); disp(poles);
disp('Closed-loop pole magnitudes ='); disp(abs(poles));
fprintf('Controllability rank = %d / 4\n',rank(ctrb(Ad,Bd)));

save('C172_DLQR_4state_FINAL.mat','A','B','Ad','Bd','K','Q','R','Ts', ...
    'u_trim','alpha_trim','theta_trim','q_trim');

disp('Saved: C172_DLQR_4state_FINAL.mat');

%% Task 2 — Compare MIL vs SIL, check the 5%, (REQ-006)
% Step 1 — plot both on one graph:
figure;
plot(alt_MIL.Time, alt_MIL.Data, 'b', 'LineWidth', 1.5); hold on;
plot(alt_SIL.Time, alt_SIL.Data, 'r--', 'LineWidth', 1.5);
legend('MIL (native blocks)', 'SIL (generated C code)');
xlabel('Time (s)'); ylabel('Altitude (ft)'); title('MIL vs SIL Altitude Tracking');
grid on;

% Step 2 — compute the actual percentage difference, don't eyeball it:
if length(alt_MIL.Data) ~= length(alt_SIL.Data)
    warning('Sample counts differ (%d vs %d) — resampling SIL onto MIL''s time points', ...
        length(alt_MIL.Data), length(alt_SIL.Data));
    alt_SIL_aligned = resample(alt_SIL, alt_MIL.Time);
    sil_data = alt_SIL_aligned.Data;
else
    sil_data = alt_SIL.Data;
end

error_pct = abs(alt_MIL.Data - sil_data) ./ alt_MIL.Data * 100;
diff_raw = alt_MIL.Data - alt_SIL.Data;
fprintf('Max absolute difference: %.15e ft\n', max(abs(diff_raw)));
fprintf('Max error: %.4f%%\n', max(error_pct));
fprintf('Mean error: %.4f%%\n', mean(error_pct));

%% REQ-002 (altitude ±5m)
settled_idx = find(alt_SIL.Time > 50);   % ignore the initial transient, only look after it's settled
settled_alt = alt_SIL.Data(settled_idx);
target_alt = 4000;  % ft, matches your Constant block
max_dev_ft = max(abs(settled_alt - target_alt));
max_dev_m = max_dev_ft * 0.3048;
fprintf('Max settled deviation: %.4f ft (%.4f m) — requirement is ±5 m\n', max_dev_ft, max_dev_m);

%% run the pitch check (REQ-001)
settled_theta = theta_SIL.Data(theta_SIL.Time > 50);
theta_trim = 0.0026556289;  % rad, from Autopilot_data.c
max_dev_deg = max(abs(settled_theta - theta_trim)) * (180/pi);
fprintf('REQ-001: Max settled pitch deviation from trim: %.4f deg — requirement is +/-2 deg\n', max_dev_deg);