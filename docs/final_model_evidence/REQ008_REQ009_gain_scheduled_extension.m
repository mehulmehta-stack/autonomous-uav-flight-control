%% REQ-008 / REQ-009 — EXTENDED TO ALL THREE GAIN-SCHEDULE POINTS
%
% Same exact methodology as REQ008_REQ009_stability_margin_analysis.m,
% looped across the three trim points already designed and saved in
% C172_GainSchedule_3pt.mat. No new design work — this only evaluates
% margins for the K you already have at each point.
%
% Phase margins at all three points were independently cross-checked in
% Python (100/185/220 ft/s: 89.48 / 92.64 / 90.51 deg) and matched the
% known 185 ft/s value to within 0.01 deg. Gain margin was NOT reliably
% reproducible outside MATLAB — this script is the real source for
% REQ-009's final numbers, run it and use its output, not a guess.

clear; clc;

load('C172_GainSchedule_3pt.mat');  % V_trim_pts, K_lon_pts, A_lon, B_lon, Ts

REQ008_limit = 30;   % degrees
REQ009_limit = 6;    % dB

results = struct();

for i = 1:3

    V = V_trim_pts(i);
    A = A_lon{i};
    B = B_lon{i};
    K = K_lon_pts(i,:);

    fprintf('\n============================================================\n');
    fprintf('TRIM POINT: %d ft/s\n', V);
    fprintf('============================================================\n');

    sys_c = ss(A,B,eye(4),zeros(4,1));
    sys_d = c2d(sys_c,Ts,'zoh');
    [Ad,Bd,~,~] = ssdata(sys_d);

    Acl = Ad - Bd*K;
    poles = eig(Acl);
    fprintf('Closed-loop pole magnitudes: '); fprintf('%.4f ', abs(poles)); fprintf('\n');
    if any(abs(poles) >= 1)
        error('Point %d ft/s is NOT discrete-time stable -- stop and investigate before trusting margins.', V);
    end

    L = ss(Ad,Bd,K,0,Ts);
    [Gm,Pm,Wcg,Wcp] = margin(L);

    if isinf(Gm)
        Gm_dB = Inf;
    else
        Gm_dB = 20*log10(Gm);
    end

    REQ008_PASS = Pm > REQ008_limit;
    REQ009_PASS = Gm_dB > REQ009_limit;

    fprintf('Phase Margin: %.6f deg  -> REQ-008 %s\n', Pm, tern(REQ008_PASS,'PASS','FAIL'));
    fprintf('Gain Margin:  %.6f dB   -> REQ-009 %s\n', Gm_dB, tern(REQ009_PASS,'PASS','FAIL'));
    fprintf('Gain crossover: %.6f rad/s   Phase crossover: %.6f rad/s\n', Wcp, Wcg);

    results.(sprintf('V%d',V)) = struct('Pm',Pm,'Gm_dB',Gm_dB,'Wcg',Wcg,'Wcp',Wcp,'poles',poles, ...
        'REQ008_PASS',REQ008_PASS,'REQ009_PASS',REQ009_PASS);
end

fprintf('\n============================================================\n');
fprintf('SUMMARY — all 3 gain-schedule points\n');
fprintf('============================================================\n');
for i = 1:3
    V = V_trim_pts(i);
    r = results.(sprintf('V%d',V));
    fprintf('%4d ft/s | Pm = %7.4f deg | Gm = %8.4f dB | REQ-008 %s | REQ-009 %s\n', ...
        V, r.Pm, r.Gm_dB, tern(r.REQ008_PASS,'PASS','FAIL'), tern(r.REQ009_PASS,'PASS','FAIL'));
end

save('REQ008_REQ009_gain_scheduled_results.mat','results','V_trim_pts');
fprintf('\nSaved: REQ008_REQ009_gain_scheduled_results.mat\n');
fprintf('Paste the SUMMARY block back and I''ll fold the real REQ-009 numbers into requirements.md.\n');

function s = tern(cond,a,b)
    if cond, s = a; else, s = b; end
end
