%% REQ-001 / REQ-002 — FINAL MODEL RE-VERIFICATION (MIL)
%
% Same measurement convention as the original checkpoint test in
% C172_DLQR_4state_FINAL_VERIFIED.m: settle window t>50s, max deviation
% from trim. Difference here: this runs the actual final gain-scheduled
% Simulink model directly (MIL), not generated code (SIL) — because no
% code has been generated for the final controller yet. That's a real,
% stated difference in verification method, not something to gloss over
% in the writeup: this closes REQ-001/002 via Simulation (MIL), not the
% SIL method the checkpoint version used.
%
% Uses the model's SAVED/DEFAULT initial condition — this script does not
% swap IC files, so whatever cruise IC is currently configured in the
% .slx is what runs. If that's not the cruise case, swap it first the
% same way validate_gain_schedule_v3.m does.
%
% GPS_Validity is left at its default (outage active t=25-35s) —
% deliberately not isolated, matching every other validation script's
% convention in this project. characterize_gps_reacquisition.m already
% showed pitch stays well inside +/-2 deg through that window, so this
% is expected to still pass with it active, not a reason to disable it.

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx';
Tsim = 80;   % long enough for a clean 30s settled window after t=50s

if ~isfile(mfile)
    error('Model file not found: %s', mfile);
end

load_system(mfile);
[~, mdl, ~] = fileparts(mfile);

fprintf('Running at the model''s current saved IC (should be cruise trim)...\n');
simOut = sim(mdl, 'StopTime', num2str(Tsim));

stateData = get_logged_var(simOut, 'val_state');

t = stateData.time;
theta = stateData.signals.values(:,11);   % rad, per c172_simulink.xml state order
h_ft  = stateData.signals.values(:,7);    % ft, per c172_simulink.xml state order

close_system(mdl, 0);

%% Trim reference
% [Guessing] using the same theta_trim as the checkpoint (0.0026556289 rad)
% and the initial logged altitude as the altitude reference, since the
% final model's commanded altitude constant was not independently
% confirmed in this pass. Correct these if the final model uses different
% trim values.
theta_trim = 0.0026556289;
h_ref_ft   = h_ft(1);

settled = t > 50;

theta_settled = theta(settled);
h_settled = h_ft(settled);

max_theta_dev_deg = max(abs(theta_settled - theta_trim)) * (180/pi);
max_h_dev_ft = max(abs(h_settled - h_ref_ft));
max_h_dev_m = max_h_dev_ft * 0.3048;

fprintf('\n============================================================\n');
fprintf('REQ-001 / REQ-002 — FINAL MODEL (MIL), settled window t>50s\n');
fprintf('============================================================\n');
fprintf('REQ-001: max settled pitch deviation from trim: %.4f deg (limit +/-2 deg) -> %s\n', ...
    max_theta_dev_deg, tern(max_theta_dev_deg<=2,'PASS','FAIL'));
fprintf('REQ-002: max settled altitude deviation: %.4f ft (%.4f m) (limit +/-5 m) -> %s\n', ...
    max_h_dev_ft, max_h_dev_m, tern(max_h_dev_m<=5,'PASS','FAIL'));

figure('Position',[100 100 900 600]);
subplot(2,1,1);
plot(t, rad2deg(theta), 'LineWidth', 1.2); hold on;
yline(rad2deg(theta_trim)+2,'r--'); yline(rad2deg(theta_trim)-2,'r--');
xline(50,'k:');
grid on; ylabel('Pitch \theta (deg)'); title('REQ-001: Pitch hold, final gain-scheduled model (MIL)');

subplot(2,1,2);
plot(t, h_ft - h_ref_ft, 'LineWidth', 1.2); hold on;
yline(5/0.3048,'r--'); yline(-5/0.3048,'r--');
xline(50,'k:');
grid on; xlabel('Time (s)'); ylabel('Altitude deviation (ft)'); title('REQ-002: Altitude hold, final gain-scheduled model (MIL)');

saveas(gcf, 'REQ001_REQ002_final_model_verification.png');

save('REQ001_REQ002_final_model_results.mat', 't','theta','h_ft', ...
    'theta_trim','h_ref_ft','max_theta_dev_deg','max_h_dev_ft','max_h_dev_m');

fprintf('\nSaved: REQ001_REQ002_final_model_verification.png, REQ001_REQ002_final_model_results.mat\n');
fprintf('Paste the REQ-001/REQ-002 block above back and I''ll fold it into requirements.md.\n');

function s = tern(cond,a,b)
    if cond, s = a; else, s = b; end
end

function data = get_logged_var(simOut, varname)
    try, data = evalin('base', varname); return; catch, end
    try, data = simOut.get(varname); return; catch, end
    try, data = simOut.(varname); return; catch, end
    error('Could not find logged variable "%s".', varname);
end
