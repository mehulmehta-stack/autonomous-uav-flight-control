%% CHARACTERIZE GPS-REACQUISITION TRANSIENT (GAIN-SCHEDULED CONTROLLER)
% ------------------------------------------------------------
% Single run, cruise IC, GPS_Validity left completely untouched (its
% default t=25-35s outage is exactly what we want active for this test).
% No Stateflow overrides, no S-Function parameter swapping -- the model's
% saved defaults already point at the cruise script/IC.
%
% Purpose: characterize what the elevator/pitch-rate actually do when
% GPS reacquires at t=35s, in the FULL gain-scheduled LQG stack -- this
% is new information, not something the earlier EKF-only GPS-outage
% validation covered.
% ------------------------------------------------------------

clear; clc; close all;
bdclose('all');

mfile = 'Final_JSB_4wp_LQG_withoutNoise_GnSch.slx';
Tsim = 40;

if ~isfile(mfile)
    error('Model file not found: %s', mfile);
end

load_system(mfile);
[~, mdl, ~] = fileparts(mfile);

fprintf('Running with GPS_Validity UNMODIFIED (default t=25-35s outage active)...\n');
simOut = sim(mdl, 'StopTime', num2str(Tsim));

stateData = get_logged_var(simOut, 'val_state');
controlData = get_logged_var(simOut, 'val_control');

t = stateData.time;
u = stateData.signals.values(:,1);
theta = stateData.signals.values(:,11);
elevator = controlData.signals.values(:,4);

close_system(mdl, 0);

%% Plot with the outage window explicitly marked
figure('Position', [100 100 900 750]);

subplot(3,1,1);
plot(t, u, 'b', 'LineWidth', 1.5); hold on;
xline(25, 'r--'); xline(35, 'r--');
yline(184.999, 'k:');
grid on; xlabel('Time (s)'); ylabel('True airspeed u (ft/s)');
title('GPS outage window (t=25-35s, dashed red) -- airspeed');

subplot(3,1,2);
plot(t, rad2deg(theta), 'b', 'LineWidth', 1.5); hold on;
xline(25, 'r--'); xline(35, 'r--');
grid on; xlabel('Time (s)'); ylabel('Pitch \theta (deg)');
title('Pitch attitude through GPS outage/reacquisition');

subplot(3,1,3);
plot(t, elevator, 'b', 'LineWidth', 1.5); hold on;
xline(25, 'r--'); xline(35, 'r--');
ylim([-1.1 1.1]);
grid on; xlabel('Time (s)'); ylabel('Elevator position (norm)');
title('Elevator command -- check for saturation and how long it persists');

saveas(gcf, 'GPS_Reacquisition_Characterization.png');

%% Numeric characterization
tail_before = t >= 20 & t < 25;
tail_outage = t >= 25 & t < 35;
tail_after  = t >= 35 & t < 40;

fid = fopen('GPS_Reacquisition_Characterization_Summary.txt', 'w');
fprintf(fid, 'GPS REACQUISITION CHARACTERIZATION (gain-scheduled LQG stack)\n');
fprintf(fid, '===============================================================\n');
fprintf(fid, 'GPS_Validity outage active (t=25-35s), unmodified from saved model.\n\n');

fprintf(fid, 'Elevator, before outage  (t=20-25s): max|e|=%.4f, mean=%.4f\n', ...
    max(abs(elevator(tail_before))), mean(elevator(tail_before)));
fprintf(fid, 'Elevator, during outage  (t=25-35s): max|e|=%.4f, mean=%.4f\n', ...
    max(abs(elevator(tail_outage))), mean(elevator(tail_outage)));
fprintf(fid, 'Elevator, after outage   (t=35-40s): max|e|=%.4f, mean=%.4f\n', ...
    max(abs(elevator(tail_after))), mean(elevator(tail_after)));

% Time to return to a small band around the pre-outage elevator level
pre_level = mean(elevator(tail_before));
post_idx = find(t >= 35);
recovered_idx = find(abs(elevator(post_idx) - pre_level) < 0.02, 1, 'first');
if ~isempty(recovered_idx)
    fprintf(fid, '\nElevator returns to within 0.02 of pre-outage level %.3f seconds after reacquisition.\n', ...
        t(post_idx(recovered_idx)) - 35);
else
    fprintf(fid, '\nElevator did NOT return within 0.02 of pre-outage level within the 5s window after reacquisition.\n');
end

fclose(fid);

fprintf('\nSaved: GPS_Reacquisition_Characterization.png, GPS_Reacquisition_Characterization_Summary.txt\n');

%% Helper (same as validate_gain_schedule_v3.m)
function data = get_logged_var(simOut, varname)
    try
        data = evalin('base', varname);
        return;
    catch
    end
    try
        data = simOut.get(varname);
        return;
    catch
    end
    try
        data = simOut.(varname);
        return;
    catch
    end
    error('Could not find logged variable "%s".', varname);
end
