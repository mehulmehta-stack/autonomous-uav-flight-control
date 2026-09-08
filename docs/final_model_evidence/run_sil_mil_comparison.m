%% run_sil_mil_comparison.m — REQ-006 / REQ-010, final model
%
% Toggles the Autopilot Model block between Normal (MIL) and
% Software-in-the-Loop (SIL), runs both, compares all THREE outputs
% (throttle, elevator, aileron) -- not just altitude the way the
% checkpoint version did.
%
% REQUIRES manually added To Workspace blocks, same pattern as
% validate_gain_schedule_v3.m and others already use:
%   - on the Throttle_cmd wire: variable 'throttle_log', Structure With Time
%   - on the Elevator wire:     variable 'elevator_log', Structure With Time
%   - on the Aileron wire:      variable 'aileron_log',  Structure With Time
% Save the model with those in place before running this script.
%
% [Likely, not Certain]: the exact SimulationMode string values below
% ('Normal', 'Software-in-the-loop (SIL)') match documented Simulink
% Coder behavior, but haven't been verified against a live session --
% if set_param errors, check the Model block's own Simulation Mode
% dropdown for the exact wording and use that string instead.

clear; clc; close all;
bdclose('all');

% ------------------------------------------------------------
% CONFIG -- fill in from setup_autopilot_model_reference.m's printed
% "Model block found: ..." line
% ------------------------------------------------------------
mainModelFile = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx';
modelBlockPath = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilot/Autopilot';
Tsim = 80;

if ~isfile(mainModelFile)
    error('Model file not found: %s', mainModelFile);
end

load_system(mainModelFile);
mdl = bdroot(modelBlockPath);

sigNames = {'throttle_log','elevator_log','aileron_log'};

%% ---- Run 1: MIL (Normal) ----
set_param(modelBlockPath, 'SimulationMode', 'Normal');
fprintf('Running MIL (Normal mode)...\n');
simOut_MIL = sim(mdl, 'StopTime', num2str(Tsim));

logMIL = struct();
for k = 1:3
    logMIL.(sigNames{k}) = get_logged_var(simOut_MIL, sigNames{k});
end

%% ---- Run 2: SIL ----
set_param(modelBlockPath, 'SimulationMode', 'Software-in-the-loop (SIL)');
fprintf('Running SIL...\n');
simOut_SIL = sim(mdl, 'StopTime', num2str(Tsim));

logSIL = struct();
for k = 1:3
    logSIL.(sigNames{k}) = get_logged_var(simOut_SIL, sigNames{k});
end

set_param(modelBlockPath, 'SimulationMode', 'Normal');  % restore
close_system(mdl, 0);

%% ---- Compare all three channels ----
fprintf('\n============================================================\n');
fprintf('REQ-006 / REQ-010 -- FINAL MODEL, MIL vs SIL, all 3 outputs\n');
fprintf('============================================================\n');

results = struct();
for k = 1:3
    name = sigNames{k};
    milSig = logMIL.(name);
    silSig = logSIL.(name);

    t_mil = milSig.time;
    mil_data = milSig.signals.values;
    sil_data = silSig.signals.values;

    if numel(mil_data) ~= numel(sil_data)
        sil_aligned = interp1(silSig.time, sil_data, t_mil);
    else
        sil_aligned = sil_data;
    end

    abs_err = abs(mil_data - sil_aligned);
    % Relative error, same convention as the checkpoint's REQ-006 test --
    % guarded against division by near-zero (throttle/elevator/aileron
    % can legitimately pass through zero).
    denom = max(abs(mil_data), 1e-6);
    pct_err = abs_err ./ denom * 100;

    fprintf('\n%s:\n', name);
    fprintf('  Max absolute error: %.6e\n', max(abs_err));
    fprintf('  Max %% error (guarded denom): %.4f%%\n', max(pct_err));
    fprintf('  Mean %% error (guarded denom): %.4f%%\n', mean(pct_err));

    results.(name).max_abs_err = max(abs_err);
    results.(name).max_pct_err = max(pct_err);
    results.(name).mean_pct_err = mean(pct_err);

    figure('Name', name);
    plot(t_mil, mil_data, 'b', 'LineWidth', 1.5); hold on;
    plot(t_mil, sil_aligned, 'r--', 'LineWidth', 1.5);
    legend('MIL', 'SIL'); grid on;
    xlabel('Time (s)'); ylabel(name, 'Interpreter', 'none');
    title(['MIL vs SIL: ' name], 'Interpreter', 'none');
    saveas(gcf, ['REQ006_REQ010_' name '_comparison.png']);
end

save('REQ006_REQ010_final_model_results.mat', 'results');
fprintf('\nSaved: REQ006_REQ010_final_model_results.mat and 3 comparison PNGs.\n');
fprintf('Paste the block above back and I''ll fold real REQ-006/REQ-010 numbers into requirements.md.\n');

function data = get_logged_var(simOut, varname)
    try, data = evalin('base', varname); return; catch, end
    try, data = simOut.get(varname); return; catch, end
    try, data = simOut.(varname); return; catch, end
    error('Could not find logged variable "%s". Confirm the To Workspace block''s Variable Name is exactly "%s" and Save Format is "Structure With Time".', varname, varname);
end
