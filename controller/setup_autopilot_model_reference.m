%% setup_autopilot_model_reference.m
%
% One-time setup: converts the in-place 'Autopilot' subsystem into a
% separate Autopilot.slx model, automatically replacing it in the main
% model with a Model block wired to the same ports. This uses Simulink's
% built-in conversion function rather than manually rebuilding the
% extraction the way autopilot_subsystem_refactor.m did for the
% checkpoint version -- less error-prone, and it's exactly what
% Simulink.SubSystem.convertToModelReference exists for.
%
% Run this ONCE. After it succeeds, you have:
%   - Autopilot.slx (standalone, matches your generated code's interface)
%   - The main model with a Model block in place of the old subsystem
% Then use run_sil_mil_comparison.m (separate script) to actually test.

clear; clc; close all;
bdclose('all');

% ------------------------------------------------------------
% CONFIG -- edit if your names differ
% ------------------------------------------------------------
mainModelFile = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx';
subsystemPath = 'Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy/Autopilot';
newModelName  = 'Autopilot';   % -> creates Autopilot.slx

if ~isfile(mainModelFile)
    error('Model file not found: %s', mainModelFile);
end

load_system(mainModelFile);

% ------------------------------------------------------------
% Required before conversion: the subsystem must be "atomic," not
% "virtual" (a plain graphical grouping). Same setting your own
% autopilot_subsystem_refactor.m already used when building the
% checkpoint-era AUTOPILOT subsystem -- this one just never got it.
% ------------------------------------------------------------
set_param(subsystemPath, 'TreatAsAtomicUnit', 'on');
fprintf('Set %s to atomic (required for model-reference conversion).\n', subsystemPath);

% ------------------------------------------------------------
% Convert. This single call:
%   - creates Autopilot.slx from the subsystem's contents
%   - replaces the subsystem in the main model with a Model block
%   - wires the Model block's ports to match the original subsystem's
%     inputs/outputs automatically
% ------------------------------------------------------------
Simulink.SubSystem.convertToModelReference( ...
    subsystemPath, ...
    newModelName, ...
    'ReplaceSubsystem', true);

fprintf('Conversion complete.\n');
fprintf('Created: %s.slx\n', newModelName);
fprintf('The main model now has a Model block in place of the subsystem.\n');

save_system(bdroot(subsystemPath));
fprintf('Main model saved.\n');

fprintf('\nNext: find the Model block''s exact name for the comparison script --\n');
mdl = bdroot(subsystemPath);
modelBlocks = find_system(mdl, 'BlockType', 'ModelReference');
for k = 1:numel(modelBlocks)
    fprintf('  Model block found: %s (references %s)\n', ...
        modelBlocks{k}, get_param(modelBlocks{k}, 'ModelName'));
end
fprintf('Use that exact block path in run_sil_mil_comparison.m''s CONFIG.\n');
