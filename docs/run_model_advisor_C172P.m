%% C172P Model Advisor execution helper
%
% Current model:
%   JSBSim_DLQR_PID_OuterLoop_AltitudeHold_c172p_Subsystem_OnlyCode.slx
%
% Purpose:
%   Run Model Advisor using the configuration/checks available in the local
%   MATLAB installation and create an HTML report.
%
% IMPORTANT:
%   Model Advisor only exposes checks provided by installed products/licenses
%   and the active Model Advisor configuration. This script does not claim
%   that every possible DO-178C check is available.
%
% Steps:
%   1. Put this script in the same folder as the final SLX.
%   2. Make that folder MATLAB's Current Folder.
%   3. Run this script.
%   4. Read the Command Window summary.
%   5. Open the generated HTML report.
%   6. Record the actual PASS/WARNING/FAIL/INCOMPLETE counts.
%
% MathWorks reference:
%   ModelAdvisor.run(Systems) runs the checks in the active configuration.
%   It can also generate a report.

clear;
clc;

model = 'JSBSim_DLQR_PID_OuterLoop_AltitudeHold_c172p_Subsystem_OnlyCode';
slx_file = [model '.slx'];

if ~isfile(slx_file)
    error(['Cannot find ' slx_file '. Set MATLAB Current Folder to the ', ...
           'folder containing the final C172P SLX.']);
end

load_system(model);

fprintf('\n=============================================\n');
fprintf('C172P MODEL ADVISOR RUN\n');
fprintf('=============================================\n');
fprintf('Model: %s\n\n', model);

% Run the checks in the model's active/default Model Advisor configuration.
results = ModelAdvisor.run( ...
    model, ...
    'DisplayResults','Details', ...
    'ReportFormat','html', ...
    'ReportPath',pwd, ...
    'ReportName','ModelAdvisor_C172P_report');

% Save the returned MATLAB result objects as evidence.
save('ModelAdvisor_C172P_results.mat','results');

fprintf('\n=============================================\n');
fprintf('MODEL ADVISOR RUN COMPLETE\n');
fprintf('=============================================\n');
fprintf('HTML report: %s\n', ...
    fullfile(pwd,'ModelAdvisor_C172P_report.html'));
fprintf('MAT-file:    %s\n', ...
    fullfile(pwd,'ModelAdvisor_C172P_results.mat'));

fprintf('\nIMPORTANT: open the HTML report and record the actual counts.\n');
fprintf('Do not label the model PASS merely because the run completed.\n');

close_system(model,0);
