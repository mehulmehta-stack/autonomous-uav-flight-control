%% ============================================================
% DAY 9 — ADD IMU LOGGING
% =============================================================

%clear;
clc;

MODEL = 'Final_JSB_4wp_DAY8B_SENSOR_FIXED';

fprintf('\n============================================================\n');
fprintf('DAY 9 — ADD IMU LOGGING\n');
fprintf('============================================================\n\n');

load_system(MODEL);

imu_block = [MODEL '/Synthetic_IMU'];

fprintf('Synthetic_IMU found:\n');
fprintf('  %s\n\n', imu_block);


%% ------------------------------------------------------------
% Find existing To Workspace blocks
% ------------------------------------------------------------

existing = find_system(MODEL, ...
    'BlockType','ToWorkspace');

fprintf('Existing To Workspace blocks:\n');

for k = 1:numel(existing)
    fprintf('  %s\n', existing{k});
end

fprintf('\n');


%% ------------------------------------------------------------
% Create logging block
% ------------------------------------------------------------

log_name = 'DAY9_IMU_LOG';

if ~isempty(find_system(MODEL, ...
        'SearchDepth',1, ...
        'Name',log_name))

    delete_block([MODEL '/' log_name]);

end


add_block( ...
    'simulink/Sinks/To Workspace', ...
    [MODEL '/' log_name], ...
    'VariableName','imu_log', ...
    'SaveFormat','Timeseries', ...
    'Position',[850 150 980 190]);


%% ------------------------------------------------------------
% Determine Synthetic_IMU output port
% ------------------------------------------------------------

ph = get_param(imu_block,'PortHandles');

fprintf('Synthetic_IMU output ports: %d\n', ...
    numel(ph.Outport));


if isempty(ph.Outport)

    error('Synthetic_IMU has no output port.');

end


%% ------------------------------------------------------------
% Connect output to logging block
% ------------------------------------------------------------

log_ph = get_param( ...
    [MODEL '/' log_name], ...
    'PortHandles');


try

    add_line( ...
        MODEL, ...
        ph.Outport(1), ...
        log_ph.Inport(1), ...
        'autorouting','on');

catch ME

    fprintf('\nCould not connect logging block.\n');
    fprintf('%s\n',ME.message);

    delete_block([MODEL '/' log_name]);

    error( ...
        'Logging connection failed. No model changes were saved.');

end


%% ------------------------------------------------------------
% Save
% ------------------------------------------------------------

save_system(MODEL);

fprintf('\n============================================================\n');
fprintf('IMU LOGGING ADDED\n');
fprintf('============================================================\n');

fprintf('\nWorkspace variable:\n');
fprintf('  imu_log\n');

fprintf('\nFormat:\n');
fprintf('  MATLAB timeseries\n');

fprintf('\nModel saved:\n');
fprintf('  %s.slx\n',MODEL);

fprintf('\n============================================================\n');