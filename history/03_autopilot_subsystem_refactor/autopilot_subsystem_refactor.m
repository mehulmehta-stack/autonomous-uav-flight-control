%% DAY 8A — AUTOPILOT ARCHITECTURE CLEANUP
% Refactors the current validated Day-7 C172P SIL model into a single
% AUTOPILOT subsystem.
%
% IMPORTANT:
% 1) Controller gains and controller mathematics are NOT changed.
% 2) The existing Autotpilot(6).slx Model Reference is preserved.
% 3) Lateral L1 + DLQR is moved inside the same AUTOPILOT boundary.
% 4) Longitudinal inputs are changed to:
%       U       -> Synthetic_AirData output
%       q       -> Synthetic_IMU output
%       altitude-> EKF estimated D converted to ft
%       alpha   -> Synthetic_AirData output
%       phi     -> EKF estimate
%       theta   -> EKF estimate
% 5) The Synthetic_AirData block is an explicit sensor interface. Its
%    current internal implementation is pass-through from JSBSim state.
%    This is intentional: later HIL replaces this sensor block, NOT the
%    AUTOPILOT.
%
% OUTPUT:
%   <original_name>_CLEAN_AUTOPILOT.slx
%
% Run this script from the folder containing the two SLX files.

clear; clc;

%% ---------------- USER FILES ----------------
rootFile = 'Final_JSB_4wp.slx';
longFile = 'Autopilot1.slx';

assert(isfile(rootFile), 'Root model not found: %s', rootFile);
assert(isfile(longFile), 'Longitudinal model not found: %s', longFile);

[~, rootBase, ~] = fileparts(rootFile);
outFile = [rootBase '_CLEAN_AUTOPILOT.slx'];

%% ---------------- LOAD MODELS ----------------
load_system(rootFile);
root = bdroot(rootBase);

% The root model normally has this exact referenced model name.
longModel = 'Autotpilot';
load_system(longFile);

set_param(root, 'SimulationCommand', 'stop');

%% ---------------- SAVE WORKING COPY ----------------
save_system(root, outFile);
close_system(root, 0);
load_system(outFile);
root = bdroot(rootBase);

fprintf('\nWorking copy:\n%s\n', fullfile(pwd, outFile));

%% ---------------- SAFETY CHECKS ----------------
requiredRootBlocks = { ...
    'Model', ...
    'Navigation_EKF', ...
    'Synthetic_IMU', ...
    'Demux', ...
    'Demux3', ...
    'Vector Concatenate'};

for k = 1:numel(requiredRootBlocks)
    assert(has_block(root, requiredRootBlocks{k}), ...
        'Required root block missing: %s', requiredRootBlocks{k});
end

assert(strcmp(get_param([root '/Model'], 'ModelNameDialog'), 'Autotpilot.slx') || ...
       strcmp(get_param([root '/Model'], 'ModelNameDialog'), 'Autotpilot'), ...
       'The existing Model Reference does not point to Autotpilot.slx.');

%% ---------------- REMOVE OLD AUTOPILOT IF SCRIPT IS RERUN ----------------
if has_block(root, 'AUTOPILOT')
    delete_block([root '/AUTOPILOT']);
end

if has_block(root, 'Synthetic_AirData')
    delete_block([root '/Synthetic_AirData']);
end

%% ================================================================
% 1. CREATE EXPLICIT SYNTHETIC AIR-DATA SENSOR INTERFACE
% ================================================================
% Current SIL implementation:
%   JSBSim U      -> U_meas
%   JSBSim alpha  -> alpha_meas
%
% The controller will no longer connect directly to JSBSim.
% In a later HIL phase this whole block can be replaced by real
% airspeed / angle-of-attack sensing without changing AUTOPILOT.

add_block('simulink/Ports & Subsystems/Subsystem', ...
    [root '/Synthetic_AirData'], ...
    'Position', [1950 650 2100 730]);

% Remove default contents.
defaultBlocks = find_system([root '/Synthetic_AirData'], ...
    'SearchDepth', 1, 'Type', 'Block');
if ~isempty(defaultBlocks)
    delete_block(defaultBlocks);
end

add_block('simulink/Ports & Subsystems/In1', ...
    [root '/Synthetic_AirData/U_truth'], ...
    'Port', '1', 'Position', [30 35 60 55]);
add_block('simulink/Ports & Subsystems/In1', ...
    [root '/Synthetic_AirData/alpha_truth'], ...
    'Port', '2', 'Position', [30 95 60 115]);
add_block('simulink/Ports & Subsystems/Out1', ...
    [root '/Synthetic_AirData/U_meas'], ...
    'Port', '1', 'Position', [260 35 290 55]);
add_block('simulink/Ports & Subsystems/Out1', ...
    [root '/Synthetic_AirData/alpha_meas'], ...
    'Port', '2', 'Position', [260 95 290 115]);

add_line([root '/Synthetic_AirData'], 'U_truth/1', 'U_meas/1', 'autorouting', 'on');
add_line([root '/Synthetic_AirData'], 'alpha_truth/1', 'alpha_meas/1', 'autorouting', 'on');

set_param([root '/Synthetic_AirData'], ...
    'BackgroundColor', 'lightBlue', ...
    'TreatAsAtomicUnit', 'on');

%% ================================================================
% 2. CREATE SINGLE AUTOPILOT SUBSYSTEM
% ================================================================
add_block('simulink/Ports & Subsystems/Subsystem', ...
    [root '/AUTOPILOT'], ...
    'Position', [2380 850 2600 1080]);

% Remove default In1/Out1.
defaultBlocks = find_system([root '/AUTOPILOT'], ...
    'SearchDepth', 1, 'Type', 'Block');
if ~isempty(defaultBlocks)
    delete_block(defaultBlocks);
end

AP = [root '/AUTOPILOT'];

% External interface:
%   1 x_hat       [N E D VN VE VD phi theta psi]
%   2 imu         [ax ay az p q r phi theta psi]
%   3 U_meas      airspeed
%   4 alpha_meas  angle of attack
% Outputs:
%   1 throttle_cmd
%   2 aileron_cmd
%   3 elevator_cmd
add_block('simulink/Ports & Subsystems/In1', [AP '/x_hat'], ...
    'Port','1', 'Position',[30 55 60 75]);
add_block('simulink/Ports & Subsystems/In1', [AP '/imu'], ...
    'Port','2', 'Position',[30 110 60 130]);
add_block('simulink/Ports & Subsystems/In1', [AP '/U_meas'], ...
    'Port','3', 'Position',[30 165 60 185]);
add_block('simulink/Ports & Subsystems/In1', [AP '/alpha_meas'], ...
    'Port','4', 'Position',[30 220 60 240]);

add_block('simulink/Ports & Subsystems/Out1', [AP '/Throttle_cmd'], ...
    'Port','1', 'Position',[1220 120 1250 140]);
add_block('simulink/Ports & Subsystems/Out1', [AP '/Aileron_cmd'], ...
    'Port','2', 'Position',[1220 430 1250 450]);
add_block('simulink/Ports & Subsystems/Out1', [AP '/Elevator_cmd'], ...
    'Port','3', 'Position',[1220 200 1250 220]);

% State-vector demuxes.
add_block('simulink/Signal Routing/Demux', [AP '/xhat_Demux'], ...
    'Outputs','9', 'Position',[130 45 165 285]);
add_block('simulink/Signal Routing/Demux', [AP '/imu_Demux'], ...
    'Outputs','9', 'Position',[130 320 165 560]);

% EKF NED D is positive down. Existing longitudinal controller expects
% altitude in ft, therefore altitude_ft = -D_m * 3.28084.
add_block('simulink/Math Operations/Gain', [AP '/D_to_Altitude_ft'], ...
    'Gain','-3.28084', ...
    'Position',[300 95 380 125]);

% Copy the already-validated longitudinal Model Reference.
add_block([root '/Model'], [AP '/Longitudinal_Control'], ...
    'Position',[600 80 850 280]);

% Copy the already-validated lateral controller components.
add_block([root '/L1_Guidance'], [AP '/L1_Guidance'], ...
    'Position',[410 350 540 455]);
add_block([root '/Sum'], [AP '/Lateral_Phi_Error'], ...
    'Position',[590 350 620 410]);
add_block([root '/Mux'], [AP '/Lateral_State_Mux'], ...
    'Position',[660 350 695 410]);
add_block([root '/Gain'], [AP '/Lateral_DLQR_Gain'], ...
    'Position',[735 355 800 405]);
add_block([root '/Gain1'], [AP '/Lateral_Negate'], ...
    'Position',[835 355 885 405]);
add_block([root '/Aileron Saturation'], [AP '/Aileron_Saturation'], ...
    'Position',[920 350 1000 410]);

% Preserve the unused L1 diagnostic outputs without allowing dangling
% signals inside the controller.
add_block('simulink/Sinks/Terminator', [AP '/L1_a_cmd_Terminator'], ...
    'Position',[650 475 670 495]);
add_block('simulink/Sinks/Terminator', [AP '/L1_eta_Terminator'], ...
    'Position',[650 505 670 525]);

% Preserve the existing unused longitudinal aileron output.
add_block('simulink/Sinks/Terminator', [AP '/Longitudinal_Aileron_Terminator'], ...
    'Position',[900 265 920 285]);

%% ---------------- INTERNAL AUTOPILOT WIRING ----------------
% External input fan-out.
add_line(AP, 'x_hat/1', 'xhat_Demux/1', 'autorouting','on');
add_line(AP, 'imu/1', 'imu_Demux/1', 'autorouting','on');

% Longitudinal controller inputs.
add_line(AP, 'U_meas/1', 'Longitudinal_Control/1', 'autorouting','on');
add_line(AP, 'imu_Demux/5', 'Longitudinal_Control/2', 'autorouting','on'); % q
add_line(AP, 'xhat_Demux/3', 'D_to_Altitude_ft/1', 'autorouting','on');   % D
add_line(AP, 'D_to_Altitude_ft/1', 'Longitudinal_Control/3', 'autorouting','on');
add_line(AP, 'alpha_meas/1', 'Longitudinal_Control/4', 'autorouting','on');
add_line(AP, 'xhat_Demux/7', 'Longitudinal_Control/5', 'autorouting','on'); % phi
add_line(AP, 'xhat_Demux/8', 'Longitudinal_Control/6', 'autorouting','on'); % theta

% Longitudinal outputs.
add_line(AP, 'Longitudinal_Control/1', 'Throttle_cmd/1', 'autorouting','on');
add_line(AP, 'Longitudinal_Control/2', 'Longitudinal_Aileron_Terminator/1', 'autorouting','on');
add_line(AP, 'Longitudinal_Control/3', 'Elevator_cmd/1', 'autorouting','on');

% L1 guidance: N, E, VN, VE.
add_line(AP, 'xhat_Demux/1', 'L1_Guidance/1', 'autorouting','on');
add_line(AP, 'xhat_Demux/2', 'L1_Guidance/2', 'autorouting','on');
add_line(AP, 'xhat_Demux/4', 'L1_Guidance/3', 'autorouting','on');
add_line(AP, 'xhat_Demux/5', 'L1_Guidance/4', 'autorouting','on');

% L1 diagnostics.
add_line(AP, 'L1_Guidance/1', 'L1_a_cmd_Terminator/1', 'autorouting','on');
add_line(AP, 'L1_Guidance/2', 'L1_eta_Terminator/1', 'autorouting','on');

% Preserve original lateral controller structure exactly:
%   phi error -> [phi error, p] -> K_lat -> -1 -> saturation.
add_line(AP, 'xhat_Demux/7', 'Lateral_Phi_Error/1', 'autorouting','on');
add_line(AP, 'L1_Guidance/3', 'Lateral_Phi_Error/2', 'autorouting','on');
add_line(AP, 'Lateral_Phi_Error/1', 'Lateral_State_Mux/1', 'autorouting','on');
add_line(AP, 'imu_Demux/4', 'Lateral_State_Mux/2', 'autorouting','on'); % p
add_line(AP, 'Lateral_State_Mux/1', 'Lateral_DLQR_Gain/1', 'autorouting','on');
add_line(AP, 'Lateral_DLQR_Gain/1', 'Lateral_Negate/1', 'autorouting','on');
add_line(AP, 'Lateral_Negate/1', 'Aileron_Saturation/1', 'autorouting','on');
add_line(AP, 'Aileron_Saturation/1', 'Aileron_cmd/1', 'autorouting','on');

set_param(AP, ...
    'BackgroundColor','lightBlue', ...
    'TreatAsAtomicUnit','on');

%% ================================================================
% 3. ROOT SENSOR CONNECTIONS
% ================================================================
% U: Demux output 1 is the existing validated U signal.
% alpha: Demux3 output 2 is the existing validated alpha signal.
add_line(root, 'Demux/1', 'Synthetic_AirData/1', 'autorouting','on');
add_line(root, 'Demux3/2', 'Synthetic_AirData/2', 'autorouting','on');

% EKF and IMU are the controller-facing navigation/sensor interfaces.
add_line(root, 'Navigation_EKF/1', 'AUTOPILOT/1', 'autorouting','on');
add_line(root, 'Synthetic_IMU/1', 'AUTOPILOT/2', 'autorouting','on');
add_line(root, 'Synthetic_AirData/1', 'AUTOPILOT/3', 'autorouting','on');
add_line(root, 'Synthetic_AirData/2', 'AUTOPILOT/4', 'autorouting','on');

%% ================================================================
% 5. REMOVE OLD ROOT CONTROLLER BLOCKS
% ================================================================
% These are now duplicated inside AUTOPILOT. All of their old signal
% lines are removed automatically with the blocks.
oldControllerBlocks = { ...
    'Model', ...
    'L1_Guidance', ...
    'Sum', ...
    'Mux', ...
    'Gain', ...
    'Gain1', ...
    'Aileron Saturation'};

for k = 1:numel(oldControllerBlocks)
    blk = [root '/' oldControllerBlocks{k}];
    if has_block(root, oldControllerBlocks{k})
        delete_block(blk);
    end
end

%% ================================================================
% 6. RECONNECT OUTPUTS AFTER OLD BLOCK DELETION
% ================================================================
% Deleting Model removed its old connections to Vector Concatenate.
% Deleting lateral blocks removed their old connections to Vector
% Concatenate and scopes. Recreate only the required clean connections.
add_line(root, 'AUTOPILOT/1', 'Vector Concatenate/1', 'autorouting','on');
add_line(root, 'AUTOPILOT/2', 'Vector Concatenate/2', 'autorouting','on');
add_line(root, 'AUTOPILOT/3', 'Vector Concatenate/3', 'autorouting','on');

add_line(root, 'AUTOPILOT/1', 'Scope15/1', 'autorouting','on');
add_line(root, 'AUTOPILOT/2', 'Scope4/1', 'autorouting','on');
add_line(root, 'AUTOPILOT/2', 'Scope29/1', 'autorouting','on');

%% ---------------- MODEL CALLBACK / DIAGNOSTIC ----------------
% Add a descriptive annotation without modifying controller equations.
try
    ann = Simulink.Annotation(root, sprintf([ ...
        'DAY 8A — CLEAN AUTOPILOT ARCHITECTURE\n' ...
        'Sensors → EKF / Air Data → AUTOPILOT → Actuators\n' ...
        'Longitudinal: U/q/altitude/alpha/phi/theta via sensor-estimated interface\n' ...
        'Lateral: EKF → L1 → DLQR\n' ...
        'Controller gains unchanged.']));
    ann.Position = [1980 470 2200 560];
catch
    % Annotation support is cosmetic; continue if unavailable.
end

%% ---------------- SAVE ----------------
save_system(root, outFile);

fprintf('\n============================================================\n');
fprintf('DAY 8A CLEANUP COMPLETE\n');
fprintf('Output: %s\n', fullfile(pwd, outFile));
fprintf('Backup: %s\n', fullfile(pwd, backupFile));
fprintf('============================================================\n');

%% ---------------- STRUCTURAL CHECKS ----------------
assert(has_block(root,'AUTOPILOT'), 'AUTOPILOT subsystem was not created.');
assert(has_block(root,'Synthetic_AirData'), 'Synthetic_AirData was not created.');
assert(~has_block(root,'Model'), 'Old root Model Reference still exists.');
assert(~has_block(root,'L1_Guidance'), 'Old root L1_Guidance still exists.');
assert(~has_block(root,'Aileron Saturation'), 'Old root lateral controller still exists.');

fprintf('\nStructural checks passed.\n');
fprintf('Next step: open the CLEAN_AUTOPILOT model and run Update Diagram / Ctrl+D.\n');
fprintf('Do NOT run the 800 s test until Ctrl+D reports no errors.\n');

%% ---------------- HELPER ----------------
function tf = has_block(model, name)
    tf = ~isempty(find_system(model, ...
        'SearchDepth', 1, ...
        'Name', name));
end
