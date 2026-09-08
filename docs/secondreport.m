%% ============================================================
% DAY 9 — RESOLVE ACTUAL SENSOR / EKF SIGNAL NAMES
% READ-ONLY — NO MODEL MODIFICATION
% ============================================================

model = 'Final_JSB_4wp_DAY8B_SENSOR_FIXED';

load_system(model);

fprintf('\n');
fprintf('============================================================\n');
fprintf('DAY 9 — RESOLVED SENSOR / EKF MAPPING\n');
fprintf('============================================================\n');

%% ------------------------------------------------------------
% Helper function
% ------------------------------------------------------------

function print_block_info(model, block)

    fprintf('\n------------------------------------------------------------\n');
    fprintf('BLOCK: %s\n', block);
    fprintf('------------------------------------------------------------\n');

    try
        fprintf('BlockType : %s\n', get_param(block,'BlockType'));
    catch
    end

    try
        fprintf('Name      : %s\n', get_param(block,'Name'));
    catch
    end

    try
        fprintf('Path      : %s\n', getfullname(block));
    catch
    end

    try
        fprintf('Ports     : %s\n', get_param(block,'Ports'));
    catch
    end
end

%% ------------------------------------------------------------
% 1. Resolve Navigation_EKF output
% ------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('NAVIGATION_EKF OUTPUT\n');
fprintf('============================================================\n');

ekf = [model '/Navigation_EKF'];

ph = get_param(ekf,'PortHandles');

if isfield(ph,'Outport')

    for i = 1:numel(ph.Outport)

        port = ph.Outport(i);

        fprintf('\nEKF OUTPORT %d\n',i);

        line = get_param(port,'Line');

        if line == -1

            fprintf('  No connected line.\n');
            continue;

        end

        dst = get_param(line,'DstPortHandle');

        if isempty(dst)

            fprintf('  No destination.\n');
            continue;

        end

        for j = 1:numel(dst)

            dstPort = dst(j);

            if dstPort == -1
                continue;
            end

            parent = get_param(dstPort,'Parent');
            portNumber = get_param(dstPort,'PortNumber');

            fprintf('  --> %s / Port %d\n', ...
                parent, portNumber);

        end
    end
end

%% ------------------------------------------------------------
% 2. Resolve Synthetic_IMU outputs
% ------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('SYNTHETIC_IMU OUTPUTS\n');
fprintf('============================================================\n');

imu = [model '/Synthetic_IMU'];

ph = get_param(imu,'PortHandles');

if isfield(ph,'Outport')

    for i = 1:numel(ph.Outport)

        port = ph.Outport(i);

        fprintf('\nIMU OUTPORT %d\n',i);

        line = get_param(port,'Line');

        if line == -1

            fprintf('  No connected line.\n');
            continue;

        end

        dst = get_param(line,'DstPortHandle');

        if isempty(dst)

            fprintf('  No destination.\n');
            continue;

        end

        for j = 1:numel(dst)

            dstPort = dst(j);

            if dstPort == -1
                continue;
            end

            parent = get_param(dstPort,'Parent');
            portNumber = get_param(dstPort,'PortNumber');

            fprintf('  --> %s / Port %d\n', ...
                parent, portNumber);

        end
    end
end

%% ------------------------------------------------------------
% 3. Resolve Synthetic_AirData outputs
% ------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('SYNTHETIC_AIRDATA OUTPUTS\n');
fprintf('============================================================\n');

air = [model '/Synthetic_AirData'];

ph = get_param(air,'PortHandles');

if isfield(ph,'Outport')

    for i = 1:numel(ph.Outport)

        port = ph.Outport(i);

        fprintf('\nAIRDATA OUTPORT %d\n',i);

        line = get_param(port,'Line');

        if line == -1

            fprintf('  No connected line.\n');
            continue;

        end

        dst = get_param(line,'DstPortHandle');

        if isempty(dst)

            fprintf('  No destination.\n');
            continue;

        end

        for j = 1:numel(dst)

            dstPort = dst(j);

            if dstPort == -1
                continue;
            end

            parent = get_param(dstPort,'Parent');
            portNumber = get_param(dstPort,'PortNumber');

            fprintf('  --> %s / Port %d\n', ...
                parent, portNumber);

        end
    end
end

%% ------------------------------------------------------------
% 4. Explicitly inspect the critical Demux blocks
% ------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('CRITICAL DEMUX MAPPING\n');
fprintf('============================================================\n');

demuxes = {'Demux1','Demux5'};

for d = 1:numel(demuxes)

    block = [model '/' demuxes{d}];

    fprintf('\n============================================================\n');
    fprintf('%s\n',demuxes{d});
    fprintf('============================================================\n');

    ph = get_param(block,'PortHandles');

    if ~isfield(ph,'Inport')
        continue;
    end

    for i = 1:numel(ph.Inport)

        port = ph.Inport(i);

        line = get_param(port,'Line');

        if line == -1
            fprintf('INPUT %d: unconnected\n',i);
            continue;
        end

        src = get_param(line,'SrcPortHandle');

        if src == -1
            fprintf('INPUT %d: no source\n',i);
            continue;
        end

        srcParent = get_param(src,'Parent');
        srcPortNumber = get_param(src,'PortNumber');

        fprintf('\nINPUT %d\n',i);
        fprintf('  Source: %s / Port %d\n', ...
            srcParent, srcPortNumber);
    end

    if isfield(ph,'Outport')

        for i = 1:numel(ph.Outport)

            port = ph.Outport(i);

            line = get_param(port,'Line');

            if line == -1
                continue;
            end

            dst = get_param(line,'DstPortHandle');

            fprintf('\nOUTPUT %d\n',i);

            for j = 1:numel(dst)

                if dst(j) == -1
                    continue;
                end

                dstParent = get_param(dst(j),'Parent');
                dstPortNumber = get_param(dst(j),'PortNumber');

                fprintf('  --> %s / Port %d\n', ...
                    dstParent, dstPortNumber);

            end
        end
    end
end

%% ------------------------------------------------------------
% 5. Inspect actual subsystem contents
% ------------------------------------------------------------

fprintf('\n============================================================\n');
fprintf('SENSOR SUBSYSTEM CONTENTS\n');
fprintf('============================================================\n');

subs = { ...
    'Synthetic_IMU', ...
    'Synthetic_AirData', ...
    'Navigation_EKF'};

for s = 1:numel(subs)

    root = [model '/' subs{s}];

    fprintf('\n============================================================\n');
    fprintf('%s\n',subs{s});
    fprintf('============================================================\n');

    blocks = find_system(root, ...
        'SearchDepth',Inf, ...
        'Type','Block');

    for b = 1:numel(blocks)

        block = blocks{b};

        fprintf('\n%s\n',block);

        try
            fprintf('  Type = %s\n',get_param(block,'BlockType'));
        catch
        end

        try
            fprintf('  Name = %s\n',get_param(block,'Name'));
        catch
        end

        try
            fprintf('  MaskType = %s\n',get_param(block,'MaskType'));
        catch
        end
    end
end

fprintf('\n============================================================\n');
fprintf('DONE — MODEL WAS NOT MODIFIED\n');
fprintf('============================================================\n');

close_system(model,0);