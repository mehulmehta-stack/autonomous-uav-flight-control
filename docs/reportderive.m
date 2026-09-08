%% ============================================================
% DAY 9 — CURRENT MODEL ARCHITECTURE EXTRACTION
% READ-ONLY — DOES NOT MODIFY OR SAVE THE MODEL
% ============================================================

bdclose('all');
clearvars;
clc;

modelFile = 'Final_JSB_4wp_DAY8B_SENSOR_FIXED.slx';

assert(isfile(modelFile), ...
    'Model not found: %s\nCurrent folder: %s', ...
    modelFile, pwd);

[~, modelName] = fileparts(modelFile);

load_system(modelFile);

fprintf('\n============================================================\n');
fprintf('DAY 9 — CURRENT MODEL ARCHITECTURE EXTRACTION\n');
fprintf('============================================================\n');

fprintf('\nModel: %s\n', modelName);
fprintf('File : %s\n', modelFile);

%% ============================================================
% 1. ROOT-LEVEL BLOCKS
% ============================================================

fprintf('\n============================================================\n');
fprintf('ROOT-LEVEL BLOCKS\n');
fprintf('============================================================\n');

blocks = find_system(modelName, ...
    'SearchDepth', 1, ...
    'Type', 'Block');

for k = 1:numel(blocks)

    blk = blocks{k};

    if strcmp(blk, modelName)
        continue;
    end

    try
        name = get_param(blk, 'Name');
        type = get_param(blk, 'BlockType');

        fprintf('%-35s | %-20s\n', name, type);

    catch
    end
end

%% ============================================================
% 2. ALL ROOT-LEVEL SIGNAL CONNECTIONS
% ============================================================

fprintf('\n============================================================\n');
fprintf('ROOT-LEVEL SIGNAL CONNECTIONS\n');
fprintf('============================================================\n');

lines = find_system(modelName, ...
    'SearchDepth', 1, ...
    'FindAll', 'on', ...
    'Type', 'line');

for k = 1:numel(lines)

    h = lines(k);

    try

        srcPort = get_param(h, 'SrcPortHandle');

        if isempty(srcPort) || srcPort <= 0
            continue;
        end

        srcBlk = get_param(srcPort, 'Parent');
        srcPortNum = get_param(srcPort, 'PortNumber');

        dstPorts = get_param(h, 'DstPortHandle');

        if isempty(dstPorts)
            continue;
        end

        for j = 1:numel(dstPorts)

            if dstPorts(j) <= 0
                continue;
            end

            dstBlk = get_param(dstPorts(j), 'Parent');
            dstPortNum = get_param(dstPorts(j), 'PortNumber');

            fprintf('%s/%d  -->  %s/%d\n', ...
                get_param(srcBlk,'Name'), ...
                srcPortNum, ...
                get_param(dstBlk,'Name'), ...
                dstPortNum);
        end

    catch
        % Ignore malformed/non-signal line objects.
    end
end

%% ============================================================
% 3. IMPORTANT CONTROLLER BLOCKS
% ============================================================

importantNames = { ...
    'Navigation_EKF', ...
    'Synthetic_IMU', ...
    'Synthetic_GPS', ...
    'Synthetic_AirData', ...
    'Mux', ...
    'Mux1', ...
    'Gain', ...
    'Gain1', ...
    'Gain2', ...
    'Gain3', ...
    'Gain4', ...
    'L1_Guidance', ...
    'AUTOPILOT'};

fprintf('\n============================================================\n');
fprintf('IMPORTANT BLOCK DETAILS\n');
fprintf('============================================================\n');

for k = 1:numel(importantNames)

    name = importantNames{k};

    matches = find_system(modelName, ...
        'SearchDepth', Inf, ...
        'Type', 'Block', ...
        'Name', name);

    if isempty(matches)
        fprintf('\n[%s] NOT FOUND\n', name);
        continue;
    end

    for j = 1:numel(matches)

        blk = matches{j};

        fprintf('\n------------------------------------------------------------\n');
        fprintf('BLOCK: %s\n', blk);
        fprintf('------------------------------------------------------------\n');

        try
            fprintf('BlockType : %s\n', ...
                get_param(blk,'BlockType'));
        catch
        end

        try
            fprintf('MaskType  : %s\n', ...
                get_param(blk,'MaskType'));
        catch
        end

        try
            fprintf('Gain      : ');
            disp(get_param(blk,'Gain'));
        catch
        end

        try
            fprintf('Value     : ');
            disp(get_param(blk,'Value'));
        catch
        end

        try
            fprintf('Inputs    : %s\n', ...
                get_param(blk,'Inputs'));
        catch
        end

        try
            fprintf('Outputs   : %s\n', ...
                get_param(blk,'Outputs'));
        catch
        end
    end
end

%% ============================================================
% 4. AUTOPILOT SUBSYSTEM INTERNAL CONNECTIONS
% ============================================================

ap = find_system(modelName, ...
    'SearchDepth', Inf, ...
    'Type', 'Block', ...
    'Name', 'AUTOPILOT');

if ~isempty(ap)

    ap = ap{1};

    fprintf('\n============================================================\n');
    fprintf('AUTOPILOT SUBSYSTEM INTERNAL BLOCKS\n');
    fprintf('============================================================\n');

    apBlocks = find_system(ap, ...
        'SearchDepth', 1, ...
        'Type', 'Block');

    for k = 1:numel(apBlocks)

        blk = apBlocks{k};

        if strcmp(blk, ap)
            continue;
        end

        try
            fprintf('%-35s | %-20s\n', ...
                get_param(blk,'Name'), ...
                get_param(blk,'BlockType'));
        catch
        end
    end

    fprintf('\n============================================================\n');
    fprintf('AUTOPILOT INTERNAL CONNECTIONS\n');
    fprintf('============================================================\n');

    apLines = find_system(ap, ...
        'SearchDepth', 1, ...
        'FindAll', 'on', ...
        'Type', 'line');

    for k = 1:numel(apLines)

        h = apLines(k);

        try

            srcPort = get_param(h,'SrcPortHandle');

            if srcPort <= 0
                continue;
            end

            srcBlk = get_param(srcPort,'Parent');
            srcNum = get_param(srcPort,'PortNumber');

            dstPorts = get_param(h,'DstPortHandle');

            for j = 1:numel(dstPorts)

                if dstPorts(j) <= 0
                    continue;
                end

                dstBlk = get_param(dstPorts(j),'Parent');
                dstNum = get_param(dstPorts(j),'PortNumber');

                fprintf('%s/%d  -->  %s/%d\n', ...
                    get_param(srcBlk,'Name'), ...
                    srcNum, ...
                    get_param(dstBlk,'Name'), ...
                    dstNum);
            end

        catch
        end
    end
end

%% ============================================================
% 5. NAVIGATION EKF DETAILS
% ============================================================

ekf = find_system(modelName, ...
    'SearchDepth', Inf, ...
    'Type', 'Block', ...
    'Name', 'Navigation_EKF');

if ~isempty(ekf)

    ekf = ekf{1};

    fprintf('\n============================================================\n');
    fprintf('NAVIGATION EKF DETAILS\n');
    fprintf('============================================================\n');

    fprintf('Path: %s\n', ekf);

    params = {'FunctionName','Parameters','SFunctionName'};

    for k = 1:numel(params)

        try
            fprintf('%s: ', params{k});
            disp(get_param(ekf,params{k}));
        catch
        end
    end
end

%% ============================================================
% 6. SAVE TEXT REPORT
% ============================================================

reportFile = 'DAY9_CURRENT_MODEL_ARCHITECTURE.txt';

diary(reportFile);

fprintf('\n\n============================================================\n');
fprintf('DAY 9 ARCHITECTURE EXTRACTION COMPLETE\n');
fprintf('============================================================\n');

diary off;

fprintf('\nReport written to:\n%s\n', ...
    fullfile(pwd, reportFile));

fprintf('\nIMPORTANT: Model was NOT modified or saved.\n');

close_system(modelName, 0);