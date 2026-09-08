%% ============================================================
% DAY 9 — IMU DATA EXTRACTION
% Log the six scalar Demux1 IMU channels
% ============================================================

clc;

fprintf('\n============================================================\n');
fprintf('DAY 9 — IMU DATA EXTRACTION\n');
fprintf('============================================================\n');

mdl = 'Final_JSB_4wp_DAY8B_SENSOR_FIXED';

load_system(mdl);

%% ------------------------------------------------------------
% Temporary To Workspace blocks
% ------------------------------------------------------------

names = {'ax','ay','az','gx','gy','gz'};

for k = 1:6

    blk = [mdl '/DAY9_TEMP_' names{k}];

    if ~isempty(find_system(mdl,'SearchDepth',1, ...
            'Name',['DAY9_TEMP_' names{k}]))
        delete_block(blk);
    end

    add_block('simulink/Sinks/To Workspace',blk, ...
        'VariableName',['DAY9_' names{k}], ...
        'SaveFormat','Timeseries', ...
        'Position',[1800 300+80*k 2000 350+80*k]);

    % Demux1 output k -> temporary logger
    add_line(mdl, ...
        ['Demux1/' num2str(k)], ...
        ['DAY9_TEMP_' names{k} '/1']);
end

%% ------------------------------------------------------------
% Run simulation
% ------------------------------------------------------------

fprintf('\nRunning 120-second simulation...\n');

sim(mdl);

fprintf('Simulation complete.\n');

%% ------------------------------------------------------------
% Read six scalar signals
% ------------------------------------------------------------

T = evalin('base','DAY9_ax');

t  = T.Time;

ax = T.Data;
ay = evalin('base','DAY9_ay').Data;
az = evalin('base','DAY9_az').Data;
gx = evalin('base','DAY9_gx').Data;
gy = evalin('base','DAY9_gy').Data;
gz = evalin('base','DAY9_gz').Data;

%% ------------------------------------------------------------
% Remove temporary blocks
% ------------------------------------------------------------

for k = 1:6
    delete_block([mdl '/DAY9_TEMP_' names{k}]);
end

%% ------------------------------------------------------------
% Verify
% ------------------------------------------------------------

fprintf('\nSamples : %d\n',length(t));
fprintf('Time    : %.3f -> %.3f s\n',t(1),t(end));

fprintf('\nChannel sizes:\n');
fprintf('ax = %d\n',length(ax));
fprintf('ay = %d\n',length(ay));
fprintf('az = %d\n',length(az));
fprintf('gx = %d\n',length(gx));
fprintf('gy = %d\n',length(gy));
fprintf('gz = %d\n',length(gz));

%% ------------------------------------------------------------
% Statistics after startup
% ------------------------------------------------------------

idx = t >= 10;

fprintf('\n============================================================\n');
fprintf('IMU STATISTICS — t >= 10 s\n');
fprintf('============================================================\n');

fprintf('\nSpecific force [m/s^2]\n');
fprintf('ax : mean = %+.6f   std = %.6f\n',mean(ax(idx)),std(ax(idx)));
fprintf('ay : mean = %+.6f   std = %.6f\n',mean(ay(idx)),std(ay(idx)));
fprintf('az : mean = %+.6f   std = %.6f\n',mean(az(idx)),std(az(idx)));

fprintf('\nGyroscope [rad/s]\n');
fprintf('gx : mean = %+.6f   std = %.6f\n',mean(gx(idx)),std(gx(idx)));
fprintf('gy : mean = %+.6f   std = %.6f\n',mean(gy(idx)),std(gy(idx)));
fprintf('gz : mean = %+.6f   std = %.6f\n',mean(gz(idx)),std(gz(idx)));

%% ------------------------------------------------------------
% Save
% ------------------------------------------------------------

imu = [ax ay az gx gy gz];

save('DAY9_IMU_DATA.mat', ...
    't','imu','ax','ay','az','gx','gy','gz');

fprintf('\nSaved: DAY9_IMU_DATA.mat\n');

fprintf('\n============================================================\n');
fprintf('DAY 9 — IMU EXTRACTION COMPLETE\n');
fprintf('============================================================\n');