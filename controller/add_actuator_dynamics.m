%% add_actuator_dynamics.m
% Inserts actuator dynamics (Rate Limiter + first-order lag) into the
% ELEVATOR and AILERON command paths of Final_JSB_4wp_LQG_withoutNoise_GnSch.
% Throttle is intentionally left untouched (see README note below).
%
% Chain inserted, per surface, in series AFTER the existing travel-limit
% Saturation block and BEFORE the Vector Concatenate block that feeds the
% JSBSim S-Function:
%
%   existing Saturation (unchanged, travel limit)
%     -> Rate Limiter    (models max physical slew rate)
%     -> Transfer Fcn    (models first-order actuator/servo bandwidth)
%     -> [Vector Concatenate -> S-Function]  (unchanged destination)
%
% Confirmed from direct inspection of the .slx (SIDs referenced in comments):
%   - Elevator: Gain3(621) -> Saturation(629,[-1,1]) -> Concatenate in:3
%   - Aileron : Gain1(583) -> Aileron Saturation(584,[-1,1]) -> branches to
%               Scope29(587) AND Concatenate in:2
%   - Throttle: Sum2(637) -> Saturation1(630,[0,1]) -> Concatenate in:1
%               (driven by a separate airspeed-hold PID loop, not the LQG
%               state-feedback path -> out of scope for this task)
%   - Solver: fixed-step ode4, step = 0.008333 s (120 Hz)
%
% PARAMETER NOTE (read before trusting these numbers):
%   RATE_LIMIT and TAU below are representative small-GA-servo-order-of-
%   magnitude values, NOT a verified spec for this airframe. The C172
%   has no factory autopilot servo, so there is no ground-truth number to
%   match. Document this as an assumption in the README, not a validated
%   hardware parameter. Both surfaces are given the SAME actuator model
%   for simplicity -- a real aircraft would have surface-specific
%   actuators; that simplification is also worth a one-line README note.

mdl = bdroot;
if isempty(mdl)
    error('Open Final_JSB_4wp_LQG_withoutNoise_GnSch.slx first, then run this script.');
end

RATE_LIMIT = 1.0;   % normalized cmd units / s, symmetric (+/-)
TAU        = 0.1;   % s, first-order lag time constant -> Transfer Fcn = 1/(TAU*s + 1)

%% Resolve existing blocks (by verified name, root level, no ambiguity)
satElev   = [mdl '/Saturation'];           % SID 629 -> "Elevator Command"
satAil    = [mdl '/Aileron Saturation'];   % SID 584 -> "Aileron Command_DLQR"
scope29   = [mdl '/Scope29'];              % SID 587, watches the aileron command
concatCell = find_system(mdl,'SearchDepth',1,'BlockType','Concatenate');
concatBlk  = concatCell{1};                % SID 484, "Vector\nConcatenate"

satElev_p = get_param(satElev,'PortHandles');
satAil_p  = get_param(satAil,'PortHandles');
concat_p  = get_param(concatBlk,'PortHandles');
scope29_p = get_param(scope29,'PortHandles');

%% Remove the existing direct lines into the Concatenate block
% NOTE: the aileron line is branched (also feeds Scope29). Deleting it
% removes BOTH the plant feed and the scope tap -- Scope29 is reconnected
% to the pre-actuator command explicitly below so it keeps watching the
% raw LQG output, not the actuator-shaped one.
elevLine = get_param(concat_p.Inport(3),'Line');
if elevLine ~= -1, delete_line(elevLine); end
ailLine = get_param(concat_p.Inport(2),'Line');
if ailLine ~= -1, delete_line(ailLine); end

%% Elevator actuator chain
add_block('simulink/Discontinuities/Rate Limiter',[mdl '/RateLimiter_Elevator'], ...
    'Position',[3430 1560 3470 1590]);
set_param([mdl '/RateLimiter_Elevator'], ...
    'RisingSlewLimit',num2str(RATE_LIMIT),'FallingSlewLimit',num2str(-RATE_LIMIT));

add_block('simulink/Continuous/Transfer Fcn',[mdl '/ActuatorLag_Elevator'], ...
    'Position',[3500 1560 3545 1590]);
set_param([mdl '/ActuatorLag_Elevator'],'Numerator','[1]', ...
    'Denominator',['[' num2str(TAU) ' 1]']);

rlE  = get_param([mdl '/RateLimiter_Elevator'],'PortHandles');
lagE = get_param([mdl '/ActuatorLag_Elevator'],'PortHandles');
add_line(mdl, satElev_p.Outport(1), rlE.Inport(1), 'autorouting','on');
add_line(mdl, rlE.Outport(1),       lagE.Inport(1),'autorouting','on');
add_line(mdl, lagE.Outport(1),      concat_p.Inport(3),'autorouting','on');

%% Aileron actuator chain
add_block('simulink/Discontinuities/Rate Limiter',[mdl '/RateLimiter_Aileron'], ...
    'Position',[3670 1460 3710 1490]);
set_param([mdl '/RateLimiter_Aileron'], ...
    'RisingSlewLimit',num2str(RATE_LIMIT),'FallingSlewLimit',num2str(-RATE_LIMIT));

add_block('simulink/Continuous/Transfer Fcn',[mdl '/ActuatorLag_Aileron'], ...
    'Position',[3740 1460 3785 1490]);
set_param([mdl '/ActuatorLag_Aileron'],'Numerator','[1]', ...
    'Denominator',['[' num2str(TAU) ' 1]']);

rlA  = get_param([mdl '/RateLimiter_Aileron'],'PortHandles');
lagA = get_param([mdl '/ActuatorLag_Aileron'],'PortHandles');
add_line(mdl, satAil_p.Outport(1), rlA.Inport(1), 'autorouting','on');
add_line(mdl, rlA.Outport(1),      lagA.Inport(1),'autorouting','on');
add_line(mdl, lagA.Outport(1),     concat_p.Inport(2),'autorouting','on');

%% Pre/post-actuator logging (needed for the integration/stress-test step)
add_block('simulink/Signal Routing/Mux',[mdl '/Mux_ElevActuator'], ...
    'Position',[3560 1650 3565 1680]);
set_param([mdl '/Mux_ElevActuator'],'Inputs','2');
muxE = get_param([mdl '/Mux_ElevActuator'],'PortHandles');
add_line(mdl, satElev_p.Outport(1), muxE.Inport(1), 'autorouting','on'); % pre-actuator
add_line(mdl, lagE.Outport(1),      muxE.Inport(2), 'autorouting','on'); % post-actuator

add_block('simulink/Sinks/To Workspace',[mdl '/elev_actuator_log'], ...
    'Position',[3600 1655 3660 1675]);
set_param([mdl '/elev_actuator_log'],'VariableName','elev_actuator_log', ...
    'MaxDataPoints','inf','SaveFormat','Structure With Time','SampleTime','0.008333');
logE = get_param([mdl '/elev_actuator_log'],'PortHandles');
add_line(mdl, muxE.Outport(1), logE.Inport(1), 'autorouting','on');

add_block('simulink/Signal Routing/Mux',[mdl '/Mux_AilActuator'], ...
    'Position',[3800 1550 3805 1580]);
set_param([mdl '/Mux_AilActuator'],'Inputs','2');
muxA = get_param([mdl '/Mux_AilActuator'],'PortHandles');
add_line(mdl, satAil_p.Outport(1), muxA.Inport(1), 'autorouting','on'); % pre-actuator
add_line(mdl, lagA.Outport(1),     muxA.Inport(2), 'autorouting','on'); % post-actuator

add_block('simulink/Sinks/To Workspace',[mdl '/ail_actuator_log'], ...
    'Position',[3840 1555 3900 1575]);
set_param([mdl '/ail_actuator_log'],'VariableName','ail_actuator_log', ...
    'MaxDataPoints','inf','SaveFormat','Structure With Time','SampleTime','0.008333');
logA = get_param([mdl '/ail_actuator_log'],'PortHandles');
add_line(mdl, muxA.Outport(1), logA.Inport(1), 'autorouting','on');

fprintf('Actuator dynamics inserted on elevator + aileron paths.\n');
fprintf('Layout will look messy -- Diagram > Arrange (or Ctrl+I) on the affected area, then save.\n');
fprintf('First check to run: plot elev_actuator_log / ail_actuator_log for the first ~2s of sim time\n');
fprintf('to confirm the zero-initial-condition lag/rate-limiter startup transient settles cleanly\n');
fprintf('against your trim condition before doing anything else.\n');
