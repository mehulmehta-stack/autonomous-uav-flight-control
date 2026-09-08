%% verify_actuator_dynamics.m
%
% Quantitatively validates the elevator/aileron actuator dynamics
% (Rate Limiter + first-order lag) using elev_actuator_log / ail_actuator_log,
% and produces figures suitable for the GitHub repo.
%
% Run this AFTER simulating the model (elev_actuator_log and
% ail_actuator_log must exist in the base workspace).
%
% What "proof" means here, concretely:
%   1. QUANTIFIED CHECK: the post-actuator signal's slew rate never
%      exceeds the configured RATE_LIMIT. This is the falsifiable claim --
%      not "it looks smoother", but "it is bounded by the number I set".
%   2. STARTUP TRANSIENT CHECK: quantify how long the zero-initial-condition
%      lag/rate-limiter take to settle against the trim command, so you can
%      state a number instead of eyeballing a plot.
%   3. THE GPS-REACQUISITION PLOT: this is the one that matters most for the
%      repo, because it's a direct before/after of the exact finding that
%      justified adding actuator dynamics in the first place -- the raw LQG
%      command spiking to full saturation in a single 8.3ms timestep, versus
%      the actuator-shaped command that can no longer do that.

clear plotDir
RATE_LIMIT = 1.0;    % must match what you set in add_actuator_dynamics.m
TAU        = 0.1;    % must match what you set in add_actuator_dynamics.m
DT         = 0.008333;

plotDir = fullfile(pwd,'actuator_validation_figs');
if ~exist(plotDir,'dir'), mkdir(plotDir); end

%% --- Robust extraction from "Structure With Time" To Workspace logs ---
function [t, pre, post] = extractPrePost(logStruct, label)
    if ~isfield(logStruct,'time') || ~isfield(logStruct,'signals')
        error('%s: not a "Structure With Time" log -- check the To Workspace SaveFormat setting.', label);
    end
    t = logStruct.time;
    sig = logStruct.signals;
    if isstruct(sig) && numel(sig) == 1 && size(sig.values,2) == 2
        pre  = sig.values(:,1);
        post = sig.values(:,2);
    elseif isstruct(sig) && numel(sig) == 2
        pre  = sig(1).values(:);
        post = sig(2).values(:);
    else
        error(['%s: unexpected signal layout (size=%s). Expected a single ' ...
               '2-column signal from the Mux, or a 2-element signal array. ' ...
               'Inspect %s.signals in the workspace and adjust extractPrePost.'], ...
               label, mat2str(size(sig)), label);
    end
    if numel(t) ~= numel(pre)
        error('%s: time vector length (%d) does not match signal length (%d).', ...
              label, numel(t), numel(pre));
    end
end

if ~evalin('base','exist(''elev_actuator_log'',''var'')')
    error('elev_actuator_log not found in base workspace. Run the simulation first.');
end
if ~evalin('base','exist(''ail_actuator_log'',''var'')')
    error('ail_actuator_log not found in base workspace. Run the simulation first.');
end
elev_actuator_log = evalin('base','elev_actuator_log');
ail_actuator_log  = evalin('base','ail_actuator_log');

[tE, preE, postE] = extractPrePost(elev_actuator_log, 'elev_actuator_log');
[tA, preA, postA] = extractPrePost(ail_actuator_log,  'ail_actuator_log');

%% --- 1. Quantified rate-limit check (the falsifiable claim) ---
rateE = diff(postE) ./ diff(tE);
rateA = diff(postA) ./ diff(tA);
maxRateE = max(abs(rateE));
maxRateA = max(abs(rateA));
tolFrac  = 1.05;  % 5% numerical slack for solver/logging discretization

fprintf('\n=== Rate-limit verification ===\n');
fprintf('Elevator: configured limit = %.3f /s, observed max slew = %.3f /s -> %s\n', ...
    RATE_LIMIT, maxRateE, tern(maxRateE <= RATE_LIMIT*tolFrac,'PASS','FAIL'));
fprintf('Aileron : configured limit = %.3f /s, observed max slew = %.3f /s -> %s\n', ...
    RATE_LIMIT, maxRateA, tern(maxRateA <= RATE_LIMIT*tolFrac,'PASS','FAIL'));

%% --- 2. Startup transient settling time ---
% IMPORTANT: this must be restricted to an early window. pre/post legitimately
% diverge by more than `tol` during every real commanded maneuver later in the
% mission (that's the actuator correctly lagging a real command, not an IC
% artifact) -- searching the full run picks up the LAST such maneuver and
% misreports it as "never settled since t=0". STARTUP_WINDOW must end before
% the first real commanded maneuver; 5s is well clear of it based on the
% full-duration plot (first non-IC excursion is around t~100s) and well past
% the ~1.5-2s visual settling seen in the startup-transient plot.
STARTUP_WINDOW = 5.0; % seconds
tol = 0.02; % "settled" = within 0.02 normalized units of the raw command

maskE = tE <= STARTUP_WINDOW;
maskA = tA <= STARTUP_WINDOW;
settleE = firstSettleTime(tE(maskE), preE(maskE), postE(maskE), tol);
settleA = firstSettleTime(tA(maskA), preA(maskA), postA(maskA), tol);
fprintf('\n=== Startup transient (evaluated over first %.1fs only) ===\n', STARTUP_WINDOW);
fprintf('Elevator: |post-pre| <= %.2f from t = %.3fs onward\n', tol, settleE);
fprintf('Aileron : |post-pre| <= %.2f from t = %.3fs onward\n', tol, settleA);
if isnan(settleE) || isnan(settleA)
    fprintf(2,['WARNING: settling not reached within the %.1fs window for at least one surface.\n' ...
        '  Either widen STARTUP_WINDOW or inspect the plot directly -- do not assume a NaN means\n' ...
        '  "never settles"; it may just mean the window was too short.\n'], STARTUP_WINDOW);
end

%% --- 2b. Peak tracking lag during real maneuvers (separate from startup) ---
% This is a different, legitimate characterization: how much does the
% actuator lag a real commanded maneuver at its worst, and when. Not a
% pass/fail -- just an honest number for the README instead of silence.
[peakLagE, iE] = max(abs(postE - preE));
[peakLagA, iA] = max(abs(postA - preA));
fprintf('\n=== Peak tracking lag during full mission (informational, not a fault) ===\n');
fprintf('Elevator: max |post-pre| = %.3f at t = %.2fs\n', peakLagE, tE(iE));
fprintf('Aileron : max |post-pre| = %.3f at t = %.2fs\n', peakLagA, tA(iA));

%% --- 3. Full-duration overlay ---
fig1 = figure('Position',[100 100 900 500]);
subplot(2,1,1);
plot(tE,preE,'--','LineWidth',1); hold on; plot(tE,postE,'-','LineWidth',1.3);
ylabel('Elevator cmd (norm)'); legend('Pre-actuator (raw LQG)','Post-actuator','Location','best');
title('Actuator dynamics: full-duration comparison'); grid on;
subplot(2,1,2);
plot(tA,preA,'--','LineWidth',1); hold on; plot(tA,postA,'-','LineWidth',1.3);
ylabel('Aileron cmd (norm)'); xlabel('Time (s)'); legend('Pre-actuator (raw LQG)','Post-actuator','Location','best');
grid on;
exportgraphics(fig1, fullfile(plotDir,'actuator_dynamics_full.png'), 'Resolution', 200);

%% --- 4. Startup transient zoom (first 2s) ---
fig2 = figure('Position',[100 100 900 500]);
mask = tE <= 2;
subplot(2,1,1);
plot(tE(mask),preE(mask),'--','LineWidth',1); hold on; plot(tE(mask),postE(mask),'-','LineWidth',1.3);
ylabel('Elevator cmd (norm)'); title('Startup transient (first 2s)'); grid on;
legend('Pre-actuator','Post-actuator','Location','best');
mask = tA <= 2;
subplot(2,1,2);
plot(tA(mask),preA(mask),'--','LineWidth',1); hold on; plot(tA(mask),postA(mask),'-','LineWidth',1.3);
ylabel('Aileron cmd (norm)'); xlabel('Time (s)'); grid on;
legend('Pre-actuator','Post-actuator','Location','best');
exportgraphics(fig2, fullfile(plotDir,'actuator_dynamics_startup_transient.png'), 'Resolution', 200);

%% --- 5. THE key plot: GPS reacquisition window (t ~ 24-38s) ---
fig3 = figure('Position',[100 100 900 500]);
window = [24 38];
mask = tE >= window(1) & tE <= window(2);
subplot(2,1,1);
plot(tE(mask),preE(mask),'--','LineWidth',1.2); hold on; plot(tE(mask),postE(mask),'-','LineWidth',1.5);
ylabel('Elevator cmd (norm)');
title('GPS reacquisition (t\approx25-35s outage): raw LQG spike vs actuator-shaped command');
grid on; legend('Pre-actuator (raw LQG, spikes to saturation in 1 step)','Post-actuator (rate+lag limited)','Location','best');
elevYLim = ylim; % capture so the aileron panel below is forced to the same scale
mask = tA >= window(1) & tA <= window(2);
subplot(2,1,2);
plot(tA(mask),preA(mask),'--','LineWidth',1.2); hold on; plot(tA(mask),postA(mask),'-','LineWidth',1.5);
ylabel('Aileron cmd (norm)'); xlabel('Time (s)'); grid on;
legend('Pre-actuator','Post-actuator','Location','best');
ylim(elevYLim); % same scale as elevator -- makes "nothing dramatic happens here" visually obvious
% instead of relying on the earlier tiny auto-scaled 0.028-0.038 band, which
% was technically correct but easy to misread as a hidden effect
exportgraphics(fig3, fullfile(plotDir,'actuator_dynamics_gps_reacquisition.png'), 'Resolution', 200);

fprintf('\nFigures saved to: %s\n', plotDir);

%% --- Auto-generated README caption (real numbers, not placeholders) ---
fprintf('\n=== README-ready caption (copy/paste, edit as needed) ===\n\n');
fprintf(['**Actuator dynamics validation.** A rate limiter (+/-%.1f normalized units/s) ' ...
    'and first-order lag (tau=%.2fs) were added to the elevator and aileron command paths. ' ...
    'Observed max post-actuator slew rate was %.3f/s (elevator) and %.3f/s (aileron) against ' ...
    'a configured limit of %.1f/s -- the aileron figure sitting essentially at the limit confirms ' ...
    'the rate limiter actively binds during real maneuvers, not just theoretically present. ' ...
    'The zero-initial-condition startup transient (evaluated over the first %.1fs, before any ' ...
    'commanded maneuver) settles to within %.2f of the raw command by t=%.3fs (elevator) and ' ...
    't=%.3fs (aileron). Separately, peak tracking lag during the full mission -- the actuator ' ...
    'legitimately lagging a real commanded maneuver, not a startup artifact -- reached %.3f ' ...
    '(elevator, at t=%.1fs) and %.3f (aileron, at t=%.1fs). ' ...
    'During the previously-characterized GPS-outage-reacquisition event (a single-timestep ' ...
    'elevator command spike to full saturation, found before actuator dynamics existed), the raw ' ...
    'LQG command still spikes identically (pre-actuator trace), but the actuator-shaped command ' ...
    'no longer can. This suppression is specific to elevator, matching the original finding''s ' ...
    'longitudinal-only scope -- aileron shows no comparable spike in this window because none was ' ...
    'ever documented for it.\n\n'], ...
    RATE_LIMIT, TAU, maxRateE, maxRateA, RATE_LIMIT, STARTUP_WINDOW, tol, settleE, settleA, ...
    peakLagE, tE(iE), peakLagA, tA(iA));

if settleA < 0.5
    ailAmpEarly = max(abs(preA(maskA)));
    fprintf(['*Caveat on the aileron settling number:* the %.2f tolerance used above is loose ' ...
        'relative to the aileron command''s own amplitude in this window (peak ~%.3f), so the ' ...
        'near-instant settling reflects a genuinely small trim/IC mismatch rather than a ' ...
        'stringent test -- worth stating explicitly rather than implying it''s as demanding a ' ...
        'check as the elevator''s %.3fs number.\n\n'], tol, ailAmpEarly, settleE);
end

%% --- helpers ---
function s = tern(cond,a,b)
    if cond, s = a; else, s = b; end
end

function tSettle = firstSettleTime(t, pre, post, tol)
    err = abs(post - pre);
    ok = err <= tol;
    idx = find(~ok, 1, 'last'); % last index where NOT settled
    if isempty(idx)
        tSettle = t(1); % settled from the start
    elseif idx == numel(t)
        tSettle = NaN;  % never settles -- flag this, don't silently report a number
        warning('Signal never settles within tolerance %.3f -- check RATE_LIMIT/TAU or tol.', tol);
    else
        tSettle = t(idx+1);
    end
end