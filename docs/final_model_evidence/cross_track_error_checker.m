%% CROSS-TRACK ERROR -- quantifies leg straightness objectively
% Uses val_state (position) and val_L1wpidx (active segment) already
% in the workspace from your last run. Computes perpendicular distance
% from the flown path to whichever straight leg the guidance law is
% currently tracking -- this is the real number "how wiggly is this,
% actually" reduces to.

clc;
lon = val_state.signals.values(:,8);
lat = val_state.signals.values(:,9);
t   = val_state.time;
wpidx = val_L1wpidx.signals.values(:,1);

lat0 = 47.0; lon0 = -122.0; R_earth = 6378137;
N = (lat - lat0) * (pi/180) * R_earth;
E = (lon - lon0) * (pi/180) * R_earth * cosd(lat0);

WP1 = [-4500, -4500]; WP2 = [0, -9000]; WP3 = [4500, -4500]; WP4 = [0, 0];

xte = zeros(size(N));
for i = 1:numel(N)
    switch wpidx(i)
        case 1, sN=WP4(1); sE=WP4(2); eN=WP1(1); eE=WP1(2);
        case 2, sN=WP1(1); sE=WP1(2); eN=WP2(1); eE=WP2(2);
        case 3, sN=WP2(1); sE=WP2(2); eN=WP3(1); eE=WP3(2);
        otherwise, sN=WP3(1); sE=WP3(2); eN=WP4(1); eE=WP4(2);
    end
    pN = eN - sN; pE = eE - sE;
    plen = sqrt(pN^2 + pE^2);
    % perpendicular (cross-track) distance, signed
    xte(i) = ((N(i)-sN)*pE - (E(i)-sE)*pN) / plen;
end

figure('Position',[100 100 900 500]);
plot(t, xte, 'r'); grid on;
xlabel('Time (s)'); ylabel('Cross-track error (m)');
title('Perpendicular deviation from the active ideal leg');
saveas(gcf, 'CrossTrackError.png');

fprintf('RMS cross-track error: %.1f m\n', rms(xte));
fprintf('Max |cross-track error|: %.1f m\n', max(abs(xte)));
fprintf('For reference: leg length = 6364 m, ideal turn radius = 1061 m\n');