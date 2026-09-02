%% Von Mises toggle — energy vs torque duality figure
% Two stacked panels on a shared gamma axis:
%   (top)    spring energy   E(gamma) = 1/2 k (c - L0)^2,  c > L0
%   (bottom) restoring torque tau(gamma) = dE/dgamma
%                            = k (c - L0) * b R sin(gamma) / c
% with the band slack (E = 0, tau = 0) wherever the span c < L0.
%
% The torque curve is the classic N-shaped bistable force-displacement
% curve: zero crossings at the two stable states and at dead center
% (180 deg), extrema = snap-through limit torques.
%
% Edit the parameter block and run; exports toggle_energy_torque.pdf
% (vector) and .png (300 dpi) into cad/output.

%% Parameters
b      = 0.8803;   % frame leg P-S
R      = 1.7178;   % arm length P-T
gamma  = 104;      % state-1 pivot angle, deg (hard stop, measured)
gamma2 = 233;      % state-2 pivot angle, deg (hard stop, measured --
                   % plate bottom corner blocks the mirror 360-gamma)
L0     = 1.2;      % spring free length
k      = 1;        % spring stiffness (units of choice)

%% Model
span   = @(g) sqrt(b^2 + R^2 - 2*b*R*cosd(g));
energy = @(g) 0.5*k*max(span(g) - L0, 0).^2;
torque = @(g) k*max(span(g) - L0, 0) .* b.*R.*sind(g) ./ span(g);

g2 = gamma2;
gg = linspace(1, 359, 720);
[tmax, imax] = max(torque(gg));
fprintf('states at %g / %g deg (asymmetric hardware stops)\n', gamma, g2);
fprintf('barrier 1->2: dE = %.4f   barrier 2->1: dE = %.4f\n', ...
        energy(180) - energy(gamma), energy(180) - energy(g2));
fprintf('state-2 holds %.4f MORE energy than state 1\n', ...
        energy(g2) - energy(gamma));
fprintf('snap-through limit torque = %.4f at gamma = %.1f deg\n', ...
        tmax, gg(imax));

%% Figure
fig = figure('Units','centimeters','Position',[2 2 12.5 11], 'Color','w');
tl  = tiledlayout(fig, 2, 1, 'Padding','compact', 'TileSpacing','compact');

col.curveE = [0.11 0.62 0.46];
col.curveT = [0.73 0.46 0.09];
col.s1     = [0.35 0.33 0.75];
col.s2     = [0.83 0.33 0.49];
col.gray   = [0.45 0.45 0.42];

% (top) energy
ax1 = nexttile(tl); hold(ax1,'on');
plot(ax1, gg, energy(gg), '-', 'Color',col.curveE, 'LineWidth',1.5);
xline(ax1, 180, ':', 'Color',col.gray);
mark_states(ax1, @(g) energy(g), gamma, g2, col);
ylabel(ax1, 'energy E(\gamma)');
title(ax1, sprintf(['(a) energy landscape — hardware stops at ' ...
      '%g%c / %g%c'], gamma, 176, gamma2, 176), ...
      'FontWeight','normal', 'FontSize',9);
style_axis(ax1);

% (bottom) torque = dE/dgamma
ax2 = nexttile(tl); hold(ax2,'on');
yline(ax2, 0, '-', 'Color',col.gray, 'Alpha',0.5);
xline(ax2, 180, ':', 'Color',col.gray);
plot(ax2, gg, torque(gg), '-', 'Color',col.curveT, 'LineWidth',1.5);
mark_states(ax2, @(g) torque(g), gamma, g2, col);
ylim(ax2, [-1.25 1.25]*tmax);
yline(ax2, tmax, ':', '\tau_{crit}', 'Color',col.curveT, ...
      'FontSize',8.5, 'LabelHorizontalAlignment','left', 'Alpha',0.6);
text(ax2, 183, 0, ' dead center', 'FontSize',8.5, 'Color',col.gray, ...
     'VerticalAlignment','bottom');
xlabel(ax2, sprintf('pivot angle \\gamma (%c)', 176));
ylabel(ax2, 'torque \tau(\gamma) = dE/d\gamma');
title(ax2, '(b) restoring torque — the N-shaped bistable curve', ...
      'FontWeight','normal', 'FontSize',9);
style_axis(ax2);

%% Export
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
outd = fullfile(here, 'output');
if ~exist(outd, 'dir'), mkdir(outd); end
exportgraphics(fig, fullfile(outd, 'toggle_energy_torque.pdf'), ...
               'ContentType','vector');
exportgraphics(fig, fullfile(outd, 'toggle_energy_torque.png'), ...
               'Resolution',300);
fprintf('exported toggle_energy_torque.pdf / .png\n');

%% Helpers
function mark_states(ax, f, g1, g2, col)
    plot(ax, g1, f(g1), 'o', 'MarkerSize',6, ...
         'MarkerFaceColor',col.s1, 'MarkerEdgeColor','none');
    plot(ax, g2, f(g2), 'o', 'MarkerSize',6, 'LineWidth',1.4, ...
         'MarkerEdgeColor',col.s2, 'MarkerFaceColor','w');
    text(ax, g1, f(g1), 'state 1  ', 'FontSize',8.5, 'Color',col.s1, ...
         'VerticalAlignment','top', 'HorizontalAlignment','right');
    text(ax, g2, f(g2), 'state 2  ', 'FontSize',8.5, 'Color',col.s2, ...
         'VerticalAlignment','top', 'HorizontalAlignment','right');
end

function style_axis(ax)
    box(ax,'on'); grid(ax,'on');
    ax.GridAlpha = 0.12; ax.FontSize = 8.5;
    xlim(ax, [0 360]); xticks(ax, 0:90:360);
end
