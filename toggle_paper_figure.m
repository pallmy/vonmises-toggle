%% Von Mises over-center toggle — publication figure
% Parametric model (same as toggle_sketch.py):
%   P = main pivot (origin), S = spring anchor at distance b, direction phi
%   T = arm tip at distance R, driven by pivot angle gamma
%   Bistable pair: gamma and 360-gamma (arm mirrored across frame line P-S),
%   equal span -> equal spring energy; dead center at gamma = 180 deg.
%
% Edit the parameter block, run, and it exports toggle_paper_figure.pdf
% (vector) and .png (300 dpi) next to this file.

%% Parameters
b       = 0.8803;    % frame leg P-S
R       = 1.7178;    % arm length P-T
gamma   = 104;       % state-1 pivot angle, deg
phi     = 127.53;    % frame leg direction from horizontal, deg
L0      = 1.2;       % spring free length
k       = 1;         % spring stiffness (units of choice)

%% Geometry
Pp  = [0 0];
S   = b*[cosd(phi)  sind(phi)];
T1  = R*[cosd(phi-gamma) sind(phi-gamma)];
T2  = R*[cosd(phi+gamma) sind(phi+gamma)];          % mirrored state
Tdc = -R*S/norm(S);                                  % dead center tip
span   = @(g) sqrt(b^2 + R^2 - 2*b*R*cosd(g));
energy = @(g) 0.5*k*max(span(g) - L0, 0).^2;
c1 = span(gamma);
dE = energy(180) - energy(gamma);

fprintf('span = %.4f   stretch = %.4f   barrier dE = %.4f\n', ...
        c1, c1 - L0, dE);

%% Figure
fig = figure('Units','centimeters','Position',[2 2 17.4 7.6], ...
             'Color','w');
tl = tiledlayout(fig, 1, 2, 'Padding','compact', 'TileSpacing','compact');

col.arm1   = [0.35 0.33 0.75];
col.arm2   = [0.83 0.33 0.49];
col.spring = [0.11 0.62 0.46];
col.frame  = [0.45 0.45 0.42];

% (a) mechanism, both states
ax1 = nexttile(tl); hold(ax1,'on');
plot(ax1, [S(1) Tdc(1)], [S(2) Tdc(2)], ':', 'Color',col.frame, ...
     'LineWidth',0.6);
plot(ax1, [Pp(1) S(1)], [Pp(2) S(2)], '-', 'Color',col.frame, ...
     'LineWidth',1.6);
sp2 = spring_pts(S, T2);
plot(ax1, sp2(:,1), sp2(:,2), '-', 'Color',[col.spring 0.35], ...
     'LineWidth',0.7);
plot(ax1, [Pp(1) T2(1)], [Pp(2) T2(2)], '--', 'Color',col.arm2, ...
     'LineWidth',1.6);
sp1 = spring_pts(S, T1);
plot(ax1, sp1(:,1), sp1(:,2), '-', 'Color',col.spring, 'LineWidth',1.0);
plot(ax1, [Pp(1) T1(1)], [Pp(2) T1(2)], '-', 'Color',col.arm1, ...
     'LineWidth',2.2);
plot(ax1, Pp(1), Pp(2), 'o', 'MarkerSize',6, 'MarkerEdgeColor',col.frame, ...
     'MarkerFaceColor','w', 'LineWidth',1.2);
plot(ax1, S(1), S(2), 'o', 'MarkerSize',5, 'MarkerEdgeColor',col.frame, ...
     'MarkerFaceColor','w', 'LineWidth',1.2);
plot(ax1, T1(1), T1(2), 'o', 'MarkerSize',7, ...
     'MarkerFaceColor',col.arm1, 'MarkerEdgeColor','none');
plot(ax1, T2(1), T2(2), 'o', 'MarkerSize',7, 'LineWidth',1.4, ...
     'MarkerEdgeColor',col.arm2, 'MarkerFaceColor','w');
text(ax1, Pp(1)-0.10, Pp(2)+0.12, 'P', 'FontSize',9);
text(ax1, S(1)-0.16,  S(2)+0.12,  'S', 'FontSize',9);
text(ax1, T1(1)+0.08, T1(2)+0.10, 'state 1', 'FontSize',8.5, ...
     'Color',col.arm1);
text(ax1, T2(1)+0.08, T2(2)-0.10, 'state 2', 'FontSize',8.5, ...
     'Color',col.arm2);
axis(ax1,'equal'); box(ax1,'on'); grid(ax1,'on');
ax1.GridAlpha = 0.12; ax1.FontSize = 8.5;
xlabel(ax1,'x'); ylabel(ax1,'y');
title(ax1, sprintf(['(a) linkage, \\gamma = %g%c / %g%c, ', ...
      'span = %.3f'], gamma, 176, 360-gamma, 176, c1), ...
      'FontWeight','normal', 'FontSize',9);

% (b) energy landscape
ax2 = nexttile(tl); hold(ax2,'on');
gg = linspace(5, 355, 400);
plot(ax2, gg, energy(gg), '-', 'Color',col.spring, 'LineWidth',1.4);
xline(ax2, 180, ':', 'Color',col.frame, 'LineWidth',0.8);
plot(ax2, gamma, energy(gamma), 'o', 'MarkerSize',6, ...
     'MarkerFaceColor',col.arm1, 'MarkerEdgeColor','none');
plot(ax2, 360-gamma, energy(360-gamma), 'o', 'MarkerSize',6, ...
     'LineWidth',1.4, 'MarkerEdgeColor',col.arm2, 'MarkerFaceColor','w');
text(ax2, gamma, energy(gamma), '  state 1', 'FontSize',8.5, ...
     'Color',col.arm1, 'VerticalAlignment','top');
text(ax2, 360-gamma, energy(360-gamma), 'state 2  ', 'FontSize',8.5, ...
     'Color',col.arm2, 'VerticalAlignment','top', ...
     'HorizontalAlignment','right');
text(ax2, 182, energy(180), sprintf(' \\DeltaE = %.3f', dE), ...
     'FontSize',8.5, 'Color',col.frame);
box(ax2,'on'); grid(ax2,'on');
ax2.GridAlpha = 0.12; ax2.FontSize = 8.5;
xlim(ax2, [0 360]); xticks(ax2, 0:90:360);
xlabel(ax2, sprintf('pivot angle \\gamma (%c)', 176));
ylabel(ax2, 'spring energy E(\gamma)');
title(ax2, sprintf('(b) energy landscape, L_0 = %.2f, k = %g', L0, k), ...
      'FontWeight','normal', 'FontSize',9);

%% Export
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
exportgraphics(fig, fullfile(here, 'output', 'toggle_paper_figure.pdf'), ...
               'ContentType','vector');
exportgraphics(fig, fullfile(here, 'output', 'toggle_paper_figure.png'), ...
               'Resolution',300);
fprintf('exported toggle_paper_figure.pdf / .png\n');

%% Helpers
function pts = spring_pts(A, B)
    n = 26; amp = 0.045;
    d = B - A; u = d/norm(d); nrm = [-u(2) u(1)];
    pts = zeros(n+1, 2); pts(1,:) = A; pts(end,:) = B;
    for i = 1:n-1
        f = i/n;
        off = amp * (2*mod(i,2)-1) * (i > 1 && i < n-1);
        pts(i+1,:) = A + d*f + nrm*off;
    end
end
