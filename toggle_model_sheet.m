%% Von Mises toggle — planform plot
% Simple presentation view: the mechanism on a warm beige background,
% with the interior of the planform (triangle P-S-T) filled in a
% darker shade. State 1 is filled solid; the mirrored state 2 is
% filled faintly behind it.
%
% Edit the parameter block and run; exports toggle_model_sheet.pdf
% (vector) and .png (300 dpi) into cad/output.

%% Parameters
b     = 0.8803;    % frame leg P-S
R     = 1.7178;    % arm length P-T
gamma = 104;       % state-1 pivot angle, deg
phi   = 127.53;    % frame leg direction, deg (drawing only)

%% Geometry
P  = [0 0];
S  = b*[cosd(phi) sind(phi)];
T1 = R*[cosd(phi-gamma) sind(phi-gamma)];
T2 = R*[cosd(phi+gamma) sind(phi+gamma)];
Td = -R*S/norm(S);                        % dead-center tip
c  = norm(T1 - S);

%% Colors
bg        = [0.93 0.91 0.86];             % beige page
fill1     = [0.84 0.78 0.66];             % planform interior, state 1
fill2     = [0.89 0.86 0.79];             % planform interior, state 2
col.arm1  = [0.35 0.33 0.75];
col.arm2  = [0.83 0.33 0.49];
col.sprng = [0.11 0.62 0.46];
col.frame = [0.42 0.40 0.36];
col.ink   = [0.25 0.24 0.21];

%% Figure
fig = figure('Units','centimeters', 'Position',[2 2 18 15], 'Color',bg);
ax  = axes(fig, 'Position',[0.04 0.06 0.92 0.86], 'Color',bg);
hold(ax, 'on');

% filled planforms first, so everything else draws on top
fill(ax, [P(1) S(1) T2(1)], [P(2) S(2) T2(2)], fill2, ...
     'EdgeColor','none');
fill(ax, [P(1) S(1) T1(1)], [P(2) S(2) T1(2)], fill1, ...
     'EdgeColor','none');

% dead-center reference
plot(ax, [S(1) Td(1)], [S(2) Td(2)], ':', 'Color',col.frame);

% state 2 (ghost)
sp2 = spring_pts(S, T2);
plot(ax, sp2(:,1), sp2(:,2), '-', 'Color',[col.sprng 0.35]);
plot(ax, [P(1) T2(1)], [P(2) T2(2)], '--', 'Color',col.arm2, ...
     'LineWidth',1.6);

% frame leg, state-1 spring and arm
plot(ax, [P(1) S(1)], [P(2) S(2)], '-', 'Color',col.frame, ...
     'LineWidth',2.0);
sp1 = spring_pts(S, T1);
plot(ax, sp1(:,1), sp1(:,2), '-', 'Color',col.sprng, 'LineWidth',1.2);
plot(ax, [P(1) T1(1)], [P(2) T1(2)], '-', 'Color',col.arm1, ...
     'LineWidth',2.6);

% joints
plot(ax, P(1), P(2), 'o', 'MarkerSize',7, 'MarkerEdgeColor',col.frame, ...
     'MarkerFaceColor',bg, 'LineWidth',1.4);
plot(ax, S(1), S(2), 'o', 'MarkerSize',6, 'MarkerEdgeColor',col.frame, ...
     'MarkerFaceColor',bg, 'LineWidth',1.4);
plot(ax, T1(1), T1(2), 'o', 'MarkerSize',9, ...
     'MarkerFaceColor',col.arm1, 'MarkerEdgeColor','none');
plot(ax, T2(1), T2(2), 'o', 'MarkerSize',9, 'LineWidth',1.5, ...
     'MarkerEdgeColor',col.arm2, 'MarkerFaceColor',bg);

% labels
text(ax, P(1)+0.10, P(2)-0.15, 'P', 'FontSize',11, 'Color',col.ink);
text(ax, S(1)-0.20, S(2)+0.14, 'S', 'FontSize',11, 'Color',col.ink);
text(ax, T1(1)-0.05, T1(2)-0.18, 'T (state 1)', 'FontSize',10, ...
     'Color',col.arm1, 'HorizontalAlignment','right');
text(ax, T2(1)+0.10, T2(2)-0.12, 'state 2', 'FontSize',10, ...
     'Color',col.arm2);
text(ax, (P(1)+S(1))/2-0.18, (P(2)+S(2))/2+0.05, 'b', 'FontSize',11, ...
     'FontAngle','italic', 'Color',col.frame);
text(ax, (P(1)+T1(1))/2, (P(2)+T1(2))/2-0.18, 'R', 'FontSize',11, ...
     'FontAngle','italic', 'Color',col.arm1);
text(ax, (S(1)+T1(1))/2, (S(2)+T1(2))/2+0.20, 'c(\gamma)', ...
     'FontSize',11, 'FontAngle','italic', 'Color',col.sprng);

% gamma arc
ga = linspace(phi-gamma, phi, 60);
plot(ax, 0.42*cosd(ga), 0.42*sind(ga), '-', 'Color',col.frame, ...
     'LineWidth',0.9);
gm = phi - gamma/2;
text(ax, 0.55*cosd(gm), 0.55*sind(gm), '\gamma', 'FontSize',12, ...
     'Color',col.ink, 'HorizontalAlignment','center');

axis(ax, 'equal'); axis(ax, 'off');
text(ax, 0.5, -0.03, sprintf(['Von Mises toggle — \\gamma = %g%c,  ' ...
     'span c = %.4f'], gamma, 176, c), 'Units','normalized', ...
     'HorizontalAlignment','center', 'FontSize',11, 'Color',col.ink);

%% Export
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
outd = fullfile(here, 'output');
if ~exist(outd, 'dir'), mkdir(outd); end
exportgraphics(fig, fullfile(outd, 'toggle_model_sheet.pdf'), ...
               'ContentType','vector', 'BackgroundColor',bg);
exportgraphics(fig, fullfile(outd, 'toggle_model_sheet.png'), ...
               'Resolution',300, 'BackgroundColor',bg);
fprintf('exported toggle_model_sheet.pdf / .png\n');

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
