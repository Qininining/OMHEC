function generate_simulation_figures(cfg, scope)
%GENERATE_SIMULATION_FIGURES Export manuscript Figs. 2--5.
if nargin < 2, scope = 'all'; end
assert(ismember(scope, {'all','anisotropy'}), 'Unknown figure scope: %s', scope);

resultFile = fullfile(cfg.dataDir, 'paper_simulation_results.mat');
if ~isfile(resultFile)
    error('Run generate_simulation_data first: %s', resultFile);
end
data = load(resultFile, 'cfg', 'convergence', 'initialSensitivity', ...
    'anisotropy', 'noiseMagnitude');
assert(isequal(cfg.kappaLevels, data.cfg.kappaLevels), ...
    'Anisotropy levels differ from saved data. Rerun the sweep first.');
% Single-column figures use two vertically stacked panels.  The height leaves
% each axes legible at IEEE column width while reserving space for the panel
% identifiers above the plotting rectangles.
panelFigureHeightCm = cfg.figureHeightCm;

if strcmp(scope, 'all')
fig = makeFigure(cfg, panelFigureHeightCm, cfg.figureWidthCm);
layout = tiledlayout(fig, 2, 1, 'Padding', 'compact', ...
    'TileSpacing', 'compact');
ax1 = nexttile(layout);
handles = plotConvergence(ax1, data.convergence.rotation, cfg);
xlabel(ax1, 'Iteration');
ylabel(ax1, '$e_{R,k}$ (deg)', 'Interpreter', 'latex');
panelLabel(ax1, '(a)');
ax2 = nexttile(layout);
plotConvergence(ax2, data.convergence.translation, cfg);
xlabel(ax2, 'Iteration');
ylabel(ax2, '$e_{t,k}$ (mm)', 'Interpreter', 'latex');
panelLabel(ax2, '(b)');
addMethodLegend(ax1, handles, cfg);
finishFigure(fig, cfg, 'fig_sim_convergence');
generate_joint_sensitivity_figure(cfg);
end

% The three parameter sweeps need less vertical space than the 100-step
% convergence history.
compactPanelFigureHeightCm = cfg.compactFigureHeightCm;

fig = makeFigure(cfg, compactPanelFigureHeightCm, cfg.figureWidthCm);
layout = tiledlayout(fig, 2, 1, 'Padding', 'compact', ...
    'TileSpacing', 'compact');
ax1 = nexttile(layout);
plotSweep(ax1, cfg.kappaLevels, data.anisotropy.rotation, cfg, true);
set(ax1, 'XScale', 'log', 'XMinorTick', 'off', 'XMinorGrid', 'off');
xlabel(ax1, 'Anisotropy ratio, \kappa');
ylabel(ax1, '$e_R$ (deg)', 'Interpreter', 'latex');
panelLabel(ax1, '(a)');
ax2 = nexttile(layout);
plotSweep(ax2, cfg.kappaLevels, data.anisotropy.translation, cfg, true);
set(ax2, 'XScale', 'log', 'XMinorTick', 'off', 'XMinorGrid', 'off');
xlabel(ax2, 'Anisotropy ratio, \kappa');
ylabel(ax2, '$e_t$ (mm)', 'Interpreter', 'latex');
panelLabel(ax2, '(b)');
addSweepLegend(ax1, cfg);
finishFigure(fig, cfg, 'fig_sim_anisotropic_noise');
if strcmp(scope, 'anisotropy'), return; end

fig = makeFigure(cfg, compactPanelFigureHeightCm, cfg.figureWidthCm);
layout = tiledlayout(fig, 2, 1, 'Padding', 'compact', ...
    'TileSpacing', 'compact');
noiseXYMicrometres = 1000 * cfg.sigmaXY * cfg.noiseScaleLevels;
ax1 = nexttile(layout);
plotSweep(ax1, noiseXYMicrometres, data.noiseMagnitude.rotation, cfg, true);
set(ax1, 'XScale', 'log', 'XMinorTick', 'off', 'XMinorGrid', 'off');
xlabel(ax1, 'In-plane noise std., \sigma_{xy} (\mum)');
ylabel(ax1, '$e_R$ (deg)', 'Interpreter', 'latex');
panelLabel(ax1, '(a)');
ax2 = nexttile(layout);
plotSweep(ax2, noiseXYMicrometres, data.noiseMagnitude.translation, cfg, true);
set(ax2, 'XScale', 'log', 'XMinorTick', 'off', 'XMinorGrid', 'off');
xlabel(ax2, 'In-plane noise std., \sigma_{xy} (\mum)');
ylabel(ax2, '$e_t$ (mm)', 'Interpreter', 'latex');
panelLabel(ax2, '(b)');
addSweepLegend(ax1, cfg);
finishFigure(fig, cfg, 'fig_sim_noise_magnitude');
end

function handles = plotConvergence(ax, values, cfg)
hold(ax, 'on');
% Display convergence only after the common full-state initialization.
startIndex = min(cfg.observabilityInitializationSteps + 1, size(values, 1));
values = values(startIndex:end, :, :);
% The estimate obtained from the six-motion initialization is labelled as
% iteration 6; the pre-initialization samples are not displayed.
iterations = cfg.observabilityInitializationSteps: ...
    (cfg.observabilityInitializationSteps + size(values, 1) - 1);
handles = gobjects(3, 1);
for method = 1:3
    samples = values(:, :, method);
    center = mean(samples, 2, 'omitnan');
    spread = std(samples, 0, 2, 'omitnan');
    % Keep the one-standard-deviation band strictly positive so it remains
    % valid on the logarithmic ordinate used by the convergence figure.
    lower = max(center - spread, max(center * 1e-3, eps));
    upper = center + spread;
    fill(ax, [iterations, fliplr(iterations)], ...
        [lower', fliplr(upper')], cfg.methodColors(method, :), ...
        'FaceAlpha', 0.10, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
    handles(method) = plot(ax, iterations, center, ...
        cfg.lineStyles{method}, 'Color', cfg.methodLineColors(method, :), ...
        'LineWidth', cfg.lineWidth);
end
xlim(ax, [iterations(1), iterations(end)]);
xticks(ax, unique([iterations(1), 20:20:iterations(end)]));
styleAxes(ax, cfg);
set(ax, 'YScale', 'log');
ylim(ax, 'auto');
end

function handles = plotSweep(ax, x, values, cfg, logY)
if nargin < 5
    logY = false;
end
hold(ax, 'on');
handles = gobjects(3, 1);
upperEndpointMax = 0;
for method = 1:3
    samples = values(:, :, method);
    center = mean(samples, 2, 'omitnan');
    spread = std(samples, 0, 2, 'omitnan');
    upperEndpointMax = max(upperEndpointMax, max(center + spread, [], 'omitnan'));
    if logY
        % Logarithmic ordinates cannot display zero or negative whisker
        % endpoints. Only the displayed lower endpoint is truncated; the
        % measured center and standard deviation remain unchanged.
        lowerEndpoint = max(center - spread, center * 1e-3);
        lowerError = center - lowerEndpoint;
    else
        lowerError = min(spread, center);
    end
    handles(method) = errorbar(ax, x, center, lowerError, spread, ...
        cfg.lineStyles{method}, 'Color', cfg.methodLineColors(method, :), ...
        'LineWidth', cfg.lineWidth, 'Marker', 'o', 'MarkerSize', 3, ...
        'CapSize', 2.5);
end
xticks(ax, x);
xlim(ax, [min(x), max(x)]);
styleAxes(ax, cfg);
if logY
    set(ax, 'YScale', 'log');
    ylim(ax, 'auto');
else
    % Include every mean-plus-standard-deviation endpoint, with headroom
    % for the whisker caps. Automatic limits can crop the Fig. 3 error bars.
    ylim(ax, [0, max(1.12 * upperEndpointMax, eps)]);
end
end

function fig = makeFigure(cfg, heightCm, widthCm)
if nargin < 3
    widthCm = cfg.figureWidthCm;
end
fig = figure('Color', 'w', 'Units', 'centimeters', ...
    'Position', [2, 2, widthCm, heightCm], ...
    'PaperPositionMode', 'auto', 'Visible', 'off');
end

function styleAxes(ax, cfg)
set(ax, 'FontName', cfg.fontName, 'FontSize', cfg.fontSize, ...
    'LabelFontSizeMultiplier', 1, 'LineWidth', 0.7, ...
    'TickDir', 'in', 'Box', 'on');
grid(ax, 'on');
ax.GridAlpha = 0.12;
ax.GridLineStyle = '--';
end

function panelLabel(ax, label)
% Place every panel identifier immediately outside the upper-left corner.
% Its left edge is aligned exactly with the left border of the axes.
text(ax, 0, 1.005, label, 'Units', 'normalized', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
    'Clipping', 'off', 'FontWeight', 'normal', 'FontName', 'Times New Roman', 'Interpreter', 'none');
end

function addMethodLegend(ax, handles, cfg)
lgd = legend(ax, handles, cfg.methodLabels, ...
    'Orientation', 'horizontal', 'Location', 'north', ...
    'Box', 'off', 'NumColumns', 3);
% A single compact legend inside the first panel serves both subplots and
% does not consume additional space outside the plotting area.
set(lgd, 'FontName', cfg.fontName, 'FontSize', 8.5, ...
    'ItemTokenSize', [18, 7]);
end

function addSweepLegend(ax, cfg)
% Match Fig. 8 with fixed line-and-circle samples. Errorbar legend glyphs
% depend on axis scale, so they differ between linear and logarithmic plots.
legendHandles = gobjects(3, 1);
xLimits = xlim(ax);
yLimits = ylim(ax);
legendX = xLimits(2) * [10, 11];
legendY = yLimits(2) * [10, 10];
for method = 1:3
    legendHandles(method) = plot(ax, legendX, legendY, ...
        cfg.lineStyles{method}, ...
        'Color', cfg.methodLineColors(method, :), ...
        'LineWidth', cfg.lineWidth, 'Marker', 'o', 'MarkerSize', 4, ...
        'MarkerFaceColor', cfg.methodColors(method, :), ...
        'MarkerEdgeColor', cfg.methodColors(method, :));
end
xlim(ax, xLimits);
ylim(ax, yLimits);
lgd = legend(ax, legendHandles, cfg.methodLabels, ...
    'Orientation', 'vertical', 'Location', 'northwest', ...
    'Box', 'off', 'NumColumns', 1);
set(lgd, 'FontName', cfg.fontName, 'FontSize', 8.5, ...
    'ItemTokenSize', [18, 7]);
% Reduce the top inset to keep the bottom legend row clear of error bars.
drawnow;
lgd.Units = 'normalized';
ax.Units = 'normalized';
legendPosition = lgd.Position;
legendPosition(2) = ax.Position(2) + ax.Position(4) - ...
    legendPosition(4) - 0.005;
lgd.Position = legendPosition;
end

function finishFigure(fig, cfg, baseName)
pngPath = fullfile(cfg.figureDir, [baseName, '.png']);
pdfPath = fullfile(cfg.figureDir, [baseName, '.pdf']);
exportgraphics(fig, pngPath, 'Resolution', cfg.resolution);
exportgraphics(fig, pdfPath, 'ContentType', 'vector');
close(fig);
end
