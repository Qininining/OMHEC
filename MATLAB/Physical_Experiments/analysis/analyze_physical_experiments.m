function analyze_physical_experiments(cfg, figuresOnly)
%ANALYZE_PHYSICAL_EXPERIMENTS Reproduce Figs. 7--8 and Table I.
% Every complete paired round in each data directory is included.
% Redraw Fig. 8 from saved statistics without rewriting data or the table.
if nargin > 1 && figuresOnly
    statistics = readtable(fullfile(cfg.dataDir, ...
        'physical_magnification_statistics.csv'), 'TextType', 'string');
    makeMagnificationComparison(statistics, cfg);
    return;
end

magnifications = {'5X', '10X', '20X'};
combinedStatistics = table();
combinedConditions = table();
for index = 1:numel(magnifications)
    magnification = magnifications{index};
    tag = lower(magnification);
    [data, observationCovariance] = load_validation_results(cfg, magnification);
    condition = physicalOperatingCondition(cfg, magnification, ...
        observationCovariance, index);
    combinedConditions = [combinedConditions; condition]; %#ok<AGROW>
    % Every complete paired round present in the paper-data
    % directory is used; no result-dependent screening is performed here.
    writetable(data, fullfile(cfg.dataDir, ...
        sprintf('physical_%s_round_results.csv', tag)));
    statistics = physicalStatistics(data, cfg);
    statistics.Magnification = repmat(string(magnification), ...
        height(statistics), 1);
    combinedStatistics = [combinedStatistics; statistics]; %#ok<AGROW>
    writetable(statistics, fullfile(cfg.dataDir, ...
        sprintf('physical_%s_statistics.csv', tag)));
    writematrix(observationCovariance, fullfile(cfg.dataDir, ...
        sprintf('physical_%s_observation_covariance_nm2.csv', tag)));
    % Figure 7 reports the repeated-calibration distribution at 5X only.
    if strcmpi(magnification, '5X')
        makeBoxplot(data, cfg, magnification);
    end
end
writetable(combinedStatistics, fullfile(cfg.dataDir, ...
    'physical_magnification_statistics.csv'));
writetable(combinedConditions, fullfile(cfg.dataDir, ...
    'physical_magnification_conditions.csv'));
writeMagnificationLatexTable(combinedStatistics, combinedConditions, ...
    fullfile(cfg.tableDir, 'physical_magnification_comparison_table.tex'), cfg);
makeMagnificationComparison(combinedStatistics, cfg);
end

function condition = physicalOperatingCondition(cfg, magnification, ...
    observationCovariance, magnificationIndex)
% Recompute the optical scale and summarize the actual measurement settings.
% The static covariance is obtained from 50 stationary observations, each
% containing an independent autofocus cycle. The ordinary sample standard
% deviation of adjacent differences is divided by sqrt(2) to recover the
% standard deviation of one observation.
dataRoot = physicalConditionDataRoot(cfg, magnification);

scaleFiles = dir(fullfile(dataRoot, 'ScaleData_*.csv'));
if numel(scaleFiles) ~= 1
    error('Expected one %s scale file, found %d.', ...
        magnification, numel(scaleFiles));
end

scalePath = fullfile(scaleFiles(1).folder, scaleFiles(1).name);
scaleRecord = read_experiment_csv(string(scalePath));
scaleSection = scaleRecord.Sections.ALL_DATA;
pixels = scaleSection.Pc_curr(:, 1:2);
stagePositions = scaleSection.Ps_curr(:, 1:2);
pixelCentered = pixels - mean(pixels, 1);
stageCentered = stagePositions - mean(stagePositions, 1);
[U, singularValues, V] = svd(pixelCentered' * stageCentered);
inPlaneRotation = U * V';
scaleNmPerPixel = trace(singularValues) / sum(pixelCentered(:) .^ 2);
fitResidual = stageCentered - ...
    scaleNmPerPixel * pixelCentered * inPlaneRotation;

standardDeviation = sqrt(diag(observationCovariance))';
lateralStandardDeviation = sqrt(mean(standardDeviation(1:2) .^ 2));
calibrationTranslationUm = [300, 150, 75];
calibrationRotationDeg = [2, 1, 0.5];
validationTranslationUm = [450, 225, 112.5];
validationRotationDeg = [3, 1.5, 0.75];
if scaleNmPerPixel >= 500
    autofocusInitialStepUm = 15;
    autofocusRefinementStepUm = 3;
elseif scaleNmPerPixel >= 250
    autofocusInitialStepUm = 15;
    autofocusRefinementStepUm = 3;
elseif scaleNmPerPixel >= 125
    autofocusInitialStepUm = 4;
    autofocusRefinementStepUm = 0.75;
else
    autofocusInitialStepUm = 12;
    autofocusRefinementStepUm = 5;
end

condition = table(string(magnification), 2, scaleNmPerPixel, ...
    sqrt(mean(sum(fitResidual .^ 2, 2))), ...
    max(vecnorm(fitResidual, 2, 2)), ...
    2448 * scaleNmPerPixel / 1000, ...
    2048 * scaleNmPerPixel / 1000, ...
    standardDeviation(1), standardDeviation(2), standardDeviation(3), ...
    standardDeviation(3) / lateralStandardDeviation, ...
    autofocusInitialStepUm, autofocusRefinementStepUm, ...
    calibrationTranslationUm(magnificationIndex), ...
    calibrationRotationDeg(magnificationIndex), ...
    validationTranslationUm(magnificationIndex), ...
    validationRotationDeg(magnificationIndex), ...
    string(scaleFiles(1).name), ...
    'VariableNames', {'Magnification', 'RearMagnification', ...
    'ScaleNmPerPixel', 'ScaleFitRmseNm', 'ScaleFitMaxNm', ...
    'FieldWidthUm', 'FieldHeightUm', 'SigmaXNm', 'SigmaYNm', ...
    'SigmaZNm', 'AnisotropyRatio', 'AutofocusInitialStepUm', ...
    'AutofocusRefinementStepUm', 'CalibrationTranslationUm', ...
    'CalibrationRotationDeg', 'ValidationTranslationUm', ...
    'ValidationRotationDeg', 'ScaleSourceFile'});
end

function dataRoot = physicalConditionDataRoot(cfg, magnification)
switch upper(magnification)
    case '5X'
        dataRoot = cfg.physicalDataRoot;
    case '10X'
        dataRoot = cfg.physicalDataRoot10X;
    case '20X'
        dataRoot = cfg.physicalDataRoot20X;
    otherwise
        error('Unsupported magnification: %s.', magnification);
end
end

function makeMagnificationComparison(statistics, cfg)
fig = figure('Color', 'w', 'Units', 'centimeters', ...
    'Position', [2, 2, cfg.physicalFigureWidthCm, 9.3], ...
    'PaperPositionMode', 'auto', 'Visible', 'off');
tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

metricFields = {'MeanLateralUm', 'MeanNormalized'};
deviationFields = {'StdLateralUm', 'StdNormalized'};
yLabels = {'$e_{xy}\;(\mu\mathrm{m})$', '$e_{\Sigma}$'};
magnificationLabels = {'5\times', '10\times', '20\times'};

for panel = 1:2
    ax = nexttile;
    hold(ax, 'on');
    for methodIndex = 1:numel(cfg.methodLabels)
        means = nan(1, 3);
        deviations = nan(1, 3);
        for magnificationIndex = 1:3
            selected = statistics.Method == string(cfg.methodLabels{methodIndex}) & ...
                statistics.Magnification == string( ...
                strrep(magnificationLabels{magnificationIndex}, '\times', 'X'));
            means(magnificationIndex) = statistics.(metricFields{panel})(selected);
            deviations(magnificationIndex) = ...
                statistics.(deviationFields{panel})(selected);
        end
        errorbar(ax, 1:3, means, deviations, ...
            'Color', cfg.methodLineColors(methodIndex, :), ...
            'LineStyle', cfg.lineStyles{methodIndex}, ...
            'LineWidth', cfg.lineWidth, 'Marker', 'o', 'MarkerSize', 5.0, ...
            'MarkerFaceColor', cfg.methodColors(methodIndex, :), ...
            'MarkerEdgeColor', cfg.methodColors(methodIndex, :), ...
            'CapSize', 5);
    end
    xticks(ax, 1:3);
    xticklabels(ax, magnificationLabels);
    xlim(ax, [0.75, 3.25]);
    xlabel(ax, 'Objective magnification');
    ylabel(ax, yLabels{panel}, 'Interpreter', 'latex');
    % The three metrics span substantially different ranges across the
    % optical configurations.  A logarithmic ordinate on every panel keeps
    % multiplicative changes and uncertainty bars readable and consistent.
    set(ax, 'YScale', 'log');
    set(ax, 'FontName', cfg.fontName, 'FontSize', cfg.fontSize, ...
        'LabelFontSizeMultiplier', 1, 'LineWidth', 0.7, ...
        'TickDir', 'in', 'Box', 'on');
    grid(ax, 'on');
    ax.GridAlpha = 0.12;
    ax.GridLineStyle = '--';
    if panel == 1
        % Fixed line-and-circle samples match simulation Figs. 3--5,
        % independently of the scale-dependent errorbar legend rendering.
        xLimits = xlim(ax);
        yLimits = ylim(ax);
        legendHandles = gobjects(3, 1);
        for method = 1:3
            legendHandles(method) = plot(ax, xLimits(2) * [10, 11], ...
                yLimits(2) * [10, 10], cfg.lineStyles{method}, ...
                'Color', cfg.methodLineColors(method, :), ...
                'LineWidth', cfg.lineWidth, 'Marker', 'o', 'MarkerSize', 4, ...
                'MarkerFaceColor', cfg.methodColors(method, :), ...
                'MarkerEdgeColor', cfg.methodColors(method, :));
        end
        xlim(ax, xLimits);
        ylim(ax, yLimits);
        lgd = legend(ax, legendHandles, cfg.methodLabels, ...
            'Location', 'northeast', 'Orientation', 'vertical', 'NumColumns', 1, ...
            'FontName', cfg.fontName, 'FontSize', 8.5, 'Box', 'off');
        set(lgd, 'ItemTokenSize', [18, 7]);
    end
    panelLabel(ax, sprintf('(%c)', char('a' + panel - 1)));
end

exportgraphics(fig, fullfile(cfg.figureDir, ...
    'fig_exp_magnification_comparison.png'), 'Resolution', cfg.resolution);
exportgraphics(fig, fullfile(cfg.figureDir, ...
    'fig_exp_magnification_comparison.pdf'), 'ContentType', 'vector');
close(fig);
end

function writeMagnificationLatexTable(statistics, conditions, filePath, cfg)
fid = fopen(filePath, 'wt');
if fid < 0
    error('Cannot create LaTeX table: %s', filePath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%% Consolidated physical conditions and paired calibration results\n');
fprintf(fid, '\\begin{table*}[b]\n\\centering\n');
fprintf(fid, ['\\caption{Optical conditions, static observation noise, and ', ...
    'validation metrics under three objective magnifications.}\n']);
fprintf(fid, '\\label{tab:experimental_statistics_magnification}\n');
fprintf(fid, '\\footnotesize\n');
fprintf(fid, '\\renewcommand{\\arraystretch}{1.12}\n');
fprintf(fid, '\\setlength{\\tabcolsep}{2.5pt}\n');
fprintf(fid, ['\\begin{tabular*}{\\textwidth}{@{\\extracolsep{\\fill}}', ...
    'cccccccccc@{}}\n\\toprule\n']);
fprintf(fid, ['\\multirow{2}{*}{Objective} & \\multirow{2}{*}{Scale} & ', ...
    '\\multirow{2}{*}{Field of view} & ', ...
    '\\multicolumn{4}{c}{Static observation statistics} & ', ...
    '\\multirow{2}{*}{Method} & ', ...
    '\\multicolumn{2}{c}{Validation metrics} \\\\\n']);
fprintf(fid, ['\\cmidrule(lr){4-7} \\cmidrule(lr){9-10}\n', ...
    ' & & & $\\sigma_x$ & $\\sigma_y$ & $\\sigma_z$ & $\\kappa$ & ', ...
    ' & $e_{xy}$ & $e_{\\Sigma}$ \\\\\n']);
fprintf(fid, [' & (nm/pixel) & ($\\mu$m$\\times$$\\mu$m) & ', ...
    '(nm) & (nm) & (nm) & & & ($\\mu$m) & \\\\\n', ...
    '\\midrule\n']);
methodLabels = cfg.methodLabels;
for magnification = ["5X", "10X", "20X"]
    condition = conditions(conditions.Magnification == magnification, :);
    for methodIndex = 1:3
        selected = statistics.Magnification == magnification & ...
            statistics.Method == string(methodLabels{methodIndex});
        row = statistics(selected, :);
        if methodIndex == 1
            fprintf(fid, ['\\multirow{3}{*}{$%s$} & \\multirow{3}{*}{%.2f} & ', ...
                '\\multirow{3}{*}{%.0f$\\times$%.0f} & ', ...
                '\\multirow{3}{*}{%.1f} & \\multirow{3}{*}{%.1f} & ', ...
                '\\multirow{3}{*}{%.1f} & \\multirow{3}{*}{%.2f} & '], ...
                strrep(char(magnification), 'X', '\times'), condition.ScaleNmPerPixel, ...
                condition.FieldWidthUm, condition.FieldHeightUm, condition.SigmaXNm, ...
                condition.SigmaYNm, condition.SigmaZNm, condition.AnisotropyRatio);
        else
            fprintf(fid, ' & & & & & & & ');
        end
        fprintf(fid, '%s & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ \\\\\n', ...
            char(row.Method), row.MeanLateralUm, row.StdLateralUm, ...
            row.MeanNormalized, row.StdNormalized);
    end
    if magnification ~= "20X"
        % Separate objective groups by whitespace while preserving a strict
        % three-rule booktabs layout (top, header, and bottom rules only).
        fprintf(fid, '\\addlinespace[2pt]\n');
    end
end
fprintf(fid, '\\bottomrule\n\\end{tabular*}\n\\end{table*}\n');
end

function statistics = physicalStatistics(data, cfg)
rows = repmat(struct('Method', "", 'N', 0, 'MeanLateralUm', NaN, ...
    'StdLateralUm', NaN, 'MeanXUm', NaN, 'StdXUm', NaN, ...
    'MeanYUm', NaN, 'StdYUm', NaN, 'MeanZUm', NaN, 'StdZUm', NaN, ...
    'MeanNormalized', NaN, 'StdNormalized', NaN), 0, 1);
for method = 1:numel(cfg.methodLabels)
    selected = data.Method == string(cfg.methodLabels{method});
    normalized = data.NormalizedRmse(selected);
    row.Method = string(cfg.methodLabels{method});
    row.N = sum(selected);
    xValues = data.ResidualXRmseUm(selected);
    yValues = data.ResidualYRmseUm(selected);
    zValues = data.ResidualZRmseUm(selected);
    lateralValues = sqrt(xValues .^ 2 + yValues .^ 2);
    row.MeanLateralUm = mean(lateralValues);
    row.StdLateralUm = std(lateralValues);
    row.MeanXUm = mean(xValues);
    row.StdXUm = std(xValues);
    row.MeanYUm = mean(yValues);
    row.StdYUm = std(yValues);
    row.MeanZUm = mean(zValues);
    row.StdZUm = std(zValues);
    row.MeanNormalized = mean(normalized);
    row.StdNormalized = std(normalized);
    rows(end + 1) = row; %#ok<AGROW>
end
statistics = struct2table(rows);
end

function makeBoxplot(data, cfg, magnification)
fig = figure('Color', 'w', 'Units', 'centimeters', ...
    'Position', [2, 2, cfg.physicalFigureWidthCm, 9.0], ...
    'PaperPositionMode', 'auto', 'Visible', 'off');
tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile;
data.LateralRmseUm = sqrt(data.ResidualXRmseUm .^ 2 + ...
    data.ResidualYRmseUm .^ 2);
drawBoxes(ax1, data, 'LateralRmseUm', cfg);
ylabel(ax1, '$e_{xy}\;(\mu\mathrm{m})$', 'Interpreter', 'latex');
if strcmpi(magnification, '5X')
    ylim(ax1, [1.5, 4.0]);
    yticks(ax1, 1.5:0.5:4.0);
end
panelLabel(ax1, '(a)');

ax2 = nexttile;
drawBoxes(ax2, data, 'NormalizedRmse', cfg);
ylabel(ax2, '$e_{\Sigma}$', 'Interpreter', 'latex');
if strcmpi(magnification, '5X')
    % Keep the 5X ordinate linear, but derive its limits from the selected
    % data so renewed experiments cannot be clipped by legacy limits.
    set(ax2, 'YScale', 'linear');
    values = data.NormalizedRmse(isfinite(data.NormalizedRmse));
    span = max(max(values) - min(values), 1);
    lowerLimit = max(0, floor(min(values) - 0.08 * span));
    upperLimit = ceil(max(values) + 0.08 * span);
    ylim(ax2, [lowerLimit, upperLimit]);
elseif strcmpi(magnification, '10X')
    set(ax2, 'YScale', 'log');
    ylim(ax2, [5, 120]);
    yticks(ax2, [5, 10, 20, 50, 100]);
    yticklabels(ax2, {'5', '10', '20', '50', '100'});
elseif strcmpi(magnification, '20X')
    set(ax2, 'YScale', 'log');
    values = data.NormalizedRmse(isfinite(data.NormalizedRmse));
    lowerLimit = max(0.5, min(values) * 0.8);
    upperLimit = max(values) * 1.2;
    ylim(ax2, [lowerLimit, upperLimit]);
end
panelLabel(ax2, '(b)');

tag = lower(magnification);
exportgraphics(fig, fullfile(cfg.figureDir, ...
    sprintf('fig_exp_%s_boxplot.png', tag)), ...
    'Resolution', cfg.resolution);
exportgraphics(fig, fullfile(cfg.figureDir, ...
    sprintf('fig_exp_%s_boxplot.pdf', tag)), ...
    'ContentType', 'vector');
close(fig);
end

function panelLabel(ax, label)
% Match the simulation figures by placing every panel identifier immediately
% outside the upper-left corner, aligned with the left border of the axes.
text(ax, 0, 1.005, label, 'Units', 'normalized', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
    'Clipping', 'off', 'FontWeight', 'normal', 'FontName', 'Times New Roman', 'Interpreter', 'none');
end

function drawBoxes(ax, data, fieldName, cfg)
hold(ax, 'on');
for method = 1:numel(cfg.methodLabels)
    selected = data.Method == string(cfg.methodLabels{method});
    values = data.(fieldName)(selected);
    boxchart(ax, method * ones(size(values)), values, ...
        'BoxFaceColor', cfg.methodColors(method, :), ...
        'MarkerColor', cfg.methodColors(method, :), 'BoxWidth', 0.48);
    scatter(ax, method + linspace(-0.07, 0.07, numel(values))', values, 15, ...
        cfg.methodColors(method, :), 'filled', 'MarkerEdgeColor', 'w', ...
        'LineWidth', 0.35);
end
xticks(ax, 1:3);
xticklabels(ax, cfg.methodLabels);
xlim(ax, [0.55, 3.45]);
set(ax, 'FontName', cfg.fontName, 'FontSize', cfg.fontSize, ...
    'LabelFontSizeMultiplier', 1, 'LineWidth', 0.7, ...
    'TickDir', 'in', 'Box', 'on');
grid(ax, 'on');
ax.GridAlpha = 0.12;
ax.GridLineStyle = '--';
end
