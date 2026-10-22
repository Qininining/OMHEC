function cfg = configure_physical_analysis(analysisDir)
%CONFIGURE_PHYSICAL_ANALYSIS Define data paths and manuscript figure style.

cfg.rootDir = fileparts(analysisDir);
cfg.outputDir = fullfile(cfg.rootDir, 'output');
cfg.dataDir = fullfile(cfg.outputDir, 'data');
cfg.figureDir = fullfile(cfg.outputDir, 'figures');
cfg.tableDir = fullfile(cfg.outputDir, 'tables');
cfg.physicalDataRoot = fullfile(cfg.rootDir, 'data', '5X');
cfg.physicalDataRoot10X = fullfile(cfg.rootDir, 'data', '10X');
cfg.physicalDataRoot20X = fullfile(cfg.rootDir, 'data', '20X');
cfg.analysisDir = analysisDir;

% Raw filenames keep the historical tags; figures and tables use the
% terminology defined in the manuscript.
cfg.methodFileTags = {'PO-LS', 'J-FGD', 'J-RIEKF'};
cfg.methodLabels = {'PO-LS', 'PR-FGD', 'PR-IEKF'};
% Unified palette used by every paper figure: blue (PO-LS), amber
% (PR-FGD), and vermilion (PR-IEKF).
cfg.methodColors = [0.0000, 0.4470, 0.6980; ...
                    0.9300, 0.5600, 0.0800; ...
                    0.8500, 0.1800, 0.1000];
% MATLAB's vector exporters do not reliably preserve line alpha. Blending
% toward the white background gives the same restrained visual weight.
cfg.lineOpacity = 0.82;
cfg.methodLineColors = cfg.lineOpacity * cfg.methodColors + ...
    (1 - cfg.lineOpacity) * ones(size(cfg.methodColors));
cfg.lineStyles = {'--', '-.', '-'};

cfg.doubleFigureWidthCm = 17.8;
% Figs. 7--8 use the IEEE single-column width.
cfg.physicalFigureWidthCm = 8.8;
cfg.fontName = 'Times New Roman';
% IEEEtran body text is 10 pt; all text embedded in figures is larger.
cfg.fontSize = 11;
cfg.lineWidth = 1.55;
cfg.resolution = 600;

end
