function cfg = configure_simulation(rootDir, quickMode)
%CONFIGURE_SIMULATION Define the deterministic Section IV experiments.
% Lengths are in millimetres and angles are in radians unless stated.

if nargin < 2
    quickMode = false;
end

cfg.outputDir = fullfile(rootDir, 'output');
cfg.dataDir = fullfile(cfg.outputDir, 'data');
cfg.figureDir = fullfile(cfg.outputDir, 'figures');
cfg.p0Policy = 'correlated_fixed_scales_30deg_0p8mm';
cfg.seed = 20260818;
cfg.numTrials = 100;
cfg.numConvergenceTrials = 100;
cfg.numSteps = 100;
cfg.finalWindow = 10;
cfg.decayGain = 0.03;
cfg.observabilityInitializationSteps = 6;
% Fixed relative cutoff for the truncated-SVD pseudoinverse used by the
% recursive single-observation PR-FGD update. This value is shared by every
% simulation condition and by the physical implementation.
cfg.jfgdSvdRelativeThreshold = 0.05;

% Length unit: millimetres; angle unit: radians.
cfg.sigmaXY = 0.003;
cfg.nominalKappa = 10;
cfg.processRotationStd = 0;
cfg.processTranslationStd = 0;
cfg.initialRotationDeg = 10;
cfg.initialTranslation = 1.00;
cfg.rotationStep = deg2rad(1.0);
cfg.translationStep = 0.10;
% Fixed characteristic length used only by PR-FGD to make its
% rotational and translational error coordinates dimensionless.  It is
% determined a priori from the common excitation step sizes, not tuned
% from the calibration results.
cfg.decayCharacteristicLength = cfg.translationStep / cfg.rotationStep;

cfg.rotationLevelsDeg = [1, 5, 10, 15, 20, 30, 40];
cfg.translationLevels = [0.05, 0.20, 0.50, 0.80, 1.20, 1.60, 2.00];
cfg.kappaLevels = 2.^(2:7);
cfg.noiseScaleLevels = [0.25, 0.5, 1, 2, 4, 8];

% All experiments use the selected correlated prior; these grids are for Fig. 3.
cfg.jointRotationStepsDeg = [0.25 0.5 1 2 4];
cfg.jointTranslationSteps = [0.025 0.05 0.1 0.2 0.4];

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

cfg.figureWidthCm = 8.8;
cfg.figureHeightCm = 9.0;
% Parameter-sweep figures contain only a few operating points per panel and
% therefore use a shorter canvas than the convergence history.
cfg.compactFigureHeightCm = 8.4;
cfg.fontName = 'Times New Roman';
% IEEEtran body text is 10 pt; all text embedded in figures is larger.
cfg.fontSize = 11;
cfg.lineWidth = 1.55;
cfg.resolution = 600;

if logical(quickMode)
    cfg.numTrials = 4;
    cfg.numConvergenceTrials = 4;
    cfg.numSteps = 35;
    cfg.finalWindow = 5;
    cfg.rotationLevelsDeg = [1, 10, 30];
    cfg.translationLevels = [0.05, 0.50, 1.60];
    cfg.kappaLevels = [4, 16, 128];
    cfg.noiseScaleLevels = [0.25, 1, 8];
end
end
