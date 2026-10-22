function run_analysis()
%RUN_ANALYSIS Reproduce the physical statistics, Figs. 7--8, and Table I.
rootDir = fileparts(mfilename('fullpath'));
analysisDir = fullfile(rootDir, 'analysis');
addpath(analysisDir);
ensureDirectory(fullfile(rootDir, 'output', 'data'));
ensureDirectory(fullfile(rootDir, 'output', 'figures'));
ensureDirectory(fullfile(rootDir, 'output', 'tables'));
cfg = configure_physical_analysis(analysisDir);
analyze_physical_experiments(cfg);
fprintf('Physical-analysis outputs: %s\n', cfg.outputDir);
end

function ensureDirectory(pathValue)
if ~isfolder(pathValue)
    mkdir(pathValue);
end
end
