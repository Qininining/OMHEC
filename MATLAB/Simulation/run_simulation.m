function run_simulation(quickMode, scope)
%RUN_SIMULATION Reproduce the simulation data and manuscript Figs. 2--5.
% RUN_SIMULATION(false) uses the manuscript configuration; true selects a
% reduced smoke test of the same pipeline.
if nargin < 1
    quickMode = false;
end
if nargin < 2, scope = 'all'; end
assert(ismember(scope, {'all','joint','anisotropy'}), 'Scope must be all, joint or anisotropy');
rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);
ensureDirectory(fullfile(rootDir, 'output', 'data'));
ensureDirectory(fullfile(rootDir, 'output', 'figures'));
cfg = configure_simulation(rootDir, quickMode);
generate_simulation_data(cfg, scope);
if strcmp(scope, 'joint')
    generate_joint_sensitivity_figure(cfg);
else
    generate_simulation_figures(cfg, scope);
end
fprintf('Simulation outputs: %s\n', cfg.outputDir);
end

function ensureDirectory(pathValue)
if ~isfolder(pathValue)
    mkdir(pathValue);
end
end
