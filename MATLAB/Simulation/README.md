# MATLAB simulations

This directory reproduces Figs. 2-5 of the accompanying manuscript using PO-LS, PR-FGD, and PR-IEKF. It includes the current joint initialization/excitation study for Fig. 3.

## Run

In this directory:

```matlab
run_simulation(true)                 % Smoke test
% Alternatively, reproduce the full manuscript configuration:
run_simulation(false)
```

`run_simulation(quickMode, scope)` keeps the default arguments `false` and `'all'`. Supported scopes are `'all'`, `'joint'`, and `'anisotropy'`. Joint mode runs only Fig. 3; anisotropy mode requires saved results from a compatible complete run. All paths are relative to the function location, rather than the caller's working directory.

`configure_simulation.m` contains the fixed seed, initialization policy, excitation, noise, and trial counts. Lengths are in **millimetres** and angles are in **radians**, except fields explicitly marked in degrees. Numerical methods and parameters match the current manuscript sources.

Quick mode uses 4 trials and 35 steps, with reduced initialization/noise sweep grids. Full mode uses 100 trials and 100 steps. Both configurations use the same output names; run full mode again after smoke testing when manuscript results are needed.

## Outputs

- `output/data/paper_simulation_results.mat`: standard convergence and parameter sweeps.
- `output/data/joint_sensitivity/study_*.mat`: the four initialization/excitation studies.
- `output/data/simulation_summary.csv`: numerical summaries.
- `output/figures/fig_sim_*.pdf` and associated PNG/FIG files: paper figures.

To redraw figures from existing matching results without rerunning the simulations:

```matlab
cfg = configure_simulation(fileparts(which('run_simulation')), false);
generate_simulation_figures(cfg);
```

Use `true` instead of `false` if the saved results came from quick mode. Fig. 3 exports an exact-size PNG through a generated standalone LaTeX wrapper, so `pdflatex`, `standalone`, and `graphicx` must be installed. The other figures are exported by MATLAB.

The functions use base MATLAB operations. The workflow was validated with MATLAB R2025b Update 4 and TeX Live 2025. Generated files are ignored by Git.
