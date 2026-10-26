# Paper simulation appendix

Use `run_simulation(quickMode, scope)` here. The entry point finds the shared `MATLAB/src/+mhec` package relative to this directory. Algorithm updates are not implemented as local copies in these scripts.

`quickMode=false` selects the manuscript configuration; `true` reduces trials and steps. Scopes are `all`, `joint`, and `anisotropy`. Anisotropy-only mode requires saved results with matching settings. Generated data, CSV summaries and figures are placed in `output/` and ignored by Git.

This directory intentionally retains paper plotting, LaTeX Fig. 3 export, joint sensitivity grids and figure formatting. Use `MATLAB/examples/demo_calibration.m` for a simple algorithm demonstration without these dependencies. See the parent reproduction README for commands and prerequisites.
