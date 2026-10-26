# Optional paper reproduction

These programs are an appendix to the reusable algorithms in `C++/src` and `MATLAB/src`. They retain the publication sweeps, figure layouts, recorded physical data and table exports. They are not needed for the method examples.

From the OMHEC root in MATLAB:

```matlab
cd reproduction/simulation
run_simulation(true)                  % Reduced check, including Fig. 3
% run_simulation(false)               % Full manuscript configuration
% run_simulation(false, 'joint')      % Fig. 3 only
% run_simulation(false, 'anisotropy') % Requires matching prior full results
```

The entry point adds `MATLAB/src` using a path relative to its own file. Standard and joint studies call the same `omhec` updates and PO-LS solver. `output/` is local to this appendix. Full runs use the original seed/configuration; smoke-test results do not replace the manuscript's reported statistics.

Publication Fig. 3 still needs `pdflatex` on PATH with `standalone`/`graphicx`. The core and simple demonstration do not have that dependency. Original figure helpers, typography and parameter grids remain only in this appendix.

For physical statistics, start at the OMHEC root:

```matlab
cd reproduction/physical_experiments
run_analysis
```

This reads the unchanged 159 CSV files in the bundled 5X, 10X and 20X folders, without hardware. Results are written to its own `output/`: 11 CSV statistics files, two figure pairs, and a LaTeX table. Physical recordings use nanometres internally; published lateral error is in micrometres. Historical J-FGD/J-RIEKF tags map to PR-FGD/PR-IEKF.
