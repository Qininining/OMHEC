# Paper experiments

Optional publication simulations and offline analysis. Core algorithms remain in `MATLAB/src/+mhec`.

From the project root in MATLAB:

```matlab
cd paper_experiments/simulation
run_simulation(true)                  % Quick check
% run_simulation(false)               % Full configuration
% run_simulation(false, 'joint')      % Joint sensitivity only
```

Publication Fig. 3 requires pdflatex, standalone and graphicx. The simple MATLAB example has no LaTeX dependency.

For the recorded 5X/10X/20X experiments, start from the project root:

```matlab
cd paper_experiments/physical_experiments
run_analysis
```

Each workflow writes its own `output/`. The 159 raw CSVs are unchanged; physical data use nm internally, while reported lateral error uses micrometres.
