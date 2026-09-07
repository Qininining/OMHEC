# Paper simulation

This directory is self-contained and generates the simulation data and
manuscript Figs. 2--5.

From MATLAB, run:

```matlab
cd('<repository>/Paper_Simulation');
run_simulation(false);   % 100 Monte Carlo trials used by the paper
```

Use `run_simulation(true)` only for a fast pipeline check. Outputs are written
to `output/data` and `output/figures`.

## Code layout

- `run_simulation.m`: public entry point;
- `configure_simulation.m`: manuscript parameters and figure style;
- `generate_simulation_data.m`: Monte Carlo experiments and estimators;
- `solve_position_only_ls.m`: PO-LS baseline solver;
- `generate_simulation_figures.m`: Figs. 2--5.
