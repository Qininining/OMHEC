# Offline physical-experiment analysis

This directory reproduces Figs. 7-8 and Table I from recorded measurements; no camera, motion controller, or Qt installation is required.

## Run

In this directory, run:

```matlab
run_analysis
```

The function locates `analysis/` and `data/` from its own location and creates `output/data/`, `output/figures/`, and `output/tables/`. Existing generated outputs are replaced. MATLAB R2025b Update 4 was used for validation.

## Recorded data

`data/5X`, `data/10X`, and `data/20X` contain the original experiment CSV files copied unchanged from the accompanying manuscript's data collection. All 159 input files are retained.

The analysis includes every complete paired round. Historical file tags are interpreted as follows:

| CSV tag | Manuscript method |
| --- | --- |
| `PO-LS` | PO-LS |
| `J-FGD` | PR-FGD |
| `J-RIEKF` | PR-IEKF |

The CSV reader and configuration functions define the input columns and conversions. Physical positions and static-noise standard deviations use **nanometres** internally; lateral validation errors are reported in **micrometres**. The covariance-weighted residual metric is dimensionless. Simulation length units are different (millimetres).

## Outputs

- `figures/fig_exp_5x_boxplot.pdf` and PNG: repeated-calibration distribution at 5X (Fig. 7).
- `figures/fig_exp_magnification_comparison.pdf` and PNG: comparison of 5X, 10X, and 20X (Fig. 8).
- `tables/physical_magnification_comparison_table.tex`: experiment details and error statistics (Table I).
- `data/physical_*_round_results.csv`, `physical_*_statistics.csv`, `physical_*_observation_covariance_nm2.csv`, and `physical_magnification_conditions.csv`: intermediate and aggregate statistics.

The raw observations remain separate from generated results. Reproduction was checked against a fresh execution of the original manuscript analysis: all 11 generated CSV files and the LaTeX table matched exactly.
