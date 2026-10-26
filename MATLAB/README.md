# MATLAB interfaces

Add `MATLAB/src` to the path. Core functions do not plot, load data or require ground truth.

| Function | Purpose |
| --- | --- |
| `mhec.initialize(Tcs0,cfg,Sigma)` | Initialize estimate, covariance and observation buffers |
| `mhec.update(state,xiStage,p0,p1,method)` | One update; method is `'iekf'` or `'fgd'` |
| `mhec.solve_position_only_ls(poses,points)` | Batch PO-LS, returning Tsc |
| `mhec.default_config()` | Default initialization, gain and process-noise parameters |

Tcs maps stage to camera; Tsc is its inverse. Inputs: 4x4 Tcs, 6x1 stage-frame motion increment [rotation; translation], and 3x1 camera positions. Batch inputs are 4x4xN tool-to-stage poses and 3xN points. Estimated covariance is `state.P`.

Defaults and examples use mm/rad. Other units require matching covariance/prior and translation scales. Six motions are collected before correction by default. Use separate states for different methods.

Run `demo_calibration` from `examples/` for a fixed-seed comparison and two ordinary error plots; pass `false` to disable plots. Run `test_core` from `tests/` for numerical regression checks. Neither workflow requires LaTeX.
