# MATLAB core and examples

Add `src/` to your MATLAB path to use the package `omhec`. Add `examples/` only when you want synthetic data and a demonstration. Core functions neither plot nor write files and do not need ground truth.

## Public operations

| Function | Purpose |
| --- | --- |
| `omhec.default_config` | Six-motion initialization, PR-FGD gain/scale and process-noise defaults |
| `omhec.initialize(Tcs0,cfg,Sigma)` | Initialize estimate, left-error covariance and observation buffers |
| `omhec.update(state,xiStage,p0,p1,method)` | One PR-IEKF (`'iekf'`) or PR-FGD (`'fgd'`) update |
| `omhec.solve_position_only_ls(poses,points)` | Batch PO-LS, returning Tsc |
| `omhec.residual_model` | Shared residual, first-order Jacobian and propagated covariance |
| `omhec.expSE3`, `omhec.adjointSE3`, `omhec.inverseTransform` | Shared transform operations |

Tcs maps stage to camera. Twists are 6x1 **stage-frame motion increments**, ordered [rotation; translation]; observations are 3x1 camera-frame positions. The examples use mm/rad, and the default covariance/prior scales are specified for millimetres. Batch PO-LS uses 4x4xN poses and 3xN points and returns the opposite-direction Tsc. Poses map tool coordinates into the stage/base frame.

The state contains `Tcs`, `P`, `Q`, configuration, point covariance, update count and stacked initialization data. The default P is the original correlated prior in left-error coordinates. If using another length unit, supply a correspondingly transformed `state.P`, process-noise parameters and PR-FGD length scale. Use distinct state instances for each method. `info.initialized` becomes true at the configured initialization count; before that estimate updates are withheld.

## Example

```matlab
addpath('examples');
results = demo_calibration();
```

From another directory, add the appropriate absolute example path. The demo creates synthetic observations, calls all three methods, reports final transforms and errors, and displays one figure with ordinary rotation/translation error curves. Set its argument to `false` for no graphics. Neither LaTeX nor paper-specific plotting functions are used.

`generate_dataset` creates observations and ground truth; `run_methods` owns evaluation and histories. To process your own data, call the core functions directly as shown in the root README instead of editing an estimator to load a file.

Run `test_core` from `tests/` (or add that directory to the path) for baseline and geometry checks. Paper reproduction lives separately in `OMHEC/reproduction/`.
