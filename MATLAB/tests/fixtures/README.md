# Frozen pre-extraction MATLAB fixture

`before_extraction.mat` contains outputs from the original `simulate_joint_trial` implementation before its algorithm code was extracted. The only instrumentation added transform and covariance histories; update formulas were not changed.

Input: original quick configuration (35 steps), seed 1234, 10-degree initial rotation perturbation and 1 mm translation perturbation. All three methods share observations. Configuration fields pointing to local output folders are omitted from this fixture.

`test_core` recreates the inputs and compares transforms, error histories and recursive covariance histories. The first covariance slot is intentionally undefined (NaN); tests compare slots 2 onward. Do not regenerate this fixture using the new core to make a failing regression pass.
