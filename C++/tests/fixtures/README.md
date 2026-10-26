# Frozen pre-extraction C++ fixture

`before_extraction.txt` records transforms and covariance matrices from the original Qt hardware estimator implementation, compiled separately before extraction. Only the test driver was added; the original estimator and math source files were unchanged.

The fixture uses the example's fixed-seed dataset (mt19937 with explicit Box-Muller noise), six-motion initialization, a 0.03 fixed gain, a 0.05 relative SVD cutoff, disabled innovation gating, initial P=0.01 I, Q=1e-8 I and residual R=1e-6 I. For each of three methods, each of 36 steps stores Tsc (16 row-major values) followed by P (36 row-major values).

`regression_scenario.hpp` defines this input protocol. `test_core` compares against the frozen output with 1e-9 absolute tolerance. The geometry/Jacobian and facade checks are separate. Do not regenerate this file using the refactored implementation to conceal a numerical change.
