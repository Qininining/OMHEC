# Core extraction validation

Validation date: 2026-10-26. Sources were tested from a separate standalone copy outside the MHEC working tree. No device was connected and no hardware application was launched.

## Environments

- Windows x64, MSVC 14.44.35207, Qt 6.2.3 for the optional hardware build.
- CMake/NMake, Eigen 3.4.0, OpenCV 4.10.0 for the core and console example.
- MATLAB R2025b Update 4; TeX Live 2025 only for the paper-reproduction appendix.

## Frozen numerical baselines

Before extracting algorithms, copies of the original MATLAB and C++ implementations were saved outside the project. The original MATLAB trial was instrumented to record estimates and covariances; its formulas were unchanged. The original C++ sources were compiled into a separate Qt-based baseline driver. Their outputs are retained in each language's tests/fixtures directory, with input/provenance notes.

- MATLAB tests compare all three transform/error trajectories on the original fixed-seed 35-step input. Recursive covariance traces match within 1e-8 absolute tolerance, excluding the deliberately undefined initial covariance history slot.
- C++ tests compare all three 36-step transform/covariance traces within 1e-9 absolute tolerance. Public streaming adapters are also checked against the extracted numerical kernels at every step.
- Both languages check SE(3) identities, the analytical instantaneous-motion Jacobian, initialization warmup and rank-deficient PO-LS inputs.
- The C++ and MATLAB regression fixtures refer to their respective original implementations. They do not assert that the two languages have identical default covariance or gating conventions.

## Core and example checks

- CMake core, test and console targets built without configuring Qt or hardware SDKs.
- CTest passed. The console demonstration completed for all three methods.
- Executable dependency inspection showed OpenCV and standard system/compiler runtimes, with no Qt dependency.
- From a clean MATLAB path, test_core and demo_calibration ran using only the standalone MATLAB source/example directories. The two ordinary result plots were exported and inspected. No LaTeX or publication plotting code was used by the demo.
- Static inspection found no GUI, random-data generation, plotting or file-output calls in the core directories.

## Optional workflows

- The migrated qmake hardware project successfully compiled/linked against the shared C++ core sources. AgeCOM.dll and PI_GCS2_DLL_x64.dll were deployed beside MHEC.exe.
- The independent paper simulation appendix completed run_simulation(true), including joint sensitivity and all four PDF outputs, using the shared MATLAB core.
- Offline physical analysis completed for all 159 recorded input CSVs. All 11 output CSV statistics files and the LaTeX table matched a fresh execution of the pre-migration analysis exactly.
- Original raw experiment data, manuscript PDF and setup image are preserved unchanged.

The full 100-trial manuscript Monte Carlo run was not repeated for this refactor. Live hardware behavior and vendor runtime deployment remain untested. Compilation may retain pre-existing unused-variable/unused-parameter warnings in the hardware and compatibility code.

## Naming update

Public package/namespace/target names are now `mhec`; the optional microscope project is `MHEC.pro`. Example and appendix directories are `examples/microscope_demo` and `paper_experiments`. MATLAB core regression and the simple example were rerun after the rename.
