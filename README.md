# Microscopic Hand-Eye Calibration in C++ and MATLAB

**MHEC** is a library for six-degree-of-freedom hand-eye calibration in micromanipulation. It estimates the transformation between the microscopic vision frame and robot base frame using 3-D positions of a single tracked feature and known robot motions, without requiring visual orientation measurements.

> Haoran Cui et al., "A 6-DOF Hand-Eye Calibration Method Based on Position Residuals for Micromanipulation."

## Motivation

- Demonstrate position-residual-based hand-eye calibration under anisotropic measurement noise.
- Share reusable implementations and examples accompanying the manuscript.

The C++ and MATLAB implementations provide the proposed position-residual-based invariant extended Kalman filter (**PR-IEKF**), position-only least squares (**PO-LS**), and position-residual fixed-gain descent (**PR-FGD**).

The C++ core uses **OpenCV and Eigen** and can be used independently of Qt. A separate [microscope demo](C++/examples/microscope_demo/README.md) provides the Qt acquisition and motion-control application. In MATLAB, `mhec.update` implements the recursive methods, while `mhec.solve_position_only_ls` provides the batch solver. Examples are separate from the core in both languages.

## Innovation

The method uses a position residual to estimate the complete hand-eye transformation from single-feature observations. An analytical Jacobian relates the residual to the transformation error and supports the analysis of motion excitation. PR-IEKF incorporates anisotropic position-measurement covariance and updates the estimate multiplicatively on SE(3), accounting for the lower axial localization precision of microscopic vision.

Full details are given in the [manuscript](docs/HandEyeCalibration.pdf). The [paper experiments](paper_experiments/README.md) contain the simulation studies and recorded-data analysis.

## Prerequisites

- **C++:** C++17, CMake 3.18+, OpenCV, and Eigen 3.4. The supplied Windows OpenCV libraries require MSVC x64 and the matching OpenCV 4.10 runtime.
- **MATLAB:** the core and examples were tested with R2025b.
- **Microscope demo only:** Qt with an MSVC x64 kit and the corresponding camera/motion SDKs.

Eigen headers and OpenCV SDK files are included in `C++/dependence/`. See the [C++](C++/README.md) and [MATLAB](MATLAB/README.md) instructions for interfaces, units, and configuration.

## Examples

From the repository root, run the MATLAB example:

```matlab
addpath('MATLAB/examples');
demo_calibration;
```

Build the C++ console example:

```sh
cmake -S C++ -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Run `build/demo_calibration` (Visual Studio: `build/Release/demo_calibration.exe`). Both examples compare the three methods on synthetic observations; the MATLAB example also plots rotation and translation errors.

## Contributing

Issues and pull requests are welcome. For software-related reports, include your operating system, compiler or MATLAB version, and a minimal example that reproduces the problem.

## Citations

If you use this work, please cite the accompanying manuscript:

> H. Cui, W. Dou, J. Shi, B. Wang, Y. Chen, and X. Liu, "A 6-DOF Hand-Eye Calibration Method Based on Position Residuals for Micromanipulation."

## License

This project is licensed under the [MIT License](LICENSE), copyright (c) 2026 Haoran Cui. Third-party components retain their [respective licenses](THIRD_PARTY_NOTICES.md).
