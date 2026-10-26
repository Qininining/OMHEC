# MHEC

Position-only hand-eye calibration in C++ and MATLAB: **PR-IEKF**, **PO-LS**, and **PR-FGD**.

## Structure

```text
C++/
  include/mhec/               Public interfaces
  src/                        Core algorithms
  examples/simulation/        Console example
  examples/microscope_demo/   Qt microscope application
  tests/
MATLAB/
  src/+mhec/                  Core algorithms
  examples/                   Simulation and simple result plots
  tests/
paper_experiments/            Paper simulations and recorded-data analysis
```

## MATLAB

```matlab
addpath('MATLAB/examples');
demo_calibration;
```

For your own observations:

```matlab
addpath('MATLAB/src');
state = mhec.initialize(Tcs0, mhec.default_config(), pointCovariance);
state = mhec.update(state, stageTwist, pPrevious, pCurrent, 'iekf');
```

See [MATLAB interfaces](MATLAB/README.md) for dimensions and units.

## C++

Requires C++17, CMake 3.18+, OpenCV and Eigen. Qt is only needed for the microscope demo.

```sh
cmake -S C++ -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Run `build/demo_calibration` (Visual Studio: `build/Release/demo_calibration.exe`). See [C++ interfaces](C++/README.md) and the [microscope demo](C++/examples/microscope_demo/README.md).

## Paper and license

Haoran Cui et al., *A 6-DOF Hand-Eye Calibration Method Based on Position Residuals for Micromanipulation.* [Manuscript](docs/HandEyeCalibration.pdf) | [Paper experiments](paper_experiments/README.md) | [Validation](docs/VALIDATION.md)

[MIT License](LICENSE), copyright (c) 2026 Haoran Cui. Dependencies retain their [own licenses](THIRD_PARTY_NOTICES.md).
