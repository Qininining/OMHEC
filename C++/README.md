# C++ core

`src/` contains the three estimators and shared math. `include/omhec/calibration.hpp` is the main API; `legacy.hpp` provides compatibility types for the original hardware application. Core files contain no Qt, GUI, device SDK, random-data generation or file output.

## Build

Use the CMake commands in the root README. Set `OMHEC_BUILD_EXAMPLES=OFF` or `BUILD_TESTING=OFF` to build only the library. Hardware is never built by this CMake project.

On Windows, use MSVC x64 and install the OpenCV 4.10.0 runtime DLL matching the supplied import libraries. On other platforms set `OpenCV_DIR` to an installed OpenCV package. The core links only OpenCV and uses bundled Eigen headers. No Qt runtime is needed by the console example or tests.

## Minimal recursive call

```cpp
#include <omhec/calibration.hpp>

omhec::Config cfg;
auto state = omhec::initialize(Tsc0, P0, Q, R, cfg);
omhec::update_iekf(state, cameraTwist, pPrevious, pCurrent);
// Or: omhec::update_fgd(otherState, cameraTwist, pPrevious, pCurrent, lengthScale);
cv::Mat Tsc = state.Tsc;
cv::Mat P = state.filter.all.KF.P;
```

All matrices are finite, double-precision `CV_64F`: `Tsc0` is 4x4, P0/Q are 6x6 and R is 3x3. R must be positive definite. Position arguments are `cv::Vec3d`. `cameraTwist` is 6x1, ordered [omega; v], in the camera frame. It is a motion increment, not a raw velocity. For a stage-frame increment:

```cpp
cv::Mat cameraTwist = cvMatFromEigen(
    Adjoint(eigenFromCvMat(state.Tsc.inv())) * eigenFromCvMat(stageTwist));
```

Tsc maps camera to stage; covariance describes the left correction of Tcs (its inverse). Translation may use any consistent length unit. Supply the same unit to positions, transform translations, twist translations and covariance blocks. PR-FGD's positive `lengthScale` is the existing normalization scale. The original default initialization count is six; estimates remain unchanged during the preceding warmup calls.

R describes residual covariance by default. `cfg.iekf_propagate_point_covariance=true` instead uses the original hardware point-covariance propagation, including its existing one-unit covariance floor. `cfg.innovation_gate_enabled` retains the original gate behavior. Choose units and covariance explicitly when using either setting.

Batch PO-LS takes a vector of tool-to-stage poses and the matching camera positions:

```cpp
cv::Mat Tsc = omhec::solve_position_only_ls(stagePoses, cameraPoints);
```

It rejects insufficient, malformed or rank-deficient observations. The recursive facade also rejects malformed sizes and nonfinite inputs. State retains the observation history for initialization; use a fresh state for a new calibration sequence.

`examples/simulation/main.cpp` shows the complete data flow and error calculation. `examples/hardware/` contains Qt acquisition/motion functionality and historical compatibility code, outside the core dependency boundary.
