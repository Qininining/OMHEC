# C++ interfaces

Include `<mhec/calibration.hpp>` and link the CMake target `mhec`.

```cpp
mhec::Config cfg;
auto state = mhec::initialize(Tsc0, P0, Q, R, cfg);
mhec::update_iekf(state, cameraTwist, pPrevious, pCurrent);
// PR-FGD: mhec::update_fgd(state, cameraTwist, pPrevious, pCurrent, lengthScale);
// PO-LS:  mhec::solve_position_only_ls(toolToStagePoses, cameraPoints);
```

- Matrices use `CV_64F`: Tsc is 4x4, P0/Q are 6x6, R is 3x3, and cameraTwist is 6x1. Positions are `cv::Vec3d`.
- Tsc maps camera to stage. Covariance describes the left correction of Tcs = inverse(Tsc).
- Twists are camera-frame increments ordered [rotation; translation], in radians and one consistent length unit. Convert a stage increment using `Adjoint(Tcs)`; see the simulation example.
- Six observations are collected before the first correction by default. Estimated covariance is `state.filter.all.KF.P`.
- R is residual covariance by default. Point-covariance propagation and innovation gating retain the original configurable behavior; point propagation includes the original one-unit covariance floor.

Use MSVC x64 with the supplied Windows OpenCV libraries and matching OpenCV 4.10 runtime on PATH. Elsewhere, provide an installed OpenCV via `OpenCV_DIR`. Eigen headers are bundled. The core requires no Qt or device SDK.

CMake options: `MHEC_BUILD_EXAMPLES` and `BUILD_TESTING`. The [microscope demo](examples/microscope_demo/README.md) has its own qmake project.
