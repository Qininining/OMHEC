# Online Microscopic Hand-Eye Calibration

**OMHEC** implements position-residual hand-eye calibration from 3-D positions of a single tracked feature and known robot motions. Start with the reusable algorithms and the small examples below.

> Haoran Cui, Weichen Dou, Jia Shi, Bowen Wang, Yuhan Chen, and Xinyu Liu, "A 6-DOF Hand-Eye Calibration Method Based on Position Residuals for Micromanipulation."

## Methods

- **PR-IEKF**: analytical position-residual Jacobian, anisotropic noise, and multiplicative SE(3) correction.
- **PO-LS**: batch position-only least squares, with rotation projected onto SO(3).
- **PR-FGD**: recursive fixed-gain position-residual correction.

The core functions consume measurements and return estimates. Synthetic data, ground truth, evaluation, plots, file output, and hardware control belong to examples or the reproduction appendix.

## Structure

```text
OMHEC/
|-- C++/
|   |-- include/omhec/       Public headers
|   |-- src/                Estimators and shared SE(3) operations
|   |-- examples/
|   |   |-- simulation/     Hardware-free console demonstration
|   |   `-- hardware/       Optional Qt acquisition/control application
|   |-- dependence/         Eigen and OpenCV SDK files
|   |-- tests/              Numerical regression and geometry checks
|   `-- CMakeLists.txt
|-- MATLAB/
|   |-- src/+omhec/         Reusable algorithms
|   |-- examples/           Measurements, method calls and simple error plots
|   `-- tests/
|-- reproduction/
|   |-- simulation/         Paper sweeps and publication figure exports
|   `-- physical_experiments/ Recorded data and offline analysis
`-- docs/                   Manuscript and validation notes
```

## Dependencies

**MATLAB examples:** base MATLAB with modern array/string syntax; validated on R2025b. No Qt, hardware SDK, or LaTeX is needed. The optional paper reproduction appendix has additional plotting/export requirements.

**C++ core/examples:** C++17, CMake 3.18+, OpenCV Core and Eigen 3.4. The supplied Windows OpenCV import libraries require an MSVC x64 compiler and the matching OpenCV 4.10 runtime on `PATH`. On other platforms, provide an installed OpenCV through `OpenCV_DIR`. The bundled Eigen headers are used directly. Qt and device SDKs are only needed by the hardware example.

## MATLAB quick start

From the repository root:

```matlab
addpath('MATLAB/examples');
results = demo_calibration();        % Fixed-seed data and two ordinary error plots
% results = demo_calibration(false); % Same estimates without plotting
```

For your own data, add only the core directory:

```matlab
addpath('MATLAB/src');
cfg = omhec.default_config();
state = omhec.initialize(Tcs0, cfg, pointCovariance);
[state, info] = omhec.update(state, stageTwist, pPrevious, pCurrent, 'iekf');
Tcs = state.Tcs;
P = state.P;
% Use 'fgd' with a separate state to run PR-FGD.
% For batch PO-LS:
Tsc = omhec.solve_position_only_ls(stagePoses, cameraPoints);
```

`Tcs0` is an initial 4-by-4 estimate, `stageTwist` is a 6-by-1 stage-frame motion increment, and the two positions are 3-by-1 camera-frame observations. See [MATLAB usage](MATLAB/README.md) for the initialization state and data conventions.

## C++ quick start

From the repository root, in a compiler-enabled terminal:

```text
cmake -S C++ -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Run `build/demo_calibration` for a single-configuration generator, or `build/Release/demo_calibration.exe` with Visual Studio. The program creates one fixed-seed dataset, calls all three estimators and prints each transform and final rotation/translation error. It does not connect to devices or produce publication figures.

Include `<omhec/calibration.hpp>` and link the `omhec` CMake target to use the library in another C++ program. See the [C++ usage and minimal calls](C++/README.md).

## Input and output conventions

- `Tcs` maps stage/base coordinates to camera coordinates; `Tsc` is its inverse. MATLAB recursive state stores `Tcs`; the C++ adapter retains the original hardware convention and stores `Tsc`. Both batch PO-LS APIs return `Tsc`.
- Twists and left correction vectors are ordered **[rotation; translation]**. Angles are in radians. Use a single consistent length unit throughout positions, transform translations, twist translations, and covariance blocks. The synthetic examples use millimetres.
- MATLAB takes **stage-frame** twists and computes the camera-frame increment internally. C++ takes a **camera-frame** increment evaluated at the current estimate; the example shows the adjoint conversion.
- Recursive algorithms collect six motions by default before the first correction. State objects retain the initialization observations and covariance. Do not share mutable state between methods.
- The analytical Jacobian is the paper's first-order instantaneous-motion Jacobian. It is not an exact derivative of an arbitrary finite SE(3) increment.
- The implementations preserve their existing numerical conventions: the MATLAB simulation uses correlated adjacent residuals at initialization; the C++ hardware implementation retains its configurable covariance propagation and innovation gate. Defaults are documented per language; the demonstrations are not a claim of cross-language numerical identity.

## Hardware example and reproduction appendix

The [Qt hardware example](C++/examples/hardware/README.md) consumes the same C++ core sources through compatibility declarations. Historical estimators and acquisition utilities remain local to that example. The CMake core build does not include it.

[reproduction/](reproduction/README.md) contains the paper's parameter sweeps, publication plotting, recorded 5X/10X/20X data and offline physical analysis. These are optional workflows, separate from the algorithm quick start. The simulations call the shared MATLAB core. The [manuscript](docs/HandEyeCalibration.pdf) gives the derivation and experimental context.

## Tests

```matlab
addpath('MATLAB/tests');
test_core
```

Tests compare estimates and covariances against frozen outputs from the pre-extraction implementations, and check SE(3), the analytical Jacobian, initialization and rank-deficient inputs. Details and environment: [docs/VALIDATION.md](docs/VALIDATION.md).

## Contributing and citation

Bug reports should include the minimal input, command, operating system, MATLAB release or compiler/OpenCV versions, and the observed result. If you use this code, please cite the manuscript listed above. Bibliographic details will be updated when a publication record is available.

## License

Original OMHEC software and software documentation use the [MIT License](LICENSE), copyright (c) 2026 Haoran Cui. Dependencies retain their own licenses; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). The paper and setup illustration retain their authors' rights.
