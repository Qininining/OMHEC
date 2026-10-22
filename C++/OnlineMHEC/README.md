# Qt/C++ hardware application

OnlineMHEC is the experimental application for camera acquisition, autofocus, motion control, and hand-eye calibration. Its existing algorithms and public C++ interfaces are retained. Offline reproduction is available separately under `MATLAB/`.

## Prerequisites

- Windows x64, C++17, and a Qt MSVC x64 kit. Qt 6.2.3 was used for build validation, with the Visual Studio 2022 x64 compiler.
- Qt modules: Core, GUI, Widgets, StateMachine, Charts, and Concurrent.
- OpenCV 4.10.0 MSVC x64 libraries, Eigen 3.4.0, Basler pylon, PI GCS2, and AgeMotion device support.

`dependence/` preserves the supplied SDK headers/import libraries and device DLLs. OpenCV and pylon runtime DLLs must be installed from matching vendor distributions and made available on `PATH` or beside the executable. Use a compatible MSVC kit; the supplied SDK import libraries do not support MinGW.

See [third-party notices](../../THIRD_PARTY_NOTICES.md) for the component inventory and licensing boundaries.

## Build

In Qt Creator, open `OnlineMHEC.pro`, select the MSVC x64 kit, and build Release. For a command-line shadow build, use an **x64 Native Tools Command Prompt** with the matching Qt `bin` directory on `PATH`, and start at the repository root:

```bat
mkdir build
cd build
qmake ..\C++\OnlineMHEC\OnlineMHEC.pro CONFIG+=release CONFIG-=debug
nmake release
windeployqt release\OnlineMHEC.exe
```

The project automatically copies `AgeCOM.dll` and `PI_GCS2_DLL_x64.dll` beside the executable after linking. AgeMotion loads its DLL from that directory, independently of the working directory. PI also supports its existing runtime configuration and installation-path fallbacks. OpenCV, pylon, and their dependent runtimes must be supplied separately; `windeployqt` deploys Qt dependencies only.

The project uses source-relative include and library paths; no personal project directory needs to be edited. The qmake file rejects unsupported operating systems and MinGW kits explicitly.

## Hardware use and outputs

The application targets the setup described in the manuscript: Basler camera, AgeMotion autofocus axis, and PI hexapod. Device drivers, SDK authorization, camera configuration, and controller connection settings must match the actual laboratory equipment. The existing `PI_HEXAPOD_*` environment settings and `PI_GCS2_DLL` override remain supported.

Calibration CSV files are timestamped and written relative to the application working directory unless an explicit path is supplied. Qt Creator users can select a writable build/run directory. Image/video destinations remain selectable in the GUI. These acquisition outputs are distinct from the bundled MATLAB input recordings.

Only compilation and DLL deployment are checked during repository validation. The application is not launched against a camera or motion stage as part of those checks; hardware operation requires the corresponding experimental setup.
