# Optional Qt hardware example

This is the original acquisition, autofocus, motion-control and calibration application. The three published estimators are compiled from the shared `C++/src/estimators.cpp` and `math.cpp`; local `MHEC.h` supplies compatibility declarations. Historical methods and CSV output remain local to this example.

Use Windows x64, Qt MSVC x64 with Core, GUI, Widgets, StateMachine, Charts and Concurrent, plus the Basler, PI and AgeMotion hardware drivers. SDK headers/import libraries remain in `dependence/`; OpenCV and Eigen are shared from `C++/dependence/`. Install matching OpenCV and Basler runtime DLLs separately.

Open `OnlineMHEC.pro` in Qt Creator, select the MSVC x64 kit and build Release. Alternatively, from an x64 developer prompt with Qt bin on PATH and starting at the OMHEC root:

```bat
mkdir build-hardware
cd build-hardware
qmake ..\C++\examples\hardware\OnlineMHEC.pro CONFIG+=release CONFIG-=debug
nmake release
windeployqt release\OnlineMHEC.exe
```

The project copies AgeCOM.dll and PI_GCS2_DLL_x64.dll next to the executable. AgeMotion loads its DLL there; PI retains its environment and vendor-installation fallbacks. The existing `PI_HEXAPOD_*` settings and `PI_GCS2_DLL` override remain supported. Qt deployment does not provide vendor/OpenCV runtimes.

Timestamped calibration CSV files use the writable working directory unless an explicit path is supplied. Image/video destinations remain selected in the GUI. Device authorization and connection settings must match the laboratory equipment.

The core CMake build does not configure this project, and no hardware is needed for the console or MATLAB examples. Hardware validation covers compilation/deployment only, with no device connection or motion.
