# Microscope demo

Qt application for camera acquisition, autofocus, motion control and calibration. The three calibration methods use the shared core in `C++/src`.

Requires Windows x64, an MSVC Qt kit with Core, GUI, Widgets, StateMachine, Charts and Concurrent, plus the matching Basler, PI and AgeMotion drivers. OpenCV/Eigen are shared from `C++/dependence`; device SDK files are under this example's `dependence/`.

Open `MHEC.pro` in Qt Creator, or run from the project root in a configured x64 developer prompt:

```bat
mkdir build-microscope
cd build-microscope
qmake ..\C++\examples\microscope_demo\MHEC.pro CONFIG+=release CONFIG-=debug
nmake release
windeployqt release\MHEC.exe
```

The build copies AgeCOM and PI DLLs beside the executable. Install matching OpenCV/pylon runtimes separately. Existing PI environment overrides remain available. Acquisition CSVs use the working directory; image/video paths are selected in the GUI.

Compilation is tested; live device operation requires the laboratory setup.
