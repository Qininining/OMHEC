# Reproduction and build validation

Validation date: 2026-10-22. Checks ran on Windows x64 using a separate copy
outside the OMHEC working tree. Generated results and build products are not
included in the release sources.

## MATLAB

Environment: MATLAB R2025b Update 4 (25.2.0.3150157), with TeX Live 2025
available for the Fig. 3 standalone PDF export.

- Reset MATLAB's search path before each workflow and checked that its entry
  point resolved inside the independent copy.
- Ran `run_simulation(true)`: all convergence and parameter-sweep error arrays
  were finite, all four joint studies contained finite terminal errors, and
  all four simulation PDFs were generated.
- Ran `run_simulation(true, 'joint')` separately: the explicit joint-study
  entry point completed and regenerated Fig. 3.
- Ran `run_analysis` on both an independent baseline copy of the current
  manuscript analysis and the reorganized package. All 11 generated CSV files
  and the LaTeX table matched exactly, including all three magnifications.
- Verified that simulation source files and the 159 physical input files
  match their current manuscript counterparts byte for byte. The personal
  `fig.m` plotting shortcut is excluded from the release.

The full 100-trial/100-step Monte Carlo run was not repeated during this
directory refactor. Its configuration, seeds, and numerical implementation
are unchanged; quick-mode results are for execution checks only. Earlier
MATLAB releases were not tested.

## C++

Environment: Qt 6.2.3 MSVC x64 kit; Visual Studio 2022 x64 compiler
(MSVC 14.44.35207); the bundled OpenCV/Eigen/device SDK files.

- Generated a shadow-build Makefile with qmake and built Release with nmake.
- Successfully linked `OnlineMHEC.exe`.
- Verified that post-link deployment copies `AgeCOM.dll` and
  `PI_GCS2_DLL_x64.dll` beside the executable, with contents identical to
  the supplied SDK files.
- Checked that AgeMotion resolves its DLL from the executable directory,
  without a personal absolute source path.

The application was not launched and no device was connected. Camera,
autofocus, motion, hardware timing, vendor authorization, and installation of
OpenCV/pylon runtime dependencies remain hardware/deployment checks. Existing
unused-variable/unused-parameter compiler warnings are retained.

## Materials and licensing

The manuscript PDF and setup illustration are unchanged copies of the current
manuscript materials. Third-party notices and Eigen license files are retained;
the OpenCV 4.10.0 upstream license is included beside its SDK subset. The
duplicated nested Eigen tree was removed from the release layout; the Eigen
tree used by the qmake include path remains intact.
