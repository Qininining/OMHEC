PI GCS2 C++ API files for the hexapod controller

Source:
  PI_C-990_V3.1/Development/C++/API

Layout used by this project:
  include/PI_GCS2_DLL.h
  include/PI_GCS2_DLL_PF.h
  lib/x64/PI_GCS2_DLL_x64.lib
  bin/x64/PI_GCS2_DLL_x64.dll

Runtime behavior:
  ParallelPlatform loads PI_GCS2_DLL_x64.dll dynamically, following PI's
  General Controller/DynamicLoadSample pattern.

  The application first tries the DLL next to the executable. CMake copies
  bin/x64/PI_GCS2_DLL_x64.dll to the build output after each build.

  PI_GCS2_DLL can still be set as an environment variable to test another
  DLL manually, but no system PATH change is required for the default build.

Notes:
  This project is configured for x64 because the other vendor libraries are
  x64. Use the x64 PI DLL and import library together.
