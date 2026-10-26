# Third-party notices

The root [MIT license](LICENSE) applies to original OMHEC software and software documentation. It does not replace the terms in third-party files. Existing copyright notices and license texts are retained with the dependencies.

| Component | Location or source | Terms and retained notices |
| --- | --- | --- |
| Eigen 3.4.0 | `C++/dependence/eigen-3.4.0/` | See `COPYING.README`, `COPYING.MPL2`, and the other `COPYING.*` files. Individual files retain their respective notices. |
| OpenCV 4.10.0 | `C++/dependence/opencv/` | Upstream release license: Apache 2.0; see the included `LICENSE`. Older file-specific license blocks remain in the supplied headers. |
| Basler pylon / GenICam | `C++/examples/hardware/dependence/pylon/` | Basler and GenICam notices remain in the headers. This SDK subset is not relicensed under MIT; obtain the applicable SDK license and runtime from Basler. |
| PI GCS2 | `C++/examples/hardware/dependence/PI/` | Physik Instrumente SDK terms; supplied header notices and `README_PI_GCS2.txt` are retained. |
| AgeMotion / AgeCOM | `C++/examples/hardware/dependence/AgeMotionForDriver/` | Vendor SDK/device authorization terms. The existing DLL and header are retained without an MIT relicensing statement. |
| Qt | External build/runtime dependency | Use the license applicable to the installed Qt distribution and modules. Qt binaries are not bundled here. |
| MATLAB | External execution environment | Requires an appropriate MathWorks installation/license; MATLAB itself is not bundled. |
| TeX Live / LaTeX packages | External Fig. 3 export dependency | The installed tools and packages retain their own licenses and are not bundled. |

The OpenCV license copy is taken from the [official OpenCV 4.10.0 source release](https://github.com/opencv/opencv/blob/4.10.0/LICENSE). Third-party binaries may include additional dependencies and notices; retain those supplied with their original distributions. The included hardware SDK subsets do not provide a complete vendor license/runtime distribution; this document does not grant additional redistribution rights.

The paper PDF and setup illustration in `docs/` are reference materials by the manuscript authors. They retain their existing rights and are not covered by the software MIT license. Recorded experiment CSV files preserve their original contents and names; their provenance is described in the physical-analysis README.
