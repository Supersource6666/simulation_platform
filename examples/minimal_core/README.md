# Minimal Chrono Core

This is an external consumer project: it links to Chrono through `find_package(Chrono CONFIG REQUIRED)` and `${CHRONO_TARGETS}`. It does not compile or copy Chrono sources itself.

From the repository root, using PowerShell and Visual Studio 2022 with C++ tools:

```powershell
cmake -S . -B build/core -G "Visual Studio 17 2022" -A x64 `
  -DCH_CORE_ONLY=ON -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKING=OFF `
  -DCH_ENABLE_YAML=OFF -DCH_ENABLE_HDF5=OFF -DCH_ENABLE_OPENMP=OFF `
  -DCH_USE_EIGEN_OPENMP=OFF -DCH_USE_SIMD=OFF -DBUILD_SHARED_LIBS=ON `
  -DEIGEN3_INCLUDE_DIR=D:/libs/eigen-3.4.1
cmake --build build/core --config Release --target Chrono_core --parallel 6 -- /p:CL_MPCount=6
cmake -S examples/minimal_core -B build/minimal_core -G "Visual Studio 17 2022" -A x64 `
  "-DChrono_DIR=$((Get-Location).Path)/build/core/cmake"
cmake --build build/minimal_core --config Release
ctest --test-dir build/minimal_core -C Release --output-on-failure
./build/minimal_core/Release/demo_core_minimal.exe
```

Adjust the Eigen header path for your machine. Eigen is required in addition to `src/chrono`, `src/chrono_thirdparty`, the root CMake files, and `cmake/`. Keep the existing `data/` directory because the root configuration copies it, although this example does not load assets.

`CH_CORE_ONLY=ON` skips optional modules, demos, tests and benchmarks in the Chrono build. It retains the full upstream Core (including its internal collision and FEA code); it is not a source-level reduction to just the time-stepping function. YAML, HDF5, OpenMP and SIMD are disabled by the command above. The upstream configuration may still detect installed GPU tools, but this example needs no GPU or visualization module.

The separate example has its own CTest check. It simulates a 1 kg body, initially at rest at the origin, with gravity `(0, -9.81, 0)` and no collisions. It takes 1000 steps of 0.001 seconds using linearized implicit Euler. Expected output:

```text
time=1.000000 s, y=-4.909905 m, vy=-9.810000 m/s
PASS
```

The exact continuous position is -4.905 m; the check allows the expected first-order integration error. The program returns a nonzero status if the time, velocity or position check fails. The required Chrono DLL is copied beside the executable at build time.

`Chrono_DIR` can point to the `cmake/` folder in a build tree or an install tree. This example uses the build tree. A local install is generally preferable for long-term external projects; installation of this stripped checkout has not been validated, and the upstream install rules still reference omitted template/importer directories.
