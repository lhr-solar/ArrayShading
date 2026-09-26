# Solar Array Irradiance Studio

A full-triangle C++20 solar-car irradiance simulator intended for a high-performance Windows
or Linux workstation. It loads the complete binary STL, builds an Embree BVH, calculates
per-cell direct, diffuse-sky, and recursively reflected irradiance, exports CSV results, and
shows the result in a native OpenGL desktop GUI.

There is one implementation in this repository. The former Python/Tk and voxel-preview path
has been removed.

## Architecture

```text
full binary STL -> transformed triangle buffer
                         |-> Embree BVH -> irradiance solver -> per-cell CSV
                         `-> OpenGL VBO -> native interactive GUI
```

OpenGL displays the original transformed triangles. It does not feed pixels back into the
physics. Embree intersects the same triangles directly, so camera position and window
resolution cannot alter the numerical answer.

## Workstation requirements

- 64-bit Windows or Linux
- C++20 compiler (Visual Studio 2022 or a current GCC/Clang)
- CMake 3.24+
- Ninja
- vcpkg
- Current OpenGL driver
- Recommended: 32 GB RAM and 8 GB GPU memory for the 14.2-million-triangle source STL

Dependencies are declared in `vcpkg.json`: Embree 4, GLFW, GLEW, GLM, and Dear ImGui.

## Fastest Windows start

Install the **Desktop development with C++** workload in Visual Studio Installer and Git for
Windows. Then open the repository in VS Code and run this one command in its terminal:

```powershell
.\run_windows.bat
```

The launcher finds Visual Studio and automatically installs missing MSVC/Windows SDK, CMake,
Ninja, Git, vcpkg, Embree, GLFW, GLEW, GLM, and Dear ImGui dependencies. Windows may show an
administrator prompt while system build tools are installed. It then builds the project and
opens the GUI using
`%USERPROFILE%\Downloads\_24-000.stl`. A different STL can be supplied as the first argument:

```powershell
.\run_windows.bat "D:\path\to\car.stl"
```

The initial build downloads and compiles the dependencies, so it is much slower than later
runs. The launcher prints a specific Visual Studio Installer instruction if a required build
component is missing.

## One-time vcpkg setup

Windows PowerShell:

```powershell
git clone https://github.com/microsoft/vcpkg.git "$HOME\vcpkg"
& "$HOME\vcpkg\bootstrap-vcpkg.bat"
$env:VCPKG_ROOT = "$HOME\vcpkg"
```

Linux:

```bash
git clone https://github.com/microsoft/vcpkg.git "$HOME/vcpkg"
"$HOME/vcpkg/bootstrap-vcpkg.sh"
export VCPKG_ROOT="$HOME/vcpkg"
```

Keep `VCPKG_ROOT` set in the terminal that launches VS Code or CMake.

## Build and test

From the repository root:

```bash
cmake --preset release-vcpkg
cmake --build --preset release-vcpkg
ctest --preset release-vcpkg
```

The first configure downloads and builds the manifest dependencies. Later builds reuse them.
VS Code exposes the same steps as **Configure release build**, **Build simulator**, and
**Run C++ tests** tasks.

## Run

Linux:

```bash
./build/release/solar_whitted_cpp \
  --stl /path/to/_24-000.stl \
  --rays 64 \
  --output results/cell-irradiance.csv
```

Windows PowerShell:

```powershell
.\build\release\solar_whitted_cpp.exe `
  --stl "D:\path\to\_24-000.stl" `
  --rays 64 `
  --output results\cell-irradiance.csv
```

Or select **Run full-mesh simulator** from **Terminal > Run Task** in VS Code and paste the
absolute STL path when prompted.

The supplied `_24-000.stl` defaults are already encoded:

- input scale `0.001` (millimetres to metres);
- rotation `X=90 degrees`; and
- translation `(0, -0.6563, 0.0153)` metres.

Use `--help` to see all transform, sun, integration, cell-grid, thread, and output options.
Use `--headless` for CSV-only execution on a compute node.

## Native GUI

The executable opens a GLFW/OpenGL window after loading the STL and completing the initial
simulation. Its left control panel provides:

- azimuth, elevation, DNI, and DHI controls;
- hemisphere-ray and reflection-depth controls;
- a **Run Simulation** button that recomputes Embree results;
- current irradiance, incident optical watts, active area, and accepted-cell metrics;
- direct/sky/reflected contribution breakdown;
- irradiance color legend;
- shell, cells, ground-grid, sun-vector, and wireframe toggles;
- CSV export and camera reset.

The right viewport renders the complete STL plus heat-colored cells. Left-drag orbits, the
mouse wheel zooms, and Escape closes the application. The panel occupies its own screen area,
so it does not cover the car.

## Current optical model

For cell normal `n` and direction to the sun `s`:

```text
E_direct = DNI * max(0, n dot s) * visibility
```

Diffuse sky uses deterministic cosine-weighted hemisphere samples with isotropic sky
radiance `DHI / pi`. Shell and ground hits return Lambertian plus normalized Phong radiance;
their specular component launches a Whitted reflection ray to the configured depth.

Each accepted Maxeon Gen III cell uses a 125 mm footprint and `0.0153 m2` active area. Output
is optical irradiance and incident sunlight—not electrical power.

## Project map

```text
CMakeLists.txt                 build definition
CMakePresets.json              reproducible release build
vcpkg.json                     C++ dependencies
include/solar/                 public types and interfaces
src/stl_mesh.cpp               streaming binary-STL loader and transform
src/trace_scene.cpp            Embree triangle scene and ray queries
src/simulation.cpp             projection and irradiance integration
src/viewer.cpp                 OpenGL + Dear ImGui desktop application
src/main.cpp                   CLI and application startup
tests/core_tests.cpp           deterministic core checks
data/cells/                    Maxeon reference inputs
docs/                          equations, status, and roadmap
.vscode/                       build/test/run tasks
```

## Not implemented yet

- Real cell/component transforms from the SolidWorks assembly
- 2x3 and 1x3 layout optimization
- Per-component measured material BRDFs
- Weather/route time-series ingestion
- Temperature and electrical conversion
- Strings, mismatch, bypass diodes, wiring, and MPPT
