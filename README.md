# Solar Array Irradiance Studio

A full-triangle C++20 solar-car irradiance simulator intended for a high-performance Windows
or Linux workstation. It loads the complete binary STL, builds an Embree BVH, calculates
per-cell direct, diffuse-sky, and recursively reflected irradiance, exports CSV results, and
shows the result in a native OpenGL desktop GUI. The GUI can fetch current DNI, DHI,
temperature, and cloud metadata for any latitude/longitude from Open-Meteo.

There is one implementation in this repository. The former Python/Tk and voxel-preview path
has been removed.

## Architecture

```text
full binary STL -> transformed triangle buffer
                         |-> upper-shell candidates -> 2x3/1x3 layout
                         |                                  |
                         |-> Embree BVH -> irradiance solver -> per-cell CSV
                         `-> OpenGL VBO --------------------> native GUI

latitude/longitude -> Open-Meteo DNI/DHI + UTC time -> solar position
vehicle heading -------------------------------------> car-relative sun vector
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

Dependencies are declared in `vcpkg.json`: Embree 4, GLFW, GLEW, GLM, Dear ImGui, CPR,
and nlohmann/json.

## Fastest Windows start

Install the **Desktop development with C++** workload in Visual Studio Installer and Git for
Windows. Then open the repository in VS Code and run this one command in its terminal:

```powershell
.\run_windows.bat
```

The launcher finds Visual Studio and automatically installs missing MSVC/Windows SDK, CMake,
Ninja, Git, vcpkg, Embree, GLFW, GLEW, GLM, Dear ImGui, CPR, and JSON dependencies. Windows
may show an administrator prompt while system build tools are installed. It then builds the
project and opens the GUI using
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

The supplied `_24-000.stl` defaults are already encoded. Its vertex coordinates are in
metres (the measured transformed bounds are approximately 1.61 m wide, 6.20 m long, and
1.25 m tall):

- input scale `1.0`;
- rotation `X=90 degrees`; and
- translation `(0, -0.6563, 0.0153)` metres.

Use `--help` to see all transform, sun, integration, cell-grid, thread, and output options.
Use `--headless` for CSV-only execution on a compute node.

Candidate placement uses a configurable world-space upper-shell mask. The default inclusion
box is `x=[-0.68,0.68]`, `y=[-2.90,2.80]`, `z=[0.12,1.00]` metres, requires a local
upward-normal component of at least `0.75`, and removes a central canopy prism
`x=[-0.32,0.32], y=[-1.45,1.35]`. CLI `--shell-*`, `--canopy-*`, and
`--shell-normal-z` options tune these provisional cutoffs. The filtered mesh is used only to
place candidate cells; the complete STL remains in Embree and continues to cast shadows.
Every candidate then requires a complete 3x3 support sample across its 125 mm footprint.
Those points must remain within 6 mm of the locally fitted tangent plane and have consistent
normals. This accepts smooth top-shell curvature while rejecting openings, sharp hub/body
geometry, and quads that would bridge unrelated surfaces. The largest smooth connected
candidate region is retained as the primary shell, removing disconnected islands such as
wheel hubs. Accepted cells follow the local shell tangent with 1 mm clearance. The planarity
tolerance is adjustable with `--max-cell-plane-error`.

The projected locations are grouped into vertical modules before simulation. A module always
contains three consecutive longitudinal cells. At each feasible row band, adjacent columns
are taken as a 2x3 module first; any supported column left over becomes a 1x3 module. The
generator evaluates all three possible longitudinal row phases and selects the one with the
most 2x3 modules, breaking ties with the number of 1x3 modules. A module also requires its
constituent cell normals to remain mutually coherent. This is a deterministic prioritized
layout for the current regular grid, not yet the final route-energy set-packing optimizer.

## Native GUI

The executable opens a GLFW/OpenGL window after loading the STL and completing the initial
simulation. Its left control panel provides:

- azimuth, elevation, DNI, and DHI controls;
- latitude, longitude, vehicle heading, and asynchronous Open-Meteo current-weather fetch;
- direct-shadow footprint density, hemisphere-ray, and reflection-depth controls;
- a **Run Simulation** button that recomputes Embree results;
- current irradiance, incident optical watts, active area, selected-cell count, and 2x3/1x3
  module counts;
- direct/sky/reflected contribution breakdown;
- live OpenGL shell irradiance preview and color legend;
- shell, cells, ground-grid, sun-vector, and wireframe toggles;
- CSV export and camera reset.

The right viewport renders the complete STL plus the generated, heat-colored cell layout by
default. Cyan outlines identify cells in 2x3 modules and orange outlines identify cells in
1x3 modules. Left-drag orbits, the
mouse wheel zooms in small bounded increments, and Escape closes the application. The panel
occupies its own screen area, so it does not cover the car. The shell heat map evaluates the
local triangle orientation using DNI plus an isotropic diffuse-sky estimate and responds
immediately while the sun and irradiance sliders move. Hovering over the shell shows the same
preview value in W/m² beneath the legend. This live shell coloring intentionally omits
self-shadowing and reflections; the generated-layout result and CSV use the complete Embree
visibility and recursive integrator after **Run Simulation** is pressed.

### Open-Meteo weather input

Enter WGS84 latitude/longitude and the vehicle heading clockwise from true north, then click
**Fetch Current Weather**. The application requests the current model time step from
Open-Meteo, applies its DNI and DHI, computes geographic solar azimuth/elevation from the
returned UTC timestamp, and rotates the sun azimuth into the vehicle frame. Click
**Run Simulation** to recompute the exact per-cell result.

The solar-position calculation follows the equation-of-time, declination, hour-angle,
zenith, and clockwise-from-north azimuth conventions documented by
[NOAA Global Monitoring Laboratory](https://gml.noaa.gov/grad/solcalc/).

Cloud cover is displayed as provenance metadata but is not multiplied into the irradiance:
Open-Meteo's DNI and DHI already include modeled atmospheric and cloud effects. The weather
request runs outside the rendering thread, so a slow network response does not freeze camera
or GUI interaction. A 15-second request timeout is enforced.

The public endpoint is intended for eligible non-commercial use and requires attribution;
review [Open-Meteo's current terms](https://open-meteo.com/en/terms) before deployment. A
production/commercial deployment should use their customer endpoint and API key.

## Current optical model

For cell normal `n` and direction to the sun `s`:

```text
E_direct = DNI * max(0, n dot s) * visibility
```

`visibility` is the fraction of a configurable `N x N` shadow-ray grid across that cell's
footprint. The default `3 x 3` grid distinguishes fully lit, fully shaded, and partially
shaded cells. Each CSV row includes `direct_visibility_fraction`. Diffuse sky and scene
reflection are currently sampled from the cell center.

Diffuse sky uses deterministic cosine-weighted hemisphere samples with isotropic sky
radiance `DHI / pi`. Shell and ground hits return Lambertian plus normalized Phong radiance;
their specular component launches a Whitted reflection ray to the configured depth.

Each selected Maxeon Gen III cell uses a 125 mm footprint and `0.0153 m2` active area. Output
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
src/weather.cpp                Open-Meteo client and UTC solar-position calculation
src/viewer.cpp                 OpenGL + Dear ImGui desktop application
src/main.cpp                   CLI and application startup
tests/core_tests.cpp           deterministic core checks
data/cells/                    Maxeon reference inputs
docs/                          equations, status, and roadmap
.vscode/                       build/test/run tasks
```

## Not implemented yet

- Real cell/component transforms from the SolidWorks assembly
- Globally optimal, route-energy-weighted 2x3/1x3 set-packing and mechanical validation
- Per-component measured material BRDFs
- Weather/route time-series ingestion
- Temperature and electrical conversion
- Strings, mismatch, bypass diodes, wiring, and MPPT
