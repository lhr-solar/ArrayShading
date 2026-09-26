# Current implementation status

## Implemented

- Single C++20 application at the repository root; no Python/Tk or voxel runtime remains.
- Streaming binary-STL loader with metre scaling, XYZ rotation, translation, validation, and
  tightly packed triangle storage.
- Full `_24-000.stl` support: 14,236,786 triangles and 711,839,384 bytes validated.
- Embree 4 high-quality BVH with exact full-mesh shadow and reflection queries.
- Automatic Maxeon Gen III candidate grid projected onto the STL upper envelope with locally
  smoothed normals.
- Per-cell direct sun, explicitly occluded diffuse sky, Lambert/Phong scene reflection, and
  bounded Whitted specular recursion.
- Multithreaded cell evaluation and CSV export.
- Native GLFW/OpenGL/Dear ImGui desktop application rendering the same full triangle buffer
  used by the solver.
- Lightweight live OpenGL shell irradiance preview with cursor readout and bounded
  fine-grained camera zoom; no per-triangle CPU/GPU heat buffer is allocated.
- Interactive sun/DNI/DHI, ray-count, recursion-depth, rerun, export, camera, heat legend,
  contribution breakdown, and display controls.
- CMake/vcpkg build, VS Code tasks, and deterministic C++ core tests.

## Model boundary

The current result is optical irradiance and incident optical watts. The automatic grid is a
placement preview, not the final array design. Shell and ground optical properties are still
provisional scalar Lambert/Phong parameters. Live shell coloring is an orientation preview;
validated occlusion and recursive reflection values are the Embree candidate-cell results.

## Next engineering inputs

1. SolidWorks Pack and Go plus STEP/glTF or separated component meshes.
2. Confirmation whether 2x3 and 1x3 cell groups are rigid or conformable.
3. Eligible-shell and mechanical keep-out regions.
4. Measured or sourced broadband optical properties for the shell, canopy, ground, and cell
   encapsulation.
5. Route, timestamp, pose, and DNI/DHI datasets for route-integrated layout scoring.

## Deferred

- Assembly-derived cell placement and material mapping.
- 2x3/1x3 layout candidate generation and set-packing optimization.
- Weather API cache and route energy integration.
- Cell temperature, electrical efficiency, strings, mismatch, bypass diodes, MPPT, and wiring.
