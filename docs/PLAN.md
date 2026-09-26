# Development plan

## Phase 1 — Full-mesh optical simulator (implemented)

- Load and transform the complete binary STL.
- Build an Embree triangle BVH.
- Project candidate Maxeon cells onto the shell.
- Calculate direct, diffuse-sky, and recursively reflected irradiance.
- Export per-cell results and aggregate incident optical power.
- Render the original triangle shell and irradiance heat map in a native OpenGL GUI.

Acceptance remaining: build and run the complete application on the target workstation, then
perform ray-count convergence and canonical-scene comparisons.

## Phase 2 — CAD semantics and calibrated materials

- Import the SolidWorks assembly hierarchy and component transformations.
- Separate eligible solar shell, canopy, body, wheels, openings, and keep-out regions.
- Map component or triangle groups to named optical materials.
- Replace provisional reflectance values with measured or sourced broadband BRDF inputs.
- Import the installed cell layout when it exists.

## Phase 3 — 2x3 / 1x3 layout optimization

- Generate feasible 2x3 and 1x3 Maxeon placement candidates over eligible shell regions.
- Enforce full footprint support, edge clearance, tile spacing, curvature, normal variation,
  canopy exclusion, and mechanical keep-outs.
- Precompute route-weighted irradiance value for every feasible candidate.
- Solve a binary weighted set-packing problem with non-overlap constraints.
- Optimize lexicographically: route energy, cell count, preference for 2x3 groups, then lower
  assembly complexity.
- Display accepted/rejected candidates and export cell transforms back to CAD-friendly data.

## Phase 4 — Route, solar position, and weather

- Add latitude, longitude, altitude, timestamp, heading, pitch, and roll inputs.
- Ingest cached interval-average DNI/DHI/GHI and temperature records.
- Transform the geographic sun direction into vehicle coordinates at every route step.
- Integrate per-cell incident energy over time.
- Add an anisotropic sky model after validating the isotropic baseline.

## Phase 5 — Electrical and thermal model

- Model encapsulation transmission and incidence-angle response.
- Estimate cell temperature from weather, speed, mounting, and irradiance.
- Add calibrated Maxeon I-V behavior, strings, mismatch, bypass diodes, wiring, and MPPT.
- Validate against reference-cell, pyranometer, temperature, and array-power measurements.
