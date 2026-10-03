# Raytracing and radiometry audit

This note records the equations implemented by the simulator and the analytic checks used
to catch sign, normalization, occlusion, and unit errors.

## Coordinate and ray conventions

- World coordinates are right-handed, in meters, with +Z upward.
- `direction_to_sun` points from a surface toward the sun and is normalized on load.
- A `Ray.direction` is the direction in which the mathematical ray travels.
- Panel normals are `normalize(u_axis × v_axis)`.
- STL triangles are intersected two-sided through Embree 4. The hit normal is oriented
  against the incident ray so the local shading hemisphere faces the ray origin.
- Rays start with a configurable metric epsilon to avoid self-intersection. Other facets of
  the same STL are not excluded, so one part of a concave shell can shadow another.

## Provisional 2x3 / 1x3 layout selection

Let `C(r,c)` mean that the projected tangent cell location at longitudinal row `r` and
lateral column `c` has complete, locally planar upper-shell support. A 1x3 module beginning
at `(r,c)` is feasible when all three locations exist and remain normally coherent:

```text
C(r+i,c) = true                           for i in {0,1,2}
n(r,c) dot n(r+i,c) >= 0.85               for i in {0,1,2}
```

A 2x3 module applies the same test to both `c` and `c+1`. For each row phase `p` in
`{0,1,2}`, the generator visits `r = p, p+3, ...`, consumes every feasible adjacent pair as
a 2x3, and then consumes each feasible unpaired column as a 1x3. It selects the phase using
the lexicographic objective:

```text
maximize (number of 2x3 modules, number of 1x3 modules)
```

The selected modules cannot overlap because rows advance in three-row bands and consumed
columns advance past each accepted module. This objective implements the current stated
preference but is restricted to a common three-row phase; it is not yet the route-weighted
global set-packing formulation in the development plan.

## Direct beam

For a panel normal **n** and unit direction to the sun **s**:

```text
E_direct = DNI × max(0, n · s) × visibility
```

`visibility` is the visible fraction of a deterministic `N x N` sample grid across the cell
footprint. Every sample launches its own hard shadow ray, so it is 0 for a fully shaded cell,
1 for a fully illuminated cell, and intermediate for a partially shaded cell. There is no
inverse-square factor because DNI is already the irradiance at the vehicle and the sun is
modeled as a directional source.

Analytic tests cover a horizontal panel, a 60° tilted panel, and full occultation.

## Isotropic diffuse sky

Uniform upper-hemisphere sky radiance is:

```text
L_sky = DHI / π
```

because `DHI = ∫ L_sky cos(theta) dω = π L_sky` on a horizontal plane. Panel integration uses
cosine-weighted directions with PDF `p(ω) = cos(theta) / π`, so the Monte Carlo/quasi-Monte
Carlo estimator becomes:

```text
E_sky ≈ (π / N) Σ visible_sky_radiance(ω_i)
```

For an unobstructed tilted plane under an isotropic upper sky, the closed-form result is:

```text
E_sky = DHI × (1 + n_z) / 2
```

The deterministic Hammersley estimate is tested against this expression at 60° tilt.

## Diffuse reflection

A surface with total reflectance `ρ` and specular fraction `k_s` has diffuse reflectance
`ρ_d = ρ(1-k_s)`. Its Lambertian outgoing radiance is:

```text
L_diffuse = ρ_d E_incident / π
```

The total material reflectance cannot exceed 1, and the diffuse/specular split sums to `ρ`,
which prevents those two lobes from allocating more than the configured reflected energy.
The `ρE/π` identity is covered by an analytic unit test.

At secondary hit points the sky contribution uses the unobstructed isotropic view factor
`(1+n_z)/2`. Primary array points use explicit hemisphere rays, so the car and STL shell can
occlude their sky view.

## Glossy and recursive reflection

Directional-sun highlights use an energy-normalized modified Phong lobe:

```text
f_specular = ρ k_s (p + 2) / (2π) × max(0, r · v)^p
L_specular,sun = f_specular × DNI × max(0, n · s) × visibility
```

where `p` is `specular_exponent`, **r** is the mirror direction of the incoming sun ray, and
**v** points toward the preceding ray vertex. This explicit term is necessary because a
zero-solid-angle directional sun is almost never hit by a finite hemisphere sample set.

Whitted reflection rays additionally follow the perfect-mirror direction for reflected sky
and multi-bounce scene radiance, multiplied by `ρ k_s` at each bounce and bounded by
`max_reflection_depth`.

## Cell projection, spatial aggregation, and incident power

Candidate cell centers form a 125 mm plus gap grid in world `(x,y)`. Placement triangles are
first filtered by a configurable XYZ inclusion box and central canopy exclusion prism; they
are not deleted merely for being curved. A vertical center ray plus neighboring height
samples estimate the local tangent:

```text
t_x = p(x+delta,y) - p(x-delta,y)
t_y = p(x,y+delta) - p(x,y-delta)
n = normalize(t_x cross t_y), with n_z > 0
```

Nine vertical rays then sample the center, edges, and corners of the footprint. Every sample
must hit the eligible shell, have a coherent normal, and remain close to the tangent plane:

```text
abs((p_support - p_center) dot n) <= 0.006 m
```

Grid-neighbor candidates are connected only across smooth normal and height changes, and only
the largest connected region is retained as the primary outer shell. This preserves the
curved shell while removing isolated wheel-hub candidates. The accepted cell uses the local
tangent axes and a 1 mm normal clearance. The unfiltered STL is still used by all later
visibility rays. Direct-beam
visibility is averaged across the cell footprint; diffuse-sky and reflected irradiance are
currently evaluated at the cell center. With published Maxeon Gen III active area
`A_active = 0.0153 m²`:

```text
E_cell = mean(E_direct + E_sky + E_reflected)
P_incident,cell = A_active × E_cell
E_array = Σ(A_active × E_cell) / Σ A_active
P_incident,array = Σ P_incident,cell
```

This is optical incident sunlight, not electrical output. Efficiency, temperature, strings,
mismatch, bypass diodes, MPPT, and wiring are intentionally outside the current result.

## Top-shell visualization

The native GUI provides an immediate orientation preview directly in the OpenGL fragment
shader. For each displayed fragment it evaluates:

```text
E_shell,preview = DNI max(0, n dot s) + DHI (1 + n_z) / 2
```

This visualization updates continuously as GUI sliders move and does not allocate a second
full-mesh irradiance buffer. It intentionally excludes visibility, recursive reflections, and
eligible-shell classification. It is therefore a visual orientation diagnostic, not a solver
output. The candidate-cell values and exported CSV continue to use Embree visibility,
hemisphere integration, and recursive reflection.

## Weather and geographic sun direction

The Open-Meteo integration supplies DNI and DHI for a requested WGS84 coordinate at the
returned UTC model time step. Solar declination, equation of time, hour angle, zenith, and
geographic azimuth are calculated locally using the NOAA solar-position approximation.
Geographic azimuth is clockwise from true north. For a vehicle heading `h` measured using
the same convention, the simulator's car-relative azimuth is:

```text
azimuth_car = wrap_0_360(azimuth_geographic - h)
```

The solver consumes Open-Meteo DNI and DHI directly. Cloud cover is retained as metadata and
is not an additional attenuation multiplier because its effect is already represented in the
weather model's irradiance components.

## Validation currently automated

- Vector normalization and cross-product orientation.
- STL-to-world scale, rotation, and translation convention.
- Geographic azimuth/elevation conversion, including overhead sun.
- UTC/location solar-position elevation and azimuth normalization.
- Unit-length, upper-hemisphere cosine-weighted samples.
- A complete 3x3 placement patch resolves to one preferred 2x3 module plus one 1x3 fill
  module, with module dimensions retained on each cell.

The complete Embree/OpenGL target still needs its first target-workstation build and the
canonical radiometry/integration test suite described below.

## Remaining validation before race-strategy use

- Convergence sweeps versus panel and hemisphere sample counts.
- Comparison against a second physically based renderer on canonical scenes.
- Comparison against a trusted plane-of-array model for time/location datasets.
- Pyranometer/reference-cell measurements on the real vehicle.
- Sensitivity and uncertainty bounds for material reflectance and sky anisotropy.
