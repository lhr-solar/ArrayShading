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

## Direct beam

For a panel normal **n** and unit direction to the sun **s**:

```text
E_direct = DNI × max(0, n · s) × visibility
```

`visibility` is 0 or 1 from a hard shadow ray. There is no inverse-square factor because DNI
is already the irradiance at the vehicle and the sun is modeled as a directional source.

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

Candidate cell centers form a 125 mm plus gap grid in world `(x,y)`. A vertical ray finds the
topmost shell intersection. Nearby top intersections in `±x` and `±y` estimate tangents, and
the local cell normal is:

```text
t_x = p(x+δ,y) - p(x-δ,y)
t_y = p(x,y+δ) - p(x,y-δ)
n = normalize(t_x × t_y), with n_z > 0
```

Cells without a top-shell hit or below the configured minimum `n_z` are omitted. The current
C++ implementation evaluates the center point of each accepted cell. With published Maxeon
Gen III active area `A_active = 0.0153 m²`:

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

## Validation currently automated

- Vector normalization and cross-product orientation.
- STL-to-world scale, rotation, and translation convention.
- Geographic azimuth/elevation conversion, including overhead sun.
- Unit-length, upper-hemisphere cosine-weighted samples.

The complete Embree/OpenGL target still needs its first target-workstation build and the
canonical radiometry/integration test suite described below.

## Remaining validation before race-strategy use

- Convergence sweeps versus panel and hemisphere sample counts.
- Comparison against a second physically based renderer on canonical scenes.
- Comparison against a trusted plane-of-array model for time/location datasets.
- Pyranometer/reference-cell measurements on the real vehicle.
- Sensitivity and uncertainty bounds for material reflectance and sky anisotropy.
