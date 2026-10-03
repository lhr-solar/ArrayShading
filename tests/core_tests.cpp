#include "solar/core.hpp"
#include "solar/simulation.hpp"
#include "solar/trace_scene.hpp"
#include "solar/weather.hpp"

#include <cmath>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

bool close(float a, float b, float tolerance = 1.0e-5f) {
  return std::abs(a - b) <= tolerance;
}

int failures = 0;

void require(bool condition, std::string_view message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  ++failures;
}

}  // namespace

int main() {
  using solar::Vec3;
  const Vec3 n = solar::normalized({3.0f, 4.0f, 0.0f});
  require(close(n.x, 0.6f), "vector normalization x");
  require(close(n.y, 0.8f), "vector normalization y");
  require(close(solar::dot(solar::cross({1, 0, 0}, {0, 1, 0}),
                           {0, 0, 1}),
                1.0f),
          "right-handed cross product");

  solar::Transform transform;
  const Vec3 transformed = transform.apply({0.0f, 0.0f, 1.0f});
  require(close(transformed.x, 0.0f), "transform x");
  require(close(transformed.y, -1.6563f), "transform rotation/translation y");
  require(close(transformed.z, 0.0153f), "transform rotation/translation z");

  const Vec3 overhead = solar::sun_direction_from_azimuth_elevation(42.0f, 90.0f);
  require(close(overhead.x, 0.0f), "overhead sun x");
  require(close(overhead.y, 0.0f), "overhead sun y");
  require(close(overhead.z, 1.0f), "overhead sun z");

  for (std::uint32_t i = 0; i < 64; ++i) {
    const Vec3 direction = solar::cosine_hemisphere(i, 64, {0, 0, 1});
    require(close(solar::length(direction), 1.0f),
            "hemisphere sample unit length");
    require(direction.z >= 0.0f, "hemisphere sample above surface");
  }

  // 2024 March equinox at Greenwich, 12:00 UTC. NOAA's approximation puts
  // the sun close to overhead; azimuth becomes sensitive near the zenith.
  const solar::SolarPosition equinox_noon =
      solar::solar_position_utc(0.0, 0.0, 1710936000LL);
  require(equinox_noon.elevation_degrees > 87.0f &&
              equinox_noon.elevation_degrees <= 90.0f,
          "solar position equinox elevation");
  require(equinox_noon.azimuth_degrees >= 0.0f &&
              equinox_noon.azimuth_degrees < 360.0f,
          "solar position azimuth normalization");

  // A narrow overhead rectangle blocks one of three sample columns across a
  // one-metre cell: six of nine direct rays remain visible.
  solar::TriangleMesh shade_mesh;
  for (const Vec3 vertex : {
           Vec3{-1.0f, -1.0f, 1.0f}, Vec3{-0.1f, -1.0f, 1.0f},
           Vec3{-0.1f, 1.0f, 1.0f}, Vec3{-1.0f, -1.0f, 1.0f},
           Vec3{-0.1f, 1.0f, 1.0f}, Vec3{-1.0f, 1.0f, 1.0f}}) {
    shade_mesh.vertices.push_back(vertex);
    shade_mesh.bounds.include(vertex);
  }
  solar::TraceScene shade_scene(shade_mesh);
  solar::Cell test_cell;
  test_cell.center = {0.0f, 0.0f, 0.5f};
  test_cell.normal = {0.0f, 0.0f, 1.0f};
  test_cell.u_axis = {1.0f, 0.0f, 0.0f};
  test_cell.v_axis = {0.0f, 1.0f, 0.0f};
  test_cell.width_m = 1.0f;
  test_cell.height_m = 1.0f;
  test_cell.active_area_m2 = 1.0f;
  std::vector<solar::Cell> test_cells{test_cell};
  solar::Sun test_sun;
  test_sun.direction_to_sun = {0.0f, 0.0f, 1.0f};
  test_sun.dni_w_m2 = 900.0f;
  test_sun.dhi_w_m2 = 0.0f;
  solar::SimulationSettings test_settings;
  test_settings.direct_samples_per_axis = 3;
  test_settings.hemisphere_samples = 1;
  test_settings.max_reflection_depth = 0;
  test_settings.worker_threads = 1;
  solar::simulate_cells(test_cells, shade_scene, test_sun, test_settings);
  require(close(test_cells[0].direct_visibility_fraction, 6.0f / 9.0f,
                1.0e-4f),
          "partial-cell direct visibility fraction");
  require(close(test_cells[0].direct_w_m2, 600.0f, 1.0e-3f),
          "partial-cell direct irradiance");

  // A rerun must clear an earlier direct result rather than retaining stale
  // illumination when weather or sun position changes.
  test_sun.dni_w_m2 = 0.0f;
  solar::simulate_cells(test_cells, shade_scene, test_sun, test_settings);
  require(close(test_cells[0].direct_w_m2, 0.0f),
          "direct irradiance reset between runs");

  // A complete 3x3 candidate patch should prefer one adjacent 2x3 module,
  // then use one 1x3 module for the remaining column.
  solar::CellGrid layout_grid;
  layout_grid.rows = 3;
  layout_grid.columns = 3;
  std::vector<solar::Cell> layout_candidates;
  for (std::uint32_t row = 0; row < layout_grid.rows; ++row) {
    for (std::uint32_t column = 0; column < layout_grid.columns; ++column) {
      solar::Cell cell;
      cell.row = row;
      cell.column = column;
      cell.center = {static_cast<float>(column), static_cast<float>(row), 0.0f};
      cell.normal = {0.0f, 0.0f, 1.0f};
      cell.u_axis = {1.0f, 0.0f, 0.0f};
      cell.v_axis = {0.0f, 1.0f, 0.0f};
      layout_candidates.push_back(cell);
    }
  }
  const solar::GeneratedLayout generated =
      solar::generate_vertical_module_layout(layout_candidates, layout_grid);
  require(generated.summary.two_by_three_modules == 1,
          "layout prioritizes one 2x3 module");
  require(generated.summary.one_by_three_modules == 1,
          "layout fills remainder with one 1x3 module");
  require(generated.summary.individual_cells == 9 &&
              generated.cells.size() == 9,
          "layout includes all nine supported cells");
  require(generated.cells.front().module_rows == 3 &&
              generated.cells.front().module_columns == 2,
          "layout records module dimensions per cell");

  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed\n";
    return 1;
  }
  std::cout << "core tests passed\n";
  return 0;
}
