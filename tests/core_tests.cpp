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

  // Placement accepts a fully supported horizontal footprint and its local
  // tangent orientation is world-up.
  solar::TriangleMesh flat_shell;
  for (const Vec3 vertex : {
           Vec3{-1.0f, -1.0f, 0.5f}, Vec3{1.0f, -1.0f, 0.5f},
           Vec3{1.0f, 1.0f, 0.5f}, Vec3{-1.0f, -1.0f, 0.5f},
           Vec3{1.0f, 1.0f, 0.5f}, Vec3{-1.0f, 1.0f, 0.5f}}) {
    flat_shell.vertices.push_back(vertex);
    flat_shell.bounds.include(vertex);
  }
  solar::TraceScene flat_scene(flat_shell);
  solar::CellGrid placement_grid;
  placement_grid.rows = 1;
  placement_grid.columns = 1;
  const std::vector<solar::Cell> flat_cells =
      solar::project_cells(flat_scene, flat_shell.bounds, placement_grid);
  require(flat_cells.size() == 1, "flat shell accepts one supported cell");
  require(!flat_cells.empty() && close(flat_cells[0].normal.x, 0.0f) &&
              close(flat_cells[0].normal.y, 0.0f) &&
              close(flat_cells[0].normal.z, 1.0f),
          "flat-shell cell is world-up");
  require(!flat_cells.empty() && close(flat_cells[0].center.z, 0.501f),
          "projected cell uses one millimetre vertical clearance");

  // A planar slope may rise substantially in world Z while remaining a valid
  // tangent surface. The cell should follow that plane rather than being
  // forced horizontal or rejected by raw height difference.
  solar::TriangleMesh sloped_shell;
  for (const Vec3 vertex : {
           Vec3{-1.0f, -1.0f, 0.3f}, Vec3{1.0f, -1.0f, 0.7f},
           Vec3{1.0f, 1.0f, 0.7f}, Vec3{-1.0f, -1.0f, 0.3f},
           Vec3{1.0f, 1.0f, 0.7f}, Vec3{-1.0f, 1.0f, 0.3f}}) {
    sloped_shell.vertices.push_back(vertex);
    sloped_shell.bounds.include(vertex);
  }
  solar::TraceScene sloped_scene(sloped_shell);
  const std::vector<solar::Cell> sloped_cells =
      solar::project_cells(sloped_scene, sloped_shell.bounds, placement_grid);
  require(sloped_cells.size() == 1,
          "planar sloped shell accepts tangent cell");
  require(!sloped_cells.empty() && sloped_cells[0].normal.x < -0.19f &&
              sloped_cells[0].normal.z > 0.98f,
          "cell follows sloped shell normal");

  // A smaller, height-disconnected upward-facing patch represents a wheel or
  // hub under an opening. Only the largest smooth grid component is retained.
  solar::TriangleMesh shell_with_hub;
  auto append_quad = [&](float x_min, float x_max, float z) {
    for (const Vec3 vertex : {
             Vec3{x_min, -1.0f, z}, Vec3{x_max, -1.0f, z},
             Vec3{x_max, 1.0f, z}, Vec3{x_min, -1.0f, z},
             Vec3{x_max, 1.0f, z}, Vec3{x_min, 1.0f, z}}) {
      shell_with_hub.vertices.push_back(vertex);
      shell_with_hub.bounds.include(vertex);
    }
  };
  append_quad(-0.25f, 0.063f, 0.5f);
  append_quad(0.065f, 0.20f, 0.2f);
  solar::TraceScene disconnected_scene(shell_with_hub);
  solar::CellGrid disconnected_grid;
  disconnected_grid.rows = 1;
  disconnected_grid.columns = 3;
  const std::vector<solar::Cell> connected_cells = solar::project_cells(
      disconnected_scene, shell_with_hub.bounds, disconnected_grid);
  require(connected_cells.size() == 2,
          "smaller disconnected hub component is removed");
  require(connected_cells.size() == 2 && connected_cells[0].column != 2 &&
              connected_cells[1].column != 2,
          "retained candidates belong to primary shell component");

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

  // A 1x3/2x3 group describes an arrangement of individually supported cells,
  // not a rigid planar plate. Curved-shell normals must not cause the layout
  // stage to reject locations that already passed footprint validation.
  for (solar::Cell& cell : layout_candidates) {
    cell.normal = cell.column == 0U
                      ? solar::normalized(Vec3{-0.5f, 0.0f, 0.8660254f})
                      : solar::normalized(Vec3{0.5f, 0.0f, 0.8660254f});
  }
  const solar::GeneratedLayout curved_generated =
      solar::generate_vertical_module_layout(layout_candidates, layout_grid);
  require(curved_generated.summary.two_by_three_modules == 1 &&
              curved_generated.summary.one_by_three_modules == 1 &&
              curved_generated.cells.size() == 9,
          "curved-shell cell groups are not treated as rigid flat plates");

  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed\n";
    return 1;
  }
  std::cout << "core tests passed\n";
  return 0;
}
