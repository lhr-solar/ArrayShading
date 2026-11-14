#include "solar/optimizer.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

solar::TriangleMesh flat_mesh() {
  solar::TriangleMesh mesh;
  for (const solar::Vec3 vertex : {
           solar::Vec3{-1.0f, -1.0f, 0.5f},
           solar::Vec3{1.0f, -1.0f, 0.5f},
           solar::Vec3{-1.0f, 1.0f, 0.5f},
           solar::Vec3{1.0f, -1.0f, 0.5f},
           solar::Vec3{1.0f, 1.0f, 0.5f},
           solar::Vec3{-1.0f, 1.0f, 0.5f}}) {
    mesh.vertices.push_back(vertex);
    mesh.bounds.include(vertex);
  }
  return mesh;
}

solar::TriangleMesh dome_mesh() {
  solar::TriangleMesh mesh;
  constexpr int divisions = 40;
  constexpr float minimum = -0.5f;
  constexpr float step = 1.0f / divisions;
  std::vector<solar::Vec3> vertices;
  vertices.reserve((divisions + 1) * (divisions + 1));
  for (int y = 0; y <= divisions; ++y) {
    for (int x = 0; x <= divisions; ++x) {
      const float px = minimum + static_cast<float>(x) * step;
      const float py = minimum + static_cast<float>(y) * step;
      const solar::Vec3 point{px, py, 0.5f + 0.5f * px * px};
      vertices.push_back(point);
      mesh.bounds.include(point);
    }
  }
  auto vertex = [&](int x, int y) -> const solar::Vec3& {
    return vertices[static_cast<std::size_t>(y * (divisions + 1) + x)];
  };
  for (int y = 0; y < divisions; ++y) {
    for (int x = 0; x < divisions; ++x) {
      const auto& a = vertex(x, y);
      const auto& b = vertex(x + 1, y);
      const auto& c = vertex(x, y + 1);
      const auto& d = vertex(x + 1, y + 1);
      mesh.vertices.insert(mesh.vertices.end(), {a, b, c, b, d, c});
    }
  }
  return mesh;
}

solar::Bounds point_bounds(float x, float y, float z) {
  solar::Bounds bounds;
  bounds.minimum = {x, y, z};
  bounds.maximum = {x, y, z};
  return bounds;
}

solar::ShellRegion permissive_region() {
  solar::ShellRegion region;
  region.minimum = {-1.0f, -1.0f, 0.0f};
  region.maximum = {1.0f, 1.0f, 1.0f};
  region.exclusion_minimum = {2.0f, 2.0f, 2.0f};
  region.exclusion_maximum = {3.0f, 3.0f, 3.0f};
  return region;
}

solar::OptimizationSettings single_anchor_settings() {
  solar::OptimizationSettings settings;
  settings.position_step_m = 0.5;
  settings.angle_step_degrees = 180.0;
  return settings;
}

void float_pitch_boundary_module_is_kept() {
  auto mesh = flat_mesh();
  solar::TraceScene scene(mesh);
  solar::CellGrid grid;
  const auto settings = single_anchor_settings();
  const auto candidates = solar::generate_module_candidates(
      scene, point_bounds(0.0f, -0.37f, 0.5f), grid, permissive_region(),
      settings);
  require(!candidates.empty(),
          "float-rounded exact-pitch axis-aligned module was rejected");
}

void rotated_footprint_cannot_cross_canopy_between_samples() {
  auto mesh = flat_mesh();
  solar::TraceScene scene(mesh);
  solar::CellGrid grid;
  auto region = permissive_region();
  region.exclusion_minimum = {-0.001f, 0.034f, 0.49f};
  region.exclusion_maximum = {0.001f, 0.036f, 0.51f};
  auto settings = single_anchor_settings();
  settings.angle_step_degrees = 45.0;
  const auto baseline = solar::generate_module_candidates(
      scene, point_bounds(0.0f, 0.0f, 0.5f), grid, permissive_region(), settings);
  bool baseline_rotated_center = false;
  for (const auto& candidate : baseline)
    for (const auto& cell : candidate.cells)
      baseline_rotated_center |= std::abs(cell.center.x) < 0.01f &&
          std::abs(cell.center.y) < 0.01f && cell.u_axis.y > 0.6f;
  require(baseline_rotated_center,
          "canopy regression needs a feasible rotated center cell before exclusion");
  const auto candidates = solar::generate_module_candidates(
      scene, point_bounds(0.0f, 0.0f, 0.5f), grid, region, settings);
  for (const auto& candidate : candidates) {
    for (const auto& cell : candidate.cells) {
      if (std::abs(cell.center.x) < 0.01f &&
          std::abs(cell.center.y) < 0.01f && cell.u_axis.y > 0.6f)
        throw std::runtime_error(
            "rotated cell crossing canopy between support samples was kept");
    }
  }
}

void curved_cell_plane_clears_all_sampled_supports() {
  auto mesh = dome_mesh();
  solar::TraceScene scene(mesh);
  solar::CellGrid grid;
  const auto settings = single_anchor_settings();
  const auto candidates = solar::generate_module_candidates(
      scene, point_bounds(0.0f, 0.0f, mesh.bounds.maximum.z), grid,
      permissive_region(), settings);
  bool checked_center_cell = false;
  for (const auto& candidate : candidates) {
    for (const auto& cell : candidate.cells) {
      if (std::abs(cell.center.x) > 0.01f ||
          std::abs(cell.center.y) > 0.01f)
        continue;
      checked_center_cell = true;
      const float start_z = mesh.bounds.maximum.z + 1.0f;
      for (int vi = -1; vi <= 1; ++vi) {
        for (int ui = -1; ui <= 1; ++ui) {
          const solar::Vec3 sample =
              cell.center +
              cell.u_axis * (static_cast<float>(ui) * grid.width_m * 0.5f) +
              cell.v_axis * (static_cast<float>(vi) * grid.height_m * 0.5f);
          const auto support = scene.intersect(
              {sample.x, sample.y, start_z}, {0.0f, 0.0f, -1.0f}, 1.0e-6f,
              1.0e30f, solar::kCarMask);
          require(support.has_value(), "dome support sample missed the shell");
          require(solar::dot(cell.center - support->point, cell.normal) >=
                      grid.surface_offset_m - 2.0e-4f,
                  "curved cell plane penetrates a sampled shell support");
        }
      }
    }
  }
  require(checked_center_cell, "dome did not produce a center placement cell");
}

}  // namespace

int main() {
  try {
    float_pitch_boundary_module_is_kept();
    rotated_footprint_cannot_cross_canopy_between_samples();
    curved_cell_plane_clears_all_sampled_supports();
  } catch (const std::exception& error) {
    std::cerr << "placement audit test failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
