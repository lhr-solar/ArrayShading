#include "solar/placement_surface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace solar {
namespace {

Vec3 triangle_centroid(const TriangleMesh& mesh, std::size_t triangle) {
  const std::size_t first = triangle * 3U;
  return (mesh.vertices[first] + mesh.vertices[first + 1U] +
          mesh.vertices[first + 2U]) /
         3.0f;
}

Vec3 triangle_normal(const TriangleMesh& mesh, std::size_t triangle) {
  const std::size_t first = triangle * 3U;
  const Vec3 raw = cross(mesh.vertices[first + 1U] - mesh.vertices[first],
                         mesh.vertices[first + 2U] - mesh.vertices[first]);
  if (length(raw) <= 1.0e-12f) return {};
  return normalized(raw);
}

std::optional<Hit> vertical_hit(const TraceScene& scene, float x, float y,
                                float start_z) {
  return scene.intersect({x, y, start_z}, {0.0f, 0.0f, -1.0f}, 1.0e-6f,
                         1.0e30f, kCarMask);
}

bool aligned_at(const TraceScene& full_scene, const Vec3& point,
                const Vec3& normal, float start_z, float tolerance,
                float minimum_upward_normal_z, float* error_out = nullptr) {
  const auto full_hit = vertical_hit(full_scene, point.x, point.y, start_z);
  if (!full_hit || full_hit->normal.z < minimum_upward_normal_z ||
      dot(full_hit->normal, normal) < 0.75f)
    return false;
  const float error = std::abs(full_hit->point.z - point.z);
  if (error_out) *error_out = error;
  return error <= tolerance;
}

}  // namespace

PlacementSurface prepare_placement_surface(
    const TriangleMesh& top_shell, const TraceScene& full_car,
    const PlacementSurfaceSettings& settings) {
  if (!std::isfinite(settings.alignment_tolerance_m) ||
      settings.alignment_tolerance_m < 0.0f ||
      !std::isfinite(settings.minimum_upward_normal_z) ||
      settings.minimum_upward_normal_z < 0.0f ||
      settings.minimum_upward_normal_z > 1.0f)
    throw std::invalid_argument("Invalid placement-surface settings");
  if (top_shell.triangle_count() == 0U ||
      top_shell.vertices.size() % 3U != 0U)
    throw std::invalid_argument("Top-shell STL contains no complete triangles");

  PlacementSurface output;
  output.region.minimum = top_shell.bounds.minimum;
  output.region.maximum = top_shell.bounds.maximum;
  const float vertical_padding = settings.alignment_tolerance_m + 0.007f;
  output.region.minimum.z -= vertical_padding;
  output.region.maximum.z += vertical_padding;
  // This scene is already semantically restricted to the body top; canopy
  // exclusion is established by matching against the complete-car exterior.
  output.region.use_canopy_exclusion = false;
  output.region.minimum_upward_normal_z = settings.minimum_upward_normal_z;

  const float start_z = top_shell.bounds.maximum.z + 10.0f;
  const std::size_t triangle_count = top_shell.triangle_count();
  const std::size_t stride = std::max<std::size_t>(1U, (triangle_count + 255U) / 256U);
  std::size_t sampled = 0;
  std::size_t matched = 0;
  double max_error = 0.0;
  for (std::size_t triangle = 0; triangle < triangle_count; triangle += stride) {
    const Vec3 center = triangle_centroid(top_shell, triangle);
    Vec3 normal = triangle_normal(top_shell, triangle);
    if (std::abs(normal.z) < settings.minimum_upward_normal_z) continue;
    if (normal.z < 0.0f) normal = -normal;
    ++sampled;
    float error = 0.0f;
    if (aligned_at(full_car, center, normal, start_z,
                   settings.alignment_tolerance_m,
                   settings.minimum_upward_normal_z, &error)) {
      ++matched;
      max_error = std::max(max_error, static_cast<double>(error));
    }
  }
  output.checked_samples = sampled;
  output.matched_samples = matched;
  output.max_alignment_error_m = max_error;
  if (sampled == 0U)
    throw std::runtime_error("Top-shell STL has no sufficiently upward-facing surface samples");
  if (matched * 2U <= sampled)
    throw std::runtime_error(
        "Top-shell STL does not align with the complete car mesh: most sampled roof points differ by more than " +
        std::to_string(settings.alignment_tolerance_m) +
        " m; check the CAD export transform and shared coordinate frame");

  for (std::size_t triangle = 0; triangle < triangle_count; ++triangle) {
    const Vec3 center = triangle_centroid(top_shell, triangle);
    Vec3 normal = triangle_normal(top_shell, triangle);
    if (std::abs(normal.z) < settings.minimum_upward_normal_z ||
        !aligned_at(full_car, center,
                    normal.z < 0.0f ? -normal : normal, start_z,
                    settings.alignment_tolerance_m,
                    settings.minimum_upward_normal_z))
      continue;
    const std::size_t first = triangle * 3U;
    output.mesh.vertices.insert(output.mesh.vertices.end(),
                                top_shell.vertices.begin() +
                                    static_cast<std::ptrdiff_t>(first),
                                top_shell.vertices.begin() +
                                    static_cast<std::ptrdiff_t>(first + 3U));
    for (std::size_t vertex = 0; vertex < 3U; ++vertex)
      output.mesh.bounds.include(top_shell.vertices[first + vertex]);
  }
  if (output.mesh.triangle_count() == 0U)
    throw std::runtime_error(
        "No top-shell triangles align with the complete car's upward-facing exterior");
  return output;
}

bool validate_shell_cell(const Cell& cell, const TraceScene& top_scene,
                         const TraceScene& full_scene, const CellGrid& grid,
                         float tolerance_m) {
  if (!std::isfinite(tolerance_m) || tolerance_m < 0.0f ||
      !std::isfinite(cell.width_m) || !std::isfinite(cell.height_m) ||
      cell.width_m <= 0.0f || cell.height_m <= 0.0f ||
      !std::isfinite(cell.center.x) || !std::isfinite(cell.center.y) ||
      !std::isfinite(cell.center.z) || !std::isfinite(cell.normal.x) ||
      !std::isfinite(cell.normal.y) || !std::isfinite(cell.normal.z) ||
      !std::isfinite(cell.u_axis.x) || !std::isfinite(cell.u_axis.y) ||
      !std::isfinite(cell.u_axis.z) || !std::isfinite(cell.v_axis.x) ||
      !std::isfinite(cell.v_axis.y) || !std::isfinite(cell.v_axis.z) ||
      length(cell.normal) <= 1.0e-12f)
    return false;
  const Vec3 normal = normalized(cell.normal);
  const float start_z = cell.center.z +
                        10.0f + 0.5f * std::max(cell.width_m, cell.height_m);
  // A 5-by-5 grid includes center, edges, and corners, and catches support
  // gaps too small to be represented by the optimizer's nine probes.
  constexpr int kSamplesPerAxis = 5;
  const float top_allowance = grid.maximum_support_plane_deviation_m +
                              grid.surface_offset_m + tolerance_m;
  const float maximum_clearance = grid.maximum_support_plane_deviation_m +
                                  grid.surface_offset_m;
  constexpr float kClearanceToleranceM = 1.0e-5f;
  for (int row = 0; row < kSamplesPerAxis; ++row) {
    const float v = (static_cast<float>(row) /
                     static_cast<float>(kSamplesPerAxis - 1) -
                     0.5f) * cell.height_m;
    for (int column = 0; column < kSamplesPerAxis; ++column) {
      const float u = (static_cast<float>(column) /
                       static_cast<float>(kSamplesPerAxis - 1) -
                       0.5f) * cell.width_m;
      const Vec3 probe = cell.center + cell.u_axis * u + cell.v_axis * v;
      const auto top_hit = vertical_hit(top_scene, probe.x, probe.y, start_z);
      const auto full_hit = vertical_hit(full_scene, probe.x, probe.y, start_z);
      if (!top_hit || !full_hit ||
          top_hit->normal.z < grid.minimum_upward_normal_z ||
          full_hit->normal.z < grid.minimum_upward_normal_z ||
          dot(top_hit->normal, normal) < 0.75f ||
          dot(full_hit->normal, normal) < 0.75f ||
          std::abs(top_hit->point.z - full_hit->point.z) > tolerance_m ||
          std::abs(dot(top_hit->point - cell.center, normal)) >
              top_allowance)
        return false;
      const float full_clearance =
          dot(cell.center - full_hit->point, normal);
      if (full_clearance < grid.surface_offset_m - kClearanceToleranceM ||
          full_clearance > maximum_clearance + kClearanceToleranceM)
        return false;
    }
  }
  return true;
}

bool conform_shell_cell(Cell& cell, const TraceScene& top_scene,
                        const TraceScene& full_scene, const CellGrid& grid,
                        float tolerance_m) {
  if (!std::isfinite(tolerance_m) || tolerance_m < 0.0f ||
      !std::isfinite(cell.width_m) || !std::isfinite(cell.height_m) ||
      cell.width_m <= 0.0f || cell.height_m <= 0.0f ||
      !std::isfinite(cell.center.x) || !std::isfinite(cell.center.y) ||
      !std::isfinite(cell.center.z) || !std::isfinite(cell.normal.x) ||
      !std::isfinite(cell.normal.y) || !std::isfinite(cell.normal.z) ||
      !std::isfinite(cell.u_axis.x) || !std::isfinite(cell.u_axis.y) ||
      !std::isfinite(cell.u_axis.z) || !std::isfinite(cell.v_axis.x) ||
      !std::isfinite(cell.v_axis.y) || !std::isfinite(cell.v_axis.z) ||
      length(cell.normal) <= 1.0e-12f)
    return false;
  const Vec3 normal = normalized(cell.normal);
  const float start_z = cell.center.z +
                        10.0f + 0.5f * std::max(cell.width_m, cell.height_m);
  constexpr int kSamplesPerAxis = 5;
  float maximum_support_above_center = -std::numeric_limits<float>::infinity();
  for (int row = 0; row < kSamplesPerAxis; ++row) {
    const float v = (static_cast<float>(row) /
                     static_cast<float>(kSamplesPerAxis - 1) -
                     0.5f) * cell.height_m;
    for (int column = 0; column < kSamplesPerAxis; ++column) {
      const float u = (static_cast<float>(column) /
                       static_cast<float>(kSamplesPerAxis - 1) -
                       0.5f) * cell.width_m;
      const Vec3 probe = cell.center + cell.u_axis * u + cell.v_axis * v;
      const auto top_hit = vertical_hit(top_scene, probe.x, probe.y, start_z);
      const auto full_hit = vertical_hit(full_scene, probe.x, probe.y, start_z);
      if (!top_hit || !full_hit ||
          top_hit->normal.z < grid.minimum_upward_normal_z ||
          full_hit->normal.z < grid.minimum_upward_normal_z ||
          dot(top_hit->normal, normal) < 0.75f ||
          dot(full_hit->normal, normal) < 0.75f ||
          std::abs(top_hit->point.z - full_hit->point.z) > tolerance_m ||
          std::abs(dot(top_hit->point - cell.center, normal)) >
              grid.maximum_support_plane_deviation_m +
                  grid.surface_offset_m + tolerance_m)
        return false;
      maximum_support_above_center = std::max(
          maximum_support_above_center,
          dot(full_hit->point - cell.center, normal));
    }
  }
  const Vec3 original_center = cell.center;
  const float required_shift = std::max(
      0.0f, maximum_support_above_center + grid.surface_offset_m);
  cell.center += normal * required_shift;
  if (validate_shell_cell(cell, top_scene, full_scene, grid, tolerance_m))
    return true;
  cell.center = original_center;
  return false;
}

}  // namespace solar
