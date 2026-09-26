#include "solar/simulation.hpp"

#include "solar/stl_mesh.hpp"

#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace solar {
namespace {

std::optional<Hit> top_hit(const TraceScene& scene, float x, float y,
                           float start_z) {
  return scene.intersect({x, y, start_z}, {0.0f, 0.0f, -1.0f}, 1.0e-6f,
                         1.0e30f, kCarMask);
}

Vec3 estimated_normal(const TraceScene& scene, float x, float y, float start_z,
                      float delta, const Hit& center) {
  const auto left = top_hit(scene, x - delta, y, start_z);
  const auto right = top_hit(scene, x + delta, y, start_z);
  const auto back = top_hit(scene, x, y - delta, start_z);
  const auto front = top_hit(scene, x, y + delta, start_z);
  const Vec3 tx = left && right ? right->point - left->point
                  : right      ? right->point - center.point
                  : left       ? center.point - left->point
                               : Vec3{1.0f, 0.0f, 0.0f};
  const Vec3 ty = back && front ? front->point - back->point
                  : front       ? front->point - center.point
                  : back        ? center.point - back->point
                                : Vec3{0.0f, 1.0f, 0.0f};
  Vec3 normal = cross(tx, ty);
  if (length(normal) < 1.0e-8f) normal = center.normal;
  normal = normalized(normal);
  return normal.z < 0.0f ? -normal : normal;
}

std::pair<Vec3, Vec3> tangent_axes(const Vec3& normal) {
  Vec3 preferred{1.0f, 0.0f, 0.0f};
  Vec3 u = preferred - normal * dot(preferred, normal);
  if (length(u) < 1.0e-8f) {
    preferred = {0.0f, 1.0f, 0.0f};
    u = preferred - normal * dot(preferred, normal);
  }
  u = normalized(u);
  return {u, normalized(cross(normal, u))};
}

class Integrator {
 public:
  Integrator(const TraceScene& scene, const Sun& sun,
             const SimulationSettings& settings)
      : scene_(scene), sun_(sun), settings_(settings) {}

  void evaluate(Cell& cell) const {
    const Vec3 origin = cell.center + cell.normal * settings_.ray_epsilon_m;
    const float cosine = std::max(0.0f, dot(cell.normal, sun_.direction_to_sun));
    if (cosine > 0.0f &&
        scene_.visible(origin, sun_.direction_to_sun,
                       settings_.ray_epsilon_m)) {
      cell.direct_w_m2 = sun_.dni_w_m2 * cosine;
    }

    float sky_sum = 0.0f;
    float reflected_sum = 0.0f;
    for (std::uint32_t index = 0; index < settings_.hemisphere_samples; ++index) {
      const Vec3 direction = cosine_hemisphere(
          index, settings_.hemisphere_samples, cell.normal);
      const auto hit = scene_.intersect(origin, direction,
                                        settings_.ray_epsilon_m);
      if (!hit) {
        if (direction.z > 0.0f) sky_sum += sun_.dhi_w_m2 / kPi;
      } else {
        reflected_sum += shade_hit(direction, *hit, 0);
      }
    }
    const float weight = kPi / static_cast<float>(settings_.hemisphere_samples);
    cell.diffuse_sky_w_m2 = sky_sum * weight;
    cell.scene_reflected_w_m2 = reflected_sum * weight;
  }

 private:
  float trace_radiance(const Vec3& origin, const Vec3& direction,
                       std::uint32_t depth) const {
    const auto hit = scene_.intersect(origin, direction,
                                      settings_.ray_epsilon_m);
    if (!hit) return direction.z > 0.0f ? sun_.dhi_w_m2 / kPi : 0.0f;
    return shade_hit(direction, *hit, depth);
  }

  float shade_hit(const Vec3& incoming_direction, const Hit& hit,
                  std::uint32_t depth) const {
    const float sun_cosine =
        std::max(0.0f, dot(hit.normal, sun_.direction_to_sun));
    float direct_incident = 0.0f;
    if (sun_cosine > 0.0f &&
        scene_.visible(hit.point + hit.normal * settings_.ray_epsilon_m,
                       sun_.direction_to_sun,
                       settings_.ray_epsilon_m)) {
      direct_incident = sun_.dni_w_m2 * sun_cosine;
    }

    const float sky_view_factor =
        std::clamp((1.0f + hit.normal.z) * 0.5f, 0.0f, 1.0f);
    const float local_incident =
        direct_incident + sun_.dhi_w_m2 * sky_view_factor;
    const float diffuse = hit.material.reflectance *
                          (1.0f - hit.material.specular_fraction) *
                          local_incident / kPi;

    float glossy_sun = 0.0f;
    if (direct_incident > 0.0f && hit.material.specular_fraction > 0.0f) {
      const Vec3 mirror = normalized(reflected(-sun_.direction_to_sun,
                                               hit.normal));
      const Vec3 view = -incoming_direction;
      const float alignment = std::max(0.0f, dot(mirror, view));
      const float phong_brdf =
          hit.material.reflectance * hit.material.specular_fraction *
          (hit.material.specular_exponent + 2.0f) / (2.0f * kPi) *
          std::pow(alignment, hit.material.specular_exponent);
      glossy_sun = phong_brdf * direct_incident;
    }

    if (depth >= settings_.max_reflection_depth ||
        hit.material.specular_fraction <= 0.0f) {
      return diffuse + glossy_sun;
    }
    const Vec3 bounce = normalized(reflected(incoming_direction, hit.normal));
    const float specular =
        hit.material.reflectance * hit.material.specular_fraction *
        trace_radiance(hit.point + bounce * settings_.ray_epsilon_m, bounce,
                       depth + 1U);
    return diffuse + glossy_sun + specular;
  }

  const TraceScene& scene_;
  const Sun& sun_;
  const SimulationSettings& settings_;
};

}  // namespace

std::vector<Cell> project_cells(const TraceScene& scene, const Bounds& bounds,
                                const CellGrid& grid) {
  const float start_z = bounds.maximum.z + 1.0f;
  const float pitch_x = grid.width_m + grid.gap_m;
  const float pitch_y = grid.height_m + grid.gap_m;
  std::vector<Cell> cells;
  cells.reserve(static_cast<std::size_t>(grid.rows) * grid.columns);
  for (std::uint32_t row = 0; row < grid.rows; ++row) {
    const float y = grid.center_y_m +
                    (static_cast<float>(row) -
                     0.5f * static_cast<float>(grid.rows - 1U)) * pitch_y;
    for (std::uint32_t column = 0; column < grid.columns; ++column) {
      const float x = grid.center_x_m +
                      (static_cast<float>(column) -
                       0.5f * static_cast<float>(grid.columns - 1U)) * pitch_x;
      const auto hit = top_hit(scene, x, y, start_z);
      if (!hit) continue;
      const Vec3 normal = estimated_normal(scene, x, y, start_z,
                                           grid.normal_sample_distance_m, *hit);
      if (normal.z < grid.minimum_upward_normal_z) continue;
      const auto [u, v] = tangent_axes(normal);
      cells.push_back({row,
                       column,
                       hit->point + normal * grid.surface_offset_m,
                       normal,
                       u,
                       v,
                       grid.width_m,
                       grid.height_m,
                       grid.active_area_m2});
    }
  }
  return cells;
}

SimulationSummary simulate_cells(std::vector<Cell>& cells, const TraceScene& scene,
                                 const Sun& sun,
                                 const SimulationSettings& settings) {
  if (cells.empty()) return {};
  const Integrator integrator(scene, sun, settings);
  const std::uint32_t hardware = std::max(1U, std::thread::hardware_concurrency());
  const std::uint32_t thread_count =
      std::max(1U, settings.worker_threads == 0 ? hardware
                                                : settings.worker_threads);
  std::atomic<std::size_t> next{0};
  std::vector<std::thread> workers;
  workers.reserve(thread_count);
  for (std::uint32_t thread = 0; thread < thread_count; ++thread) {
    workers.emplace_back([&] {
      while (true) {
        const std::size_t index = next.fetch_add(1, std::memory_order_relaxed);
        if (index >= cells.size()) break;
        integrator.evaluate(cells[index]);
      }
    });
  }
  for (auto& worker : workers) worker.join();

  SimulationSummary result;
  result.cell_count = cells.size();
  for (const Cell& cell : cells) {
    result.active_area_m2 += cell.active_area_m2;
    result.incident_solar_power_w += cell.total_w_m2() * cell.active_area_m2;
  }
  result.area_weighted_irradiance_w_m2 =
      result.incident_solar_power_w / result.active_area_m2;
  return result;
}

std::vector<float> simulate_top_shell_irradiance(
    const TriangleMesh& mesh, const TraceScene& scene, const Sun& sun,
    const SimulationSettings& settings) {
  const std::size_t triangle_count = mesh.triangle_count();
  std::vector<float> irradiance(triangle_count, -1.0f);
  if (triangle_count == 0) return irradiance;

  const float start_z = mesh.bounds.maximum.z + 1.0f;
  const float envelope_tolerance =
      std::max(0.002f, mesh.bounds.extent().z * 1.0e-5f);
  const std::uint32_t hardware =
      std::max(1U, std::thread::hardware_concurrency());
  const std::uint32_t thread_count =
      std::max(1U, settings.worker_threads == 0 ? hardware
                                                : settings.worker_threads);
  constexpr std::size_t kChunkSize = 1024U;
  std::atomic<std::size_t> next{0};
  std::vector<std::thread> workers;
  workers.reserve(thread_count);

  for (std::uint32_t thread = 0; thread < thread_count; ++thread) {
    workers.emplace_back([&] {
      while (true) {
        const std::size_t begin =
            next.fetch_add(kChunkSize, std::memory_order_relaxed);
        if (begin >= triangle_count) break;
        const std::size_t end = std::min(begin + kChunkSize, triangle_count);
        for (std::size_t triangle = begin; triangle < end; ++triangle) {
          const std::size_t first = triangle * 3U;
          const Vec3 edge_a =
              mesh.vertices[first + 1U] - mesh.vertices[first];
          const Vec3 edge_b =
              mesh.vertices[first + 2U] - mesh.vertices[first];
          const Vec3 raw_normal = cross(edge_a, edge_b);
          if (raw_normal.z <= 0.0f) continue;
          const Vec3 center = (mesh.vertices[first] + mesh.vertices[first + 1U] +
                               mesh.vertices[first + 2U]) /
                              3.0f;
          const auto top = top_hit(scene, center.x, center.y, start_z);
          if (!top || std::abs(top->point.z - center.z) > envelope_tolerance ||
              top->normal.z <= 0.05f) {
            continue;
          }

          const float cosine =
              std::max(0.0f, dot(top->normal, sun.direction_to_sun));
          float direct = 0.0f;
          const Vec3 origin =
              top->point + top->normal * settings.ray_epsilon_m;
          if (cosine > 0.0f &&
              scene.visible(origin, sun.direction_to_sun,
                            settings.ray_epsilon_m, 1.0e30f, kCarMask)) {
            direct = sun.dni_w_m2 * cosine;
          }
          const float sky_view =
              std::clamp((1.0f + top->normal.z) * 0.5f, 0.0f, 1.0f);
          irradiance[triangle] = direct + sun.dhi_w_m2 * sky_view;
        }
      }
    });
  }
  for (auto& worker : workers) worker.join();
  return irradiance;
}

void write_cell_csv(const std::filesystem::path& path,
                    const std::vector<Cell>& cells) {
  if (!path.parent_path().empty())
    std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  if (!output) throw std::runtime_error("Cannot write CSV: " + path.string());
  output << "row,column,x_m,y_m,z_m,nx,ny,nz,direct_w_m2,diffuse_sky_w_m2,"
            "scene_reflected_w_m2,total_w_m2,active_area_m2,incident_w\n";
  output << std::setprecision(9);
  for (const Cell& cell : cells) {
    output << cell.row << ',' << cell.column << ',' << cell.center.x << ','
           << cell.center.y << ',' << cell.center.z << ',' << cell.normal.x
           << ',' << cell.normal.y << ',' << cell.normal.z << ','
           << cell.direct_w_m2 << ',' << cell.diffuse_sky_w_m2 << ','
           << cell.scene_reflected_w_m2 << ',' << cell.total_w_m2() << ','
           << cell.active_area_m2 << ','
           << cell.total_w_m2() * cell.active_area_m2 << '\n';
  }
}

}  // namespace solar
