#include "solar/optimizer.hpp"
#include "solar/placement_surface.hpp"

#include <Highs.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace solar {
namespace {

bool in_bounds(const Vec3& point, const Vec3& minimum, const Vec3& maximum) {
  return point.x >= minimum.x && point.x <= maximum.x &&
         point.y >= minimum.y && point.y <= maximum.y &&
         point.z >= minimum.z && point.z <= maximum.z;
}

std::optional<Hit> top_hit(const TraceScene& scene, float x, float y,
                           float start_z) {
  return scene.intersect({x, y, start_z}, {0.0f, 0.0f, -1.0f}, 1.0e-6f,
                         1.0e30f, kCarMask);
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

std::array<Vec3, 4> projected_corners(const Cell& cell);
bool polygon_intersects_box(std::array<Vec3, 4> polygon, const Vec3& minimum,
                            const Vec3& maximum);

std::optional<Cell> supported_cell(const TraceScene& scene, float x, float y,
                                   float start_z, const CellGrid& grid,
                                   const ShellRegion& region, float yaw,
                                   bool follow_xy_heading = false) {
  const auto hit = top_hit(scene, x, y, start_z);
  if (!hit || !in_bounds(hit->point, region.minimum, region.maximum) ||
      (region.use_canopy_exclusion &&
       in_bounds(hit->point, region.exclusion_minimum,
                 region.exclusion_maximum)) ||
      hit->normal.z < std::max(grid.minimum_upward_normal_z,
                               region.minimum_upward_normal_z))
    return std::nullopt;

  const auto [base_u, base_v] = tangent_axes(hit->normal);
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  Vec3 u = normalized(base_u * c + base_v * s);
  Vec3 v = normalized(-base_u * s + base_v * c);
  if (follow_xy_heading) {
    // Lift the contour's XY heading into the cell's local tangent plane.
    v = normalized({-s, c, (hit->normal.x * s - hit->normal.y * c) / hit->normal.z});
    u = normalized(cross(v, hit->normal));
  }
  std::array<Hit, 9> supports{};
  float maximum_signed_support_deviation = 0.0f;
  for (int vi = -1; vi <= 1; ++vi) {
    for (int ui = -1; ui <= 1; ++ui) {
      const Vec3 point = hit->point + u * (static_cast<float>(ui) *
                                                   grid.width_m * 0.5f) +
                         v * (static_cast<float>(vi) * grid.height_m * 0.5f);
      const auto support = ui == 0 && vi == 0
                               ? hit
                               : top_hit(scene, point.x, point.y, start_z);
      if (!support ||
          !in_bounds(support->point, region.minimum, region.maximum) ||
          (region.use_canopy_exclusion &&
           in_bounds(support->point, region.exclusion_minimum,
                     region.exclusion_maximum)) ||
          support->normal.z < std::max(grid.minimum_upward_normal_z,
                                       region.minimum_upward_normal_z) ||
          std::abs(dot(support->point - hit->point, hit->normal)) >
              grid.maximum_support_plane_deviation_m ||
          dot(support->normal, hit->normal) < 0.75f)
        return std::nullopt;
      maximum_signed_support_deviation = std::max(
          maximum_signed_support_deviation,
          dot(support->point - hit->point, hit->normal));
      supports[static_cast<std::size_t>((vi + 1) * 3 + ui + 1)] = *support;
    }
  }
  Cell cell;
  cell.center = hit->point +
                hit->normal * (grid.surface_offset_m +
                               maximum_signed_support_deviation);
  cell.normal = hit->normal;
  cell.u_axis = u;
  cell.v_axis = v;
  cell.width_m = grid.width_m;
  cell.height_m = grid.height_m;
  cell.active_area_m2 = grid.active_area_m2;
  const auto corners = projected_corners(cell);
  for (const Vec3& corner : corners)
    if (!in_bounds(corner, region.minimum, region.maximum))
      return std::nullopt;
  if (region.use_canopy_exclusion &&
      polygon_intersects_box(corners, region.exclusion_minimum,
                             region.exclusion_maximum))
    return std::nullopt;
  return cell;
}

using PointKey = std::tuple<int, int, int, int, int, int>;
PointKey cell_key(const Cell& cell) {
  constexpr double scale = 100000.0;
  return {static_cast<int>(std::lround(cell.center.x * scale)),
          static_cast<int>(std::lround(cell.center.y * scale)),
          static_cast<int>(std::lround(cell.center.z * scale)),
          static_cast<int>(std::lround(cell.u_axis.x * scale)),
          static_cast<int>(std::lround(cell.u_axis.y * scale)),
          static_cast<int>(std::lround(cell.u_axis.z * scale))};
}

std::array<Vec3, 4> projected_corners(const Cell& cell) {
  const Vec3 u = cell.u_axis * (cell.width_m * 0.5f);
  const Vec3 v = cell.v_axis * (cell.height_m * 0.5f);
  return {cell.center - u - v, cell.center + u - v,
          cell.center + u + v, cell.center - u + v};
}

bool polygon_intersects_box(std::array<Vec3, 4> polygon, const Vec3& minimum,
                            const Vec3& maximum) {
  struct Plane {
    int axis;
    float bound;
    bool keep_greater;
  };
  const std::array<Plane, 6> planes{{
      {0, minimum.x, true}, {0, maximum.x, false},
      {1, minimum.y, true}, {1, maximum.y, false},
      {2, minimum.z, true}, {2, maximum.z, false}}};
  auto coordinate = [](const Vec3& point, int axis) {
    return axis == 0 ? point.x : axis == 1 ? point.y : point.z;
  };
  std::vector<Vec3> clipped(polygon.begin(), polygon.end());
  for (const Plane& plane : planes) {
    auto inside = [&](const Vec3& point) {
      return plane.keep_greater
                 ? coordinate(point, plane.axis) >= plane.bound
                 : coordinate(point, plane.axis) <= plane.bound;
    };
    if (clipped.empty()) return false;
    std::vector<Vec3> output;
    output.reserve(clipped.size() + 2U);
    Vec3 previous = clipped.back();
    bool previous_inside = inside(previous);
    for (const Vec3& current : clipped) {
      const bool current_inside = inside(current);
      if (current_inside != previous_inside) {
        const float previous_coordinate = coordinate(previous, plane.axis);
        const float denominator = coordinate(current, plane.axis) -
                                  previous_coordinate;
        if (std::abs(denominator) > 1.0e-12f) {
          const float t = (plane.bound - previous_coordinate) / denominator;
          output.push_back(previous + (current - previous) * t);
        }
      }
      if (current_inside) output.push_back(current);
      previous = current;
      previous_inside = current_inside;
    }
    clipped = std::move(output);
  }
  return !clipped.empty();
}

bool overlaps(const Cell& a, const Cell& b, float gap) {
  const auto ca = projected_corners(a);
  const auto cb = projected_corners(b);
  const auto perpendicular_xy = [](const Vec3& axis) {
    const float magnitude = std::hypot(axis.x, axis.y);
    if (magnitude <= 1.0e-8f) return Vec3{0.0f, 0.0f, 0.0f};
    return Vec3{-axis.y / magnitude, axis.x / magnitude, 0.0f};
  };
  const std::array<Vec3, 4> axes{
      perpendicular_xy(a.u_axis), perpendicular_xy(a.v_axis),
      perpendicular_xy(b.u_axis), perpendicular_xy(b.v_axis)};
  for (const Vec3& axis : axes) {
    if (length(axis) < 0.5f) continue;
    float amin = std::numeric_limits<float>::infinity();
    float amax = -amin;
    float bmin = amin;
    float bmax = -amin;
    for (const Vec3& p : ca) {
      const float projection = p.x * axis.x + p.y * axis.y;
      amin = std::min(amin, projection);
      amax = std::max(amax, projection);
    }
    for (const Vec3& p : cb) {
      const float projection = p.x * axis.x + p.y * axis.y;
      bmin = std::min(bmin, projection);
      bmax = std::max(bmax, projection);
    }
    constexpr float kGapComparisonToleranceM = 1.0e-6f;
    if (amax + gap <= bmin + kGapComparisonToleranceM ||
        bmax + gap <= amin + kGapComparisonToleranceM)
      return false;
  }
  return true;
}

bool modules_conflict(const ModuleCandidate& a, const ModuleCandidate& b,
                      float gap) {
  for (const Cell& ca : a.cells)
    for (const Cell& cb : b.cells)
      if (overlaps(ca, cb, gap)) return true;
  return false;
}

void validate_settings(const OptimizationSettings& settings) {
  if (!std::isfinite(settings.max_active_area_m2) ||
      !std::isfinite(settings.position_step_m) ||
      !std::isfinite(settings.angle_step_degrees) ||
      !std::isfinite(settings.cell_gap_m) ||
      !std::isfinite(settings.efficiency) ||
      !(settings.max_active_area_m2 > 0.0) ||
      !(settings.position_step_m > 0.0) ||
      !(settings.angle_step_degrees > 0.0 &&
        settings.angle_step_degrees <= 180.0) ||
      !(settings.cell_gap_m >= 0.0) ||
      !(settings.efficiency > 0.0 && settings.efficiency <= 1.0))
    throw std::invalid_argument("Invalid optimization settings");
}

}  // namespace

OptimizationSettings optimization_settings(const RaceConfig& config) {
  OptimizationSettings settings;
  settings.max_active_area_m2 = config.max_active_area_m2;
  settings.position_step_m = config.position_step_m;
  settings.angle_step_degrees = config.angle_step_degrees;
  settings.cell_gap_m = 0.003;
  settings.efficiency = config.cell_efficiency;
  validate_settings(settings);
  return settings;
}

void write_optimized_race(const std::filesystem::path& path,
                          const RaceConfig& config, const RaceRun& race,
                          const OptimizedLayout& result) {
  if (!path.parent_path().empty())
    std::filesystem::create_directories(path.parent_path());
  nlohmann::json output;
  output["config"] = {{"track_path", config.track_path.string()},
                      {"date", config.date},
                      {"start_local", config.start_local},
                      {"end_local", config.end_local},
                      {"timezone", config.timezone},
                      {"utc_offset_seconds", config.utc_offset_seconds},
                      {"speed_mph", config.speed_mph},
                      {"start_distance_m", config.start_distance_m},
                      {"max_active_area_m2", config.max_active_area_m2},
                      {"cell_efficiency", config.cell_efficiency},
                      {"track_step_m", config.track_step_m},
                      {"position_step_m", config.position_step_m},
                      {"angle_step_degrees", config.angle_step_degrees}};
  output["race"] = {{"duration_seconds", race.duration_seconds},
                    {"distance_m", race.distance_m},
                    {"laps", race.laps},
                    {"weather_source", race.weather_source},
                    {"sample_count", race.samples.size()},
                    {"start_origin_caveat",
                     "Track distance is relative to the track reference origin, "
                     "not a verified start/finish line."}};
  output["optimization"] = {
      {"candidate_modules", result.candidate_modules},
      {"optical_energy_wh", result.optical_energy_wh},
      {"electrical_energy_wh", result.electrical_energy_wh},
      {"mean_electrical_power_w", result.mean_electrical_power_w},
      {"energy_upper_bound_wh", result.energy_upper_bound_wh},
      {"energy_upper_bound_type", "optical incident energy"},
      {"relative_gap", result.relative_gap},
      {"proven_optimal", result.proven_optimal},
      {"status", result.status},
      {"individual_cells", result.layout.summary.individual_cells},
      {"one_by_three_modules", result.layout.summary.one_by_three_modules},
      {"two_by_three_modules", result.layout.summary.two_by_three_modules}};
  output["cells"] = nlohmann::json::array();
  for (const Cell& cell : result.layout.cells) {
    const double optical_wh = cell.total_w_m2() * cell.active_area_m2 *
                              race.duration_seconds / 3600.0;
    output["cells"].push_back(
        {{"module_id", cell.module_id},
         {"module_rows", cell.module_rows},
         {"module_columns", cell.module_columns},
         {"center_m", {cell.center.x, cell.center.y, cell.center.z}},
         {"normal", {cell.normal.x, cell.normal.y, cell.normal.z}},
         {"u_axis", {cell.u_axis.x, cell.u_axis.y, cell.u_axis.z}},
         {"v_axis", {cell.v_axis.x, cell.v_axis.y, cell.v_axis.z}},
         {"active_area_m2", cell.active_area_m2},
         {"direct_visibility_fraction", cell.direct_visibility_fraction},
         {"mean_direct_w_m2", cell.direct_w_m2},
         {"mean_diffuse_sky_w_m2", cell.diffuse_sky_w_m2},
         {"mean_scene_reflected_w_m2", cell.scene_reflected_w_m2},
         {"optical_energy_wh", optical_wh},
         {"electrical_energy_wh", optical_wh * config.cell_efficiency}});
  }
  std::ofstream file(path);
  if (!file) throw std::runtime_error("Cannot write optimized race: " + path.string());
  file << std::setw(2) << output << '\n';
  file.flush();
  if (!file) throw std::runtime_error("Failed writing optimized race: " + path.string());
}

std::vector<ModuleCandidate> generate_module_candidates(
    const TraceScene& placement_scene, const Bounds& bounds,
    const CellGrid& grid, const ShellRegion& region,
    const OptimizationSettings& settings) {
  validate_settings(settings);
  if (grid.width_m <= 0.0f || grid.height_m <= 0.0f ||
      grid.active_area_m2 <= 0.0f)
    throw std::invalid_argument("Cell dimensions and active area must be positive");
  const double module_area = grid.active_area_m2 * 3.0;
  std::vector<ModuleCandidate> result;
  if (module_area > settings.max_active_area_m2) return result;

  const float pitch = static_cast<float>(settings.position_step_m);
  const double anchors_x = std::ceil((bounds.maximum.x - bounds.minimum.x) / settings.position_step_m) + 1;
  const double anchors_y = std::ceil((bounds.maximum.y - bounds.minimum.y) / settings.position_step_m) + 1;
  const double angles = std::ceil(180.0 / settings.angle_step_degrees);
  if (!std::isfinite(anchors_x * anchors_y * angles) ||
      anchors_x * anchors_y * angles > 2000000 ||
      bounds.minimum.x + pitch == bounds.minimum.x ||
      bounds.minimum.y + pitch == bounds.minimum.y)
    throw std::invalid_argument("Placement resolution is too fine; increase position or angle step");
  const float start_z = bounds.maximum.z + 1.0f;
  const float gap = grid.gap_m;
  const int yaw_count = std::max(1, static_cast<int>(std::ceil(
      180.0 / settings.angle_step_degrees)));
  std::set<std::vector<PointKey>> unique;
  for (float y = bounds.minimum.y; y <= bounds.maximum.y; y += pitch) {
    for (float x = bounds.minimum.x; x <= bounds.maximum.x; x += pitch) {
      for (int yaw_index = 0; yaw_index < yaw_count; ++yaw_index) {
        const float yaw = static_cast<float>(yaw_index) *
                          static_cast<float>(settings.angle_step_degrees) *
                          kPi / 180.0f;
        for (std::uint32_t columns : {1U, 2U}) {
          if (columns == 2U && module_area * 2.0 >
                                   settings.max_active_area_m2 + 1.0e-9)
            continue;
          const float cell_pitch_u = grid.width_m + gap;
          const float cell_pitch_v = grid.height_m + gap;
          const float c = std::cos(yaw), s = std::sin(yaw);
          const Vec3 center_u{c, s, 0.0f};
          const Vec3 center_v{-s, c, 0.0f};
          ModuleCandidate candidate;
          bool supported = true;
          std::vector<PointKey> key;
          for (std::uint32_t row = 0; row < 3U && supported; ++row) {
            for (std::uint32_t column = 0; column < columns; ++column) {
              const float du = (static_cast<float>(column) -
                                0.5f * static_cast<float>(columns - 1U)) *
                               cell_pitch_u;
              const float dv = (static_cast<float>(row) - 1.0f) * cell_pitch_v;
              const float cx = x + center_u.x * du + center_v.x * dv;
              const float cy = y + center_u.y * du + center_v.y * dv;
              auto cell = supported_cell(placement_scene, cx, cy, start_z,
                                         grid, region, yaw);
              if (!cell) {
                supported = false;
                break;
              }
              candidate.cells.push_back(*cell);
              key.push_back(cell_key(*cell));
            }
          }
          if (!supported) continue;
          for (std::size_t first = 0; first < candidate.cells.size() && supported;
               ++first) {
            for (std::size_t second = first + 1U;
                 second < candidate.cells.size(); ++second) {
              if (overlaps(candidate.cells[first], candidate.cells[second],
                           static_cast<float>(settings.cell_gap_m))) {
                supported = false;
                break;
              }
            }
          }
          if (!supported) continue;
          std::sort(key.begin(), key.end());
          if (unique.insert(key).second) result.push_back(std::move(candidate));
        }
      }
    }
  }
  return result;
}

std::vector<std::size_t> group_track_profiles(
    const CellTrackProfiles& profiles, std::size_t zone_count, float cell_gap_m) {
  if ((zone_count != 2 && zone_count != 3) || profiles.race.samples.empty() ||
      profiles.irradiance_w_m2.size() != profiles.race.samples.size() ||
      !std::isfinite(cell_gap_m) || cell_gap_m < 0)
    throw std::invalid_argument("Invalid track profiles, zone count, or cell gap");
  std::map<std::uint32_t, std::size_t> module_index;
  std::vector<std::vector<std::size_t>> modules;
  for (std::size_t i = 0; i < profiles.cells.size(); ++i) {
    const auto id = profiles.cells[i].module_id;
    if (!id) throw std::invalid_argument("Every cell must have a module ID");
    const auto [it, inserted] = module_index.emplace(id, modules.size());
    if (inserted) modules.emplace_back();
    modules[it->second].push_back(i);
  }
  if (modules.size() < zone_count) throw std::invalid_argument("Not enough modules for the requested zones");
  const std::size_t n = modules.size(), count = profiles.race.samples.size();
  std::vector<std::vector<double>> curves(n, std::vector<double>(count));
  double total_duration = 0;
  for (std::size_t t = 0; t < count; ++t) {
    const auto dt = profiles.race.samples[t].duration_seconds;
    if (!(dt > 0) || !std::isfinite(dt) || profiles.irradiance_w_m2[t].size() != profiles.cells.size())
      throw std::invalid_argument("Invalid track profile dimensions or sample duration");
    total_duration += dt;
    for (std::size_t i = 0; i < n; ++i) {
      for (const auto c : modules[i]) {
        const auto value = profiles.irradiance_w_m2[t][c];
        if (!std::isfinite(value) || value < 0) throw std::invalid_argument("Invalid irradiance in track profile");
        curves[i][t] += value;
      }
      curves[i][t] /= static_cast<double>(modules[i].size());
    }
  }
  const auto point_segment_distance = [](const Vec3& p, const Vec3& a, const Vec3& b) {
    const double x = b.x - a.x, y = b.y - a.y;
    const double len2 = x*x + y*y;
    const double u = len2 > 0 ? std::clamp(((p.x-a.x)*x+(p.y-a.y)*y)/len2, 0.0, 1.0) : 0.0;
    return std::hypot(p.x-a.x-u*x, p.y-a.y-u*y);
  };
  const auto nearby = [&](const Cell& a, const Cell& b) {
    const auto ac = projected_corners(a), bc = projected_corners(b);
    double minimum = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
      minimum = std::min(minimum, point_segment_distance(ac[i], bc[j], bc[(j+1)%4]));
      minimum = std::min(minimum, point_segment_distance(bc[j], ac[i], ac[(i+1)%4]));
    }
    return minimum <= static_cast<double>(cell_gap_m) + 1.0e-5;
  };
  std::vector<std::vector<std::size_t>> neighbors(n);
  for (std::size_t i = 0; i < n; ++i) for (std::size_t j = 0; j < i; ++j) {
    bool adjacent = false;
    for (auto a : modules[i]) { for (auto b : modules[j])
      if (nearby(profiles.cells[a], profiles.cells[b])) { adjacent = true; break; }
      if (adjacent) break;
    }
    if (adjacent) { neighbors[i].push_back(j); neighbors[j].push_back(i); }
  }
  std::vector<bool> visited(n);
  std::vector<std::size_t> stack{0}; visited[0] = true;
  while (!stack.empty()) { const auto i = stack.back(); stack.pop_back();
    for (auto j : neighbors[i]) if (!visited[j]) { visited[j] = true; stack.push_back(j); }
  }
  if (std::find(visited.begin(), visited.end(), false) != visited.end())
    throw std::runtime_error("Packed modules form disconnected islands at the configured cell gap; contiguous zones cannot be formed");
  const auto difference = [&](const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0;
    for (std::size_t t = 0; t < count; ++t) {
      const double d = a[t] - b[t];
      sum += profiles.race.samples[t].duration_seconds * d*d;
    }
    return sum / total_duration;
  };
  const auto unassigned = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> assignments(n, unassigned), seeds{0};
  assignments[0] = 0;
  while (seeds.size() < zone_count) {
    std::size_t best = unassigned; double best_distance = -1;
    for (std::size_t i = 0; i < n; ++i) if (assignments[i] == unassigned) {
      double nearest = std::numeric_limits<double>::infinity();
      for (auto seed : seeds) nearest = std::min(nearest, difference(curves[i], curves[seed]));
      if (nearest > best_distance) { best_distance = nearest; best = i; }
    }
    assignments[best] = seeds.size(); seeds.push_back(best);
  }
  std::vector<std::vector<double>> means(zone_count, std::vector<double>(count));
  std::vector<std::size_t> zone_cells(zone_count);
  for (std::size_t z = 0; z < zone_count; ++z) { means[z] = curves[seeds[z]]; zone_cells[z] = modules[seeds[z]].size(); }
  for (std::size_t remaining = n - zone_count; remaining; --remaining) {
    std::size_t best = unassigned, best_zone = unassigned; double cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) if (assignments[i] == unassigned)
      for (auto j : neighbors[i]) if (assignments[j] != unassigned) {
        const auto z = assignments[j];
        const auto value = difference(curves[i], means[z]);
        if (value < cost) { cost = value; best = i; best_zone = z; }
      }
    if (best == unassigned) throw std::runtime_error("Unable to grow contiguous zones");
    assignments[best] = best_zone;
    const auto old_count = zone_cells[best_zone], added = modules[best].size();
    for (std::size_t t = 0; t < count; ++t)
      means[best_zone][t] = (means[best_zone][t]*old_count + curves[best][t]*added) / (old_count + added);
    zone_cells[best_zone] += added;
  }
  std::vector<std::size_t> result(profiles.cells.size());
  for (std::size_t i = 0; i < n; ++i) for (auto cell : modules[i]) result[cell] = assignments[i];
  return result;
}

CellTrackProfiles sample_cell_track_profiles(
    const std::vector<Cell>& cells, const TraceScene& scene,
    const RaceRun& race, const SimulationSettings& settings,
    const OptimizationProgressCallback& progress,
    const OptimizationStopCallback& stop) {
  if (cells.empty() || race.samples.empty())
    throw std::invalid_argument("Track profiles require cells and race samples");
  CellTrackProfiles result;
  result.cells = cells;
  result.race = race;
  result.irradiance_w_m2.reserve(race.samples.size());
  std::vector<Cell> batch = cells;
  if (progress) progress({OptimizationPhase::scoring, 0, race.samples.size()});
  for (const auto& sample : race.samples) {
    if (!std::isfinite(sample.duration_seconds) || sample.duration_seconds <= 0.0)
      throw std::invalid_argument("Track sample duration must be finite and positive");
    if (stop && stop()) throw std::runtime_error("Track profiling stopped; no partial profiles retained");
    simulate_cells(batch, scene, sample.sun, settings);
    auto& row = result.irradiance_w_m2.emplace_back();
    row.reserve(batch.size());
    for (const auto& cell : batch) row.push_back(cell.total_w_m2());
    if (progress) progress({OptimizationPhase::scoring, result.irradiance_w_m2.size(), race.samples.size()});
  }
  return result;
}

GeneratedLayout pack_shell_perimeter_inward(
    const TraceScene& placement_scene, const Bounds& bounds,
    const TraceScene& complete_trace_scene, const CellGrid& grid,
    const ShellRegion& region, const OptimizationSettings& settings) {
  const float pitch_u = grid.width_m + grid.gap_m;
  const float pitch_v = grid.height_m + grid.gap_m;
  const float step = static_cast<float>(std::min({settings.position_step_m,
      static_cast<double>(grid.width_m), static_cast<double>(grid.height_m)}) / 8.0);
  const float start_z = bounds.maximum.z + 1.0f;
  const double x_count = std::ceil(bounds.extent().x / step) + 3;
  const double y_count = std::ceil(bounds.extent().y / step) + 3;
  if (!(step > 0) || !(grid.width_m > 0) || !(grid.height_m > 0) ||
      !std::isfinite(grid.gap_m) || grid.gap_m < 0 ||
      !(x_count >= 3 && y_count >= 3 && x_count * y_count <= 2000000) ||
      !std::isfinite(settings.angle_step_degrees) || !(settings.angle_step_degrees > 0))
    throw std::invalid_argument("Invalid shell bounds or packing resolution; increase position step");
  const int nx = static_cast<int>(x_count), ny = static_cast<int>(y_count);
  const auto index = [nx](int x, int y) { return y * nx + x; };
  const Vec3 origin = bounds.minimum - Vec3{step, step, 0};
  std::vector<float> depth(static_cast<std::size_t>(nx) * ny, 0.0f);
  // Sample the projected eligible footprint, including openings. The distance
  // field only guides placement; the 3D mesh still decides whether cells fit.
  for (int y = 1; y < ny - 1; ++y) for (int x = 1; x < nx - 1; ++x) {
    const auto hit = top_hit(placement_scene, origin.x + x*step, origin.y + y*step, start_z);
    if (hit && in_bounds(hit->point, region.minimum, region.maximum) &&
        hit->normal.z >= std::max(grid.minimum_upward_normal_z, region.minimum_upward_normal_z) &&
        (!region.use_canopy_exclusion || !in_bounds(hit->point, region.exclusion_minimum, region.exclusion_maximum)))
      depth[index(x,y)] = std::numeric_limits<float>::infinity();
  }
  const float diagonal = step * std::sqrt(2.0f);
  for (int sign : {1, -1}) {
    for (int iy = 1; iy < ny - 1; ++iy) for (int ix = 1; ix < nx - 1; ++ix) {
      const int x = sign > 0 ? ix : nx-1-ix, y = sign > 0 ? iy : ny-1-iy;
      auto& d = depth[index(x,y)];
      d = std::min({d, depth[index(x-sign,y)] + step, depth[index(x,y-sign)] + step,
          depth[index(x-1,y-sign)] + diagonal, depth[index(x+1,y-sign)] + diagonal});
    }
  }
  GeneratedLayout result;
  struct Placement { Bounds footprint; ModuleCandidate module; };
  std::vector<Placement> selected;
  const auto place = [&](Vec3 anchor, float yaw, std::uint32_t columns) {
    const Vec3 u{std::cos(yaw), std::sin(yaw), 0}, v{-u.y, u.x, 0};
    ModuleCandidate candidate;
    Bounds footprint;
    for (int row = 0; row < 3; ++row) for (std::uint32_t col = 0; col < columns; ++col) {
      const Vec3 p = anchor + u * ((col - 0.5f*(columns-1))*pitch_u) + v * ((row-1)*pitch_v);
      auto cell = supported_cell(placement_scene, p.x, p.y, start_z, grid, region, yaw, true);
      if (!cell || !conform_shell_cell(*cell, placement_scene, complete_trace_scene, grid)) return false;
      for (const auto& other : candidate.cells)
        if (overlaps(*cell, other, grid.gap_m)) return false;
      for (const auto& corner : projected_corners(*cell)) footprint.include(corner);
      candidate.cells.push_back(*cell);
    }
    for (const auto& prior : selected) {
      // Skip module pairs that are already farther apart than the gap in X or Y.
      const float pad = 2*grid.gap_m;
      if (footprint.maximum.x + pad < prior.footprint.minimum.x ||
          prior.footprint.maximum.x + pad < footprint.minimum.x ||
          footprint.maximum.y + pad < prior.footprint.minimum.y ||
          prior.footprint.maximum.y + pad < footprint.minimum.y) continue;
      if (modules_conflict(candidate, prior.module, grid.gap_m)) return false;
    }
    const auto id = static_cast<std::uint32_t>(selected.size() + 1);
    for (std::size_t i = 0; i < candidate.cells.size(); ++i) {
      auto& cell = candidate.cells[i];
      cell.module_id = id; cell.module_rows = 3; cell.module_columns = columns;
      cell.row = static_cast<std::uint32_t>(i / columns);
      cell.column = static_cast<std::uint32_t>(i % columns);
      result.cells.push_back(cell);
    }
    if (columns == 2) ++result.summary.two_by_three_modules;
    else ++result.summary.one_by_three_modules;
    selected.push_back({footprint, std::move(candidate)});
    return true;
  };
  // The supplied shell has a broad rectangular tail and a narrow rounded nose.
  // Compare near-end widths so the walk starts at the tail even if Y is flipped.
  const int end_rows = std::clamp(static_cast<int>(std::ceil(std::min(pitch_v/step, static_cast<float>(ny-2)))), 1, ny-2);
  const auto end_width = [&](bool high) {
    int widest = 0;
    for (int k = 1; k <= end_rows; ++k) {
      const int y = high ? ny-1-k : k;
      int left = nx, right = -1;
      for (int x = 1; x < nx-1; ++x) if (depth[index(x,y)] > 0) { left = std::min(left, x); right = x; }
      widest = std::max(widest, right-left+1);
    }
    return widest;
  };
  const bool rear_at_high_y = end_width(true) >= end_width(false);
  const float yaw_adjustment = static_cast<float>(std::min(settings.angle_step_degrees, 15.0)) * kPi / 180.0f;
  const auto fill_contour = [&](float inset, std::uint32_t columns) {
    // Extract oriented edges of the inset footprint. Interior is on the left.
    // Scan from the rear, low X first, to seed the walk at the rear-left end.
    const float threshold = inset + step * 0.5f;
    const auto inside = [&](int x, int y) { return depth[index(x,y)] >= threshold; };
    constexpr std::array<int, 4> dx{1, 0, -1, 0}, dy{0, 1, 0, -1};
    std::vector<unsigned char> edges(depth.size());
    for (int y = 1; y < ny-1; ++y) for (int x = 1; x < nx-1; ++x) {
      if (!inside(x,y)) continue;
      if (!inside(x,y-1)) edges[index(x,y)] |= 1U;
      if (!inside(x+1,y)) edges[index(x+1,y)] |= 2U;
      if (!inside(x,y+1)) edges[index(x+1,y+1)] |= 4U;
      if (!inside(x-1,y)) edges[index(x,y+1)] |= 8U;
    }
    for (int row = 1; row < ny; ++row) for (int sx = 1; sx < nx; ++sx) {
      const int sy = rear_at_high_y ? ny-row : row;
      while (edges[index(sx,sy)]) {
        std::vector<Vec3> loop;
        int x = sx, y = sy, direction = 0;
        while (!(edges[index(x,y)] & (1U << direction))) ++direction;
        do {
          loop.push_back({origin.x + (x-0.5f)*step, origin.y + (y-0.5f)*step, 0});
          edges[index(x,y)] &= static_cast<unsigned char>(~(1U << direction));
          x += dx[direction]; y += dy[direction];
          const auto next = edges[index(x,y)];
          if (!next) break;
          // At diagonal contacts, follow the same component instead of crossing.
          for (int turn : {1, 0, 3, 2}) {
            const int d = (direction+turn)%4;
            if (next & (1U << d)) { direction = d; break; }
          }
        } while (x != sx || y != sy);
        const float perimeter = static_cast<float>(loop.size()) * step;
        if (x != sx || y != sy || perimeter < 3*pitch_v) continue;
        const auto point_at = [&](float s) {
          s = std::fmod(s, perimeter);
          if (s < 0) s += perimeter;
          const float f = s / step;
          const auto i = std::min(static_cast<std::size_t>(f), loop.size()-1);
          return loop[i] + (loop[(i+1)%loop.size()] - loop[i]) * std::clamp(f-static_cast<float>(i), 0.0f, 1.0f);
        };
        const float half_span = std::min(1.5f*pitch_v, perimeter*0.125f);
        for (float s = 0; s < perimeter;) {
          const Vec3 tangent = point_at(s+half_span) - point_at(s-half_span);
          if (std::hypot(tangent.x,tangent.y) < step) { s += step; continue; }
          const float heading = std::atan2(tangent.y,tangent.x);
          const Vec3 inward{-std::sin(heading), std::cos(heading), 0};
          bool placed = false;
          for (float nudge : {0.0f, 0.5f, 1.0f}) {
            for (float angle : {0.0f, -0.5f, 0.5f, -1.0f, 1.0f}) {
              if (place(point_at(s) + inward*(nudge*step), heading - kPi*0.5f + angle*yaw_adjustment, columns)) {
                placed = true; break;
              }
            }
            if (placed) break;
          }
          s += placed ? 3*pitch_v : step;
        }
      }
    }
  };
  const float max_depth = *std::max_element(depth.begin(), depth.end());
  for (float band = 0; band + grid.width_m * 0.5f < max_depth; band += 2*pitch_u) {
    fill_contour(band + pitch_u * 0.5f + grid.width_m * 0.5f, 2);
    fill_contour(band + grid.width_m * 0.5f, 1);
    fill_contour(band + pitch_u + grid.width_m * 0.5f, 1);
  }
  // Try staggered narrow contours after the main bands. Near a curve or a
  // split in the footprint, a leftover pocket need not line up with a lane.
  for (float inset = (grid.width_m + pitch_u)*0.5f; inset < max_depth; inset += pitch_u)
    fill_contour(inset, 1);
  result.summary.individual_cells = result.cells.size();
  return result;
}

OptimizedLayout solve_module_packing(
    const std::vector<ModuleCandidate>& candidates,
    const OptimizationSettings& settings,
    const OptimizationProgressCallback& progress,
    const OptimizationStopCallback& stop) {
  validate_settings(settings);
  OptimizedLayout output;
  output.candidate_modules = candidates.size();
  if (candidates.empty()) {
    output.status = "No feasible modules";
    return output;
  }
  if (progress) progress({OptimizationPhase::packing_setup});
  for (const ModuleCandidate& candidate : candidates) {
    if ((candidate.cells.size() != 3U && candidate.cells.size() != 6U) ||
        !std::isfinite(candidate.energy_wh) || candidate.energy_wh < 0.0)
      throw std::invalid_argument("Each module must contain 3 or 6 cells and have finite nonnegative energy");
    for (const Cell& cell : candidate.cells)
      if (!(cell.active_area_m2 > 0.0f) ||
          !std::isfinite(cell.active_area_m2))
        throw std::invalid_argument("Module cells need finite positive active areas");
  }

  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("mip_rel_gap", 1.0e-9);
  highs.setOptionValue("mip_abs_gap", 1.0e-9);
  HighsModel model;
  HighsLp& lp = model.lp_;
  const HighsInt n = static_cast<HighsInt>(candidates.size());
  lp.num_col_ = n;
  lp.col_cost_.resize(n);
  lp.col_lower_.assign(n, 0.0);
  lp.col_upper_.assign(n, 1.0);
  model.lp_.integrality_.assign(n, HighsVarType::kInteger);
  std::vector<double> areas(candidates.size());
  std::vector<double> energies(candidates.size());
  std::vector<double> twos(candidates.size());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];
    areas[i] = std::accumulate(candidate.cells.begin(), candidate.cells.end(), 0.0,
        [](double sum, const Cell& cell) { return sum + cell.active_area_m2; });
    energies[i] = candidate.energy_wh;
    twos[i] = candidate.cells.size() == 6 ? 1.0 : 0.0;
    lp.col_cost_[i] = -energies[i];
  }

  std::set<std::pair<HighsInt, HighsInt>> conflict_set;
  constexpr float kBucketSize = 0.25f;
  double maximum_cell_radius = 0;
  for (const auto& candidate : candidates) {
    for (const auto& cell : candidate.cells) {
      if (!std::isfinite(cell.width_m) || !std::isfinite(cell.height_m) ||
          cell.width_m <= 0 || cell.height_m <= 0)
        throw std::invalid_argument("Module cell footprints must be finite and positive");
      maximum_cell_radius = std::max(maximum_cell_radius,
          0.5 * (cell.width_m * std::hypot(cell.u_axis.x, cell.u_axis.y) +
                 cell.height_m * std::hypot(cell.v_axis.x, cell.v_axis.y)));
    }
  }
  const double bucket_reach = std::ceil((2 * maximum_cell_radius + settings.cell_gap_m) / kBucketSize);
  if (!std::isfinite(bucket_reach) || bucket_reach > 10000)
    throw std::invalid_argument("Module footprint or gap is too large for conflict search");
  const int bucket_radius = std::max(1, static_cast<int>(bucket_reach));
  std::map<std::pair<int, int>, std::vector<HighsInt>> buckets;
  for (HighsInt i = 0; i < n; ++i) {
    for (const Cell& cell : candidates[static_cast<std::size_t>(i)].cells) {
      const int bx = static_cast<int>(std::floor(cell.center.x / kBucketSize));
      const int by = static_cast<int>(std::floor(cell.center.y / kBucketSize));
      for (int dx = -bucket_radius; dx <= bucket_radius; ++dx) {
        for (int dy = -bucket_radius; dy <= bucket_radius; ++dy) {
          const auto bucket = buckets.find({bx + dx, by + dy});
          if (bucket == buckets.end()) continue;
          for (const HighsInt other : bucket->second) {
            if (other == i) continue;
            const HighsInt first = std::min(i, other);
            const HighsInt second = std::max(i, other);
            if (conflict_set.contains({first, second})) continue;
            if (modules_conflict(candidates[static_cast<std::size_t>(first)],
                                 candidates[static_cast<std::size_t>(second)],
                                 static_cast<float>(settings.cell_gap_m)))
              conflict_set.emplace(first, second);
          }
        }
      }
      buckets[{bx, by}].push_back(i);
    }
  }
  std::vector<std::pair<HighsInt, HighsInt>> conflicts(conflict_set.begin(),
                                                       conflict_set.end());
  std::vector<std::vector<HighsInt>> conflict_rows(
      static_cast<std::size_t>(n));
  for (std::size_t row = 0; row < conflicts.size(); ++row) {
    conflict_rows[static_cast<std::size_t>(conflicts[row].first)].push_back(
        static_cast<HighsInt>(row));
    conflict_rows[static_cast<std::size_t>(conflicts[row].second)].push_back(
        static_cast<HighsInt>(row));
  }
  const HighsInt area_row = static_cast<HighsInt>(conflicts.size());
  lp.num_row_ = area_row + 1;
  lp.row_lower_.assign(static_cast<std::size_t>(lp.num_row_),
                       -kHighsInf);
  lp.row_upper_.assign(static_cast<std::size_t>(lp.num_row_), kHighsInf);
  for (std::size_t row = 0; row < conflicts.size(); ++row)
    lp.row_upper_[row] = 1.0;
  lp.row_lower_[static_cast<std::size_t>(area_row)] = -kHighsInf;
  lp.row_upper_[static_cast<std::size_t>(area_row)] =
      settings.max_active_area_m2;
  lp.a_matrix_.format_ = MatrixFormat::kColwise;
  lp.a_matrix_.start_.assign(static_cast<std::size_t>(n) + 1U, 0);
  for (HighsInt col = 0; col < n; ++col) {
    std::vector<HighsInt> rows =
        conflict_rows[static_cast<std::size_t>(col)];
    rows.push_back(area_row);
    lp.a_matrix_.index_.insert(lp.a_matrix_.index_.end(), rows.begin(), rows.end());
    for (const HighsInt row : rows)
      lp.a_matrix_.value_.push_back(row == area_row ? areas[col] : 1.0);
    lp.a_matrix_.start_[static_cast<std::size_t>(col) + 1U] =
        static_cast<HighsInt>(lp.a_matrix_.index_.size());
  }
  // Always start with a feasible layout. Even if the search is interrupted,
  // do not discard this layout for a weaker solver incumbent.
  std::vector<std::size_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
    if (energies[a] != energies[b]) return energies[a] > energies[b];
    if (areas[a] != areas[b]) return areas[a] > areas[b];
    return twos[a] > twos[b];
  });
  std::vector<double> selected_values(candidates.size(), 0.0);
  std::vector<bool> blocked(candidates.size(), false);
  double used_area = 0.0;
  double best_energy = 0.0;
  double best_area = 0.0;
  double best_twos = 0.0;
  for (const auto i : order) {
    if (blocked[i]) continue;
    if (used_area + areas[i] > settings.max_active_area_m2) continue;
    selected_values[i] = 1.0;
    used_area += areas[i];
    for (const auto row : conflict_rows[i]) {
      const auto [a, b] = conflicts[static_cast<std::size_t>(row)];
      blocked[static_cast<std::size_t>(a == static_cast<HighsInt>(i) ? b : a)] = true;
    }
  }
  // Sum in the same order used for solver incumbents.
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (selected_values[i] < 0.5) continue;
    best_energy += energies[i];
    best_area += areas[i];
    best_twos += twos[i];
  }
  const double seed_energy = best_energy;
  constexpr double kAreaTolerance = 1.0e-12;
  const auto consider_solution = [&](const HighsSolution& solution, double energy_floor,
                                     double area_floor, OptimizationPhase phase) {
    if (!solution.value_valid || solution.col_value.size() != candidates.size()) return false;
    double energy = 0.0;
    double area = 0.0;
    double two_by_three_count = 0;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const double x = solution.col_value[i];
      if (!std::isfinite(x) || std::abs(x - (x >= 0.5 ? 1.0 : 0.0)) > 1.0e-6) return false;
      if (x < 0.5) continue;
      area += areas[i];
      energy += energies[i];
      two_by_three_count += twos[i];
    }
    if (area > settings.max_active_area_m2 + kAreaTolerance ||
        energy < energy_floor || area + kAreaTolerance < area_floor) return false;
    for (const auto& [a, b] : conflicts)
      if (solution.col_value[a] >= 0.5 && solution.col_value[b] >= 0.5) return false;
    const bool better_area = area > best_area + kAreaTolerance;
    const bool same_area = std::abs(area - best_area) <= kAreaTolerance;
    const bool better_grouping = same_area && two_by_three_count > best_twos;
    const bool better = phase == OptimizationPhase::solving
        ? energy > best_energy || (energy == best_energy && (better_area || better_grouping))
        : better_area || better_grouping;
    if (better) {
      selected_values = solution.col_value;
      best_energy = energy;
      best_area = area;
      best_twos = two_by_three_count;
    }
    return true;
  };
  const auto configure_search = [&](Highs& solver, OptimizationPhase phase) {
    if (!progress && !stop) return;
    solver.setCallback([&, phase](int, const std::string&, const HighsCallbackOutput* data,
                                 HighsCallbackInput* input, void*) {
      if (input && stop && stop()) input->user_interrupt = true;
      if (progress && phase == OptimizationPhase::solving && data) {
        const double best = std::isfinite(data->mip_primal_bound)
            ? std::max(best_energy, -data->mip_primal_bound) : best_energy;
        const double upper = -data->mip_dual_bound;
        const double gap = std::isfinite(upper) && upper > 0
            ? std::clamp((upper - best) / upper, 0.0, 1.0) : -1.0;
        progress({phase, 0, 0, best * settings.efficiency, gap});
      }
    }, nullptr);
    solver.startCallback(kCallbackMipInterrupt);
  };
  if (highs.passModel(model) == HighsStatus::kError)
    throw std::runtime_error("HiGHS rejected module-packing model");
  std::vector<HighsInt> columns(candidates.size());
  std::iota(columns.begin(), columns.end(), 0);
  highs.setSolution(n, columns.data(), selected_values.data());
  configure_search(highs, OptimizationPhase::solving);
  if (progress) progress({OptimizationPhase::solving, 0, 0, best_energy * settings.efficiency});
  const bool stopped_before_search = stop && stop();
  if (!stopped_before_search) highs.run();
  const auto model_status = highs.getModelStatus();
  const auto& info = highs.getInfo();
  output.energy_upper_bound_wh = -1.0;
  if (info.valid && std::isfinite(info.mip_dual_bound))
    output.energy_upper_bound_wh = std::max(0.0, -info.mip_dual_bound);
  const bool solver_result_valid = !stopped_before_search &&
      consider_solution(highs.getSolution(), seed_energy, 0.0, OptimizationPhase::solving);
  output.proven_optimal = model_status == HighsModelStatus::kOptimal && solver_result_valid;
  output.status = stopped_before_search ? "Stopped (feasible starting layout)"
                                       : highs.modelStatusToString(model_status);
  if (!stopped_before_search && !solver_result_valid) output.status += " (feasible starting layout retained)";

  if (output.proven_optimal) {
    // The small Wh tolerance allows for floating-point summation between passes.
    const double energy_floor = std::max(seed_energy,
        best_energy - 1.0e-9 * std::max(1.0, std::abs(best_energy)));
    const auto tie_break = [&](const std::vector<double>& objective, OptimizationPhase phase) {
      if (stop && stop()) return false;
      Highs solver;
      solver.setOptionValue("output_flag", false);
      solver.setOptionValue("mip_rel_gap", 1.0e-9);
      solver.setOptionValue("mip_abs_gap", 1.0e-9);
      if (solver.passModel(model) == HighsStatus::kError) return false;
      for (HighsInt i = 0; i < n; ++i) solver.changeColCost(i, -objective[i]);
      if (solver.addRow(energy_floor, kHighsInf, n, columns.data(), energies.data()) == HighsStatus::kError)
        return false;
      const bool fix_area = phase == OptimizationPhase::module_tie_break;
      const double area_floor = fix_area ? best_area - kAreaTolerance : 0.0;
      if (fix_area && solver.addRow(area_floor, kHighsInf, n, columns.data(), areas.data()) == HighsStatus::kError)
        return false;
      solver.setSolution(n, columns.data(), selected_values.data());
      configure_search(solver, phase);
      if (progress) progress({phase, 0, 0, best_energy * settings.efficiency, 0.0});
      if (stop && stop()) return false;
      solver.run();
      const bool valid = consider_solution(solver.getSolution(), energy_floor, area_floor, phase);
      return valid && solver.getModelStatus() == HighsModelStatus::kOptimal;
    };
    if (!tie_break(areas, OptimizationPhase::area_tie_break))
      output.status += " (area tie-break not proven)";
    else if (!tie_break(twos, OptimizationPhase::module_tie_break))
      output.status += " (2x3 tie-break not proven)";
  }

  std::uint32_t module_id = 1;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    if (selected_values[index] < 0.5) continue;
    const auto& candidate = candidates[index];
    const std::uint32_t module_columns = candidate.cells.size() == 6U ? 2U : 1U;
    for (std::size_t cell_index = 0; cell_index < candidate.cells.size(); ++cell_index) {
      Cell cell = candidate.cells[cell_index];
      cell.row = static_cast<std::uint32_t>(cell_index / module_columns);
      cell.column = static_cast<std::uint32_t>(cell_index % module_columns);
      cell.module_id = module_id;
      cell.module_rows = 3U;
      cell.module_columns = module_columns;
      output.layout.cells.push_back(cell);
    }
    if (module_columns == 2U) ++output.layout.summary.two_by_three_modules;
    else ++output.layout.summary.one_by_three_modules;
    output.optical_energy_wh += candidate.energy_wh;
    ++module_id;
  }
  output.layout.summary.individual_cells = output.layout.cells.size();
  output.electrical_energy_wh =
      output.optical_energy_wh * settings.efficiency;
  if (!info.valid || !std::isfinite(info.mip_dual_bound))
    output.energy_upper_bound_wh =
        output.proven_optimal ? output.optical_energy_wh : -1.0;
  output.relative_gap = output.energy_upper_bound_wh > 0.0
                            ? std::max(0.0, (output.energy_upper_bound_wh -
                                             output.optical_energy_wh) /
                                                output.energy_upper_bound_wh)
                            : (output.proven_optimal ? 0.0 : -1.0);
  return output;
}

OptimizedLayout optimize_race_layout(
    const TraceScene& placement_scene, const Bounds& bounds,
    const TraceScene& complete_trace_scene, const CellGrid& grid,
    const ShellRegion& region, const RaceRun& race,
    const SimulationSettings& simulation,
    const OptimizationSettings& settings,
    const OptimizationProgressCallback& progress,
    const OptimizationStopCallback& stop) {
  if (race.samples.empty() || !std::isfinite(race.duration_seconds) || race.duration_seconds <= 0)
    throw std::invalid_argument("Race needs positive duration and at least one timed sample");
  double duration_sum = 0;
  for (const auto& sample : race.samples) {
    if (!std::isfinite(sample.duration_seconds) || sample.duration_seconds <= 0)
      throw std::invalid_argument("Race sample duration must be finite and positive");
    duration_sum += sample.duration_seconds;
  }
  if (std::abs(duration_sum - race.duration_seconds) > 1.0e-6 * race.duration_seconds)
    throw std::invalid_argument("Race sample weights must cover the complete race duration");
  OptimizationSettings effective_settings = settings;
  effective_settings.cell_gap_m = grid.gap_m;
  if (progress) progress({OptimizationPhase::candidates});
  auto candidates = generate_module_candidates(placement_scene, bounds, grid,
                                              region, effective_settings);
  // A module is atomic: deleting individual failing cells could turn a 2x3
  // into an arbitrary three-cell shape that is not a valid 1x3 module.
  std::vector<ModuleCandidate> supported_candidates;
  supported_candidates.reserve(candidates.size());
  for (auto& candidate : candidates) {
    bool supported = true;
    for (auto& cell : candidate.cells)
      if (!conform_shell_cell(cell, placement_scene, complete_trace_scene, grid)) {
        supported = false;
        break;
      }
    if (supported) supported_candidates.push_back(std::move(candidate));
  }
  candidates = std::move(supported_candidates);
  candidates.erase(
      std::remove_if(candidates.begin(), candidates.end(),
                     [&](const ModuleCandidate& candidate) {
                       for (std::size_t first = 0;
                            first < candidate.cells.size(); ++first)
                         for (std::size_t second = first + 1U;
                              second < candidate.cells.size(); ++second)
                           if (overlaps(candidate.cells[first],
                                        candidate.cells[second],
                                        static_cast<float>(
                                            effective_settings.cell_gap_m)))
                             return true;
                       return false;
                     }),
      candidates.end());
  std::map<PointKey, Cell> unique_cells;
  for (const ModuleCandidate& candidate : candidates)
    for (const Cell& cell : candidate.cells)
      unique_cells.try_emplace(cell_key(cell), cell);
  std::map<PointKey, double> energy;
  std::map<PointKey, std::array<double, 4>> irradiance_time_integrals;
  double sampled_duration = 0.0;
  std::size_t completed_samples = 0;
  if (progress && !unique_cells.empty())
    progress({OptimizationPhase::scoring, 0, race.samples.size()});
  for (const RaceSample& sample : race.samples) {
    if (stop && stop()) throw std::runtime_error("Optimization stopped before scoring finished; no partial race totals saved");
    std::vector<Cell> batch;
    batch.reserve(unique_cells.size());
    for (const auto& [key, cell] : unique_cells) {
      (void)key;
      batch.push_back(cell);
    }
    if (batch.empty()) break;
    simulate_cells(batch, complete_trace_scene, sample.sun, simulation);
    sampled_duration += sample.duration_seconds;
    for (const Cell& cell : batch) {
      energy[cell_key(cell)] += cell.total_w_m2() * cell.active_area_m2 *
                                sample.duration_seconds / 3600.0;
      auto& integral = irradiance_time_integrals[cell_key(cell)];
      integral[0] += cell.direct_visibility_fraction * sample.duration_seconds;
      integral[1] += cell.direct_w_m2 * sample.duration_seconds;
      integral[2] += cell.diffuse_sky_w_m2 * sample.duration_seconds;
      integral[3] += cell.scene_reflected_w_m2 * sample.duration_seconds;
    }
    ++completed_samples;
    if (progress) progress({OptimizationPhase::scoring, completed_samples, race.samples.size()});
  }
  for (ModuleCandidate& candidate : candidates) {
    for (const Cell& cell : candidate.cells) candidate.energy_wh += energy[cell_key(cell)];
  }
  OptimizedLayout result = solve_module_packing(candidates, effective_settings, progress, stop);
  if (progress) progress({OptimizationPhase::finishing});
  if (sampled_duration > 0.0) {
    for (Cell& cell : result.layout.cells) {
      const auto found = irradiance_time_integrals.find(cell_key(cell));
      if (found == irradiance_time_integrals.end()) continue;
      const double scale = 1.0 / sampled_duration;
      cell.direct_visibility_fraction =
          static_cast<float>(found->second[0] * scale);
      cell.direct_w_m2 = static_cast<float>(found->second[1] * scale);
      cell.diffuse_sky_w_m2 = static_cast<float>(found->second[2] * scale);
      cell.scene_reflected_w_m2 = static_cast<float>(found->second[3] * scale);
    }
  }
  result.electrical_energy_wh =
      result.optical_energy_wh * effective_settings.efficiency;
  result.mean_electrical_power_w = race.duration_seconds > 0.0
                                      ? result.electrical_energy_wh * 3600.0 /
                                            race.duration_seconds
                                      : 0.0;
  return result;
}

}  // namespace solar
