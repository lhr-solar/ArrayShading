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
                                   const ShellRegion& region, float yaw) {
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
  const Vec3 u = normalized(base_u * c + base_v * s);
  const Vec3 v = normalized(-base_u * s + base_v * c);
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
      !std::isfinite(settings.time_limit_seconds) ||
      !std::isfinite(settings.cell_gap_m) ||
      !std::isfinite(settings.efficiency) ||
      !(settings.max_active_area_m2 > 0.0) ||
      !(settings.position_step_m > 0.0) ||
      !(settings.angle_step_degrees > 0.0 &&
        settings.angle_step_degrees <= 180.0) ||
      !(settings.time_limit_seconds > 0.0) || !(settings.cell_gap_m >= 0.0) ||
      !(settings.efficiency > 0.0 && settings.efficiency <= 1.0))
    throw std::invalid_argument("Invalid optimization settings");
}

}  // namespace

OptimizationSettings optimization_settings(const RaceConfig& config) {
  OptimizationSettings settings;
  settings.max_active_area_m2 = config.max_active_area_m2;
  settings.position_step_m = config.position_step_m;
  settings.angle_step_degrees = config.angle_step_degrees;
  settings.time_limit_seconds = config.solver_time_limit_seconds;
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
                      {"angle_step_degrees", config.angle_step_degrees},
                      {"solver_time_limit_seconds",
                       config.solver_time_limit_seconds}};
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

OptimizedLayout solve_module_packing(
    const std::vector<ModuleCandidate>& candidates,
    const OptimizationSettings& settings) {
  validate_settings(settings);
  OptimizedLayout output;
  output.candidate_modules = candidates.size();
  if (candidates.empty()) {
    output.status = "No feasible modules";
    return output;
  }
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
  highs.setOptionValue("time_limit", settings.time_limit_seconds);
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
  for (HighsInt i = 0; i < n; ++i)
    lp.col_cost_[i] = -candidates[static_cast<std::size_t>(i)].energy_wh;

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
      lp.a_matrix_.value_.push_back(
          row == area_row
              ? std::accumulate(candidates[static_cast<std::size_t>(col)].cells.begin(),
                                candidates[static_cast<std::size_t>(col)].cells.end(),
                                0.0, [](double sum, const Cell& cell) {
                                  return sum + cell.active_area_m2;
                                })
              : 1.0);
    lp.a_matrix_.start_[static_cast<std::size_t>(col) + 1U] =
        static_cast<HighsInt>(lp.a_matrix_.index_.size());
  }
  const HighsStatus pass_status = highs.passModel(model);
  if (pass_status == HighsStatus::kError)
    throw std::runtime_error("HiGHS rejected module-packing model");
  highs.run();
  const HighsModelStatus model_status = highs.getModelStatus();
  output.proven_optimal = model_status == HighsModelStatus::kOptimal;
  output.status = highs.modelStatusToString(model_status);
  const HighsInfo& info = highs.getInfo();
  const bool primary_has_solution = highs.getSolution().value_valid;
  if (info.valid && std::isfinite(info.mip_dual_bound))
    output.energy_upper_bound_wh = std::max(0.0, -info.mip_dual_bound);

  std::vector<double> selected_values;
  if (primary_has_solution) selected_values = highs.getSolution().col_value;
  if (output.proven_optimal && primary_has_solution) {
    double primary_energy = 0.0;
    for (std::size_t i = 0; i < candidates.size(); ++i)
      if (i < selected_values.size() && selected_values[i] >= 0.5)
        primary_energy += candidates[i].energy_wh;
    const double energy_tolerance =
        1.0e-9 * std::max(1.0, std::abs(primary_energy));
    const double remaining_seconds =
        settings.time_limit_seconds - highs.getRunTime();
    if (remaining_seconds > 0.001) {
      HighsModel secondary_model = model;
      HighsLp& secondary_lp = secondary_model.lp_;
      const HighsInt tie_row = secondary_lp.num_row_++;
      secondary_lp.row_lower_.push_back(primary_energy - energy_tolerance);
      secondary_lp.row_upper_.push_back(kHighsInf);
      const auto original_start = secondary_lp.a_matrix_.start_;
      const auto original_index = secondary_lp.a_matrix_.index_;
      const auto original_value = secondary_lp.a_matrix_.value_;
      secondary_lp.a_matrix_.start_.assign(static_cast<std::size_t>(n) + 1U,
                                           0);
      secondary_lp.a_matrix_.index_.clear();
      secondary_lp.a_matrix_.value_.clear();
      for (HighsInt col = 0; col < n; ++col) {
        for (HighsInt nz = original_start[static_cast<std::size_t>(col)];
             nz < original_start[static_cast<std::size_t>(col) + 1U]; ++nz) {
          secondary_lp.a_matrix_.index_.push_back(
              original_index[static_cast<std::size_t>(nz)]);
          secondary_lp.a_matrix_.value_.push_back(
              original_value[static_cast<std::size_t>(nz)]);
        }
        secondary_lp.a_matrix_.index_.push_back(tie_row);
        secondary_lp.a_matrix_.value_.push_back(
            candidates[static_cast<std::size_t>(col)].energy_wh);
        secondary_lp.a_matrix_.start_[static_cast<std::size_t>(col) + 1U] =
            static_cast<HighsInt>(secondary_lp.a_matrix_.index_.size());
        secondary_lp.col_cost_[static_cast<std::size_t>(col)] =
            candidates[static_cast<std::size_t>(col)].cells.size() == 6U
                ? -1.0
                : 0.0;
      }
      Highs secondary;
      secondary.setOptionValue("output_flag", false);
      secondary.setOptionValue("time_limit", remaining_seconds);
      const HighsStatus secondary_pass = secondary.passModel(secondary_model);
      if (secondary_pass != HighsStatus::kError) {
        secondary.run();
        if (secondary.getModelStatus() == HighsModelStatus::kOptimal &&
            secondary.getSolution().value_valid) {
          const auto& values = secondary.getSolution().col_value;
          double tie_energy = 0.0;
          for (std::size_t i = 0; i < candidates.size(); ++i)
            if (i < values.size() && values[i] >= 0.5)
              tie_energy += candidates[i].energy_wh;
          if (tie_energy + energy_tolerance >= primary_energy)
            selected_values = values;
        }
      }
    }
  }
  bool has_feasible_solution = !selected_values.empty();
  if (!has_feasible_solution) {
    std::vector<std::size_t> order(candidates.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return candidates[a].energy_wh > candidates[b].energy_wh;
    });
    std::vector<bool> blocked(candidates.size(), false);
    selected_values.assign(candidates.size(), 0.0);
    double used_area = 0.0;
    for (const std::size_t index : order) {
      if (blocked[index]) continue;
      const double area = std::accumulate(
          candidates[index].cells.begin(), candidates[index].cells.end(), 0.0,
          [](double sum, const Cell& cell) { return sum + cell.active_area_m2; });
      if (used_area + area > settings.max_active_area_m2 + 1.0e-9) continue;
      selected_values[index] = 1.0;
      used_area += area;
      for (const HighsInt row : conflict_rows[index]) {
        const auto& pair = conflicts[static_cast<std::size_t>(row)];
        const HighsInt other = pair.first == static_cast<HighsInt>(index)
                                   ? pair.second
                                   : pair.first;
        blocked[static_cast<std::size_t>(other)] = true;
      }
    }
    has_feasible_solution = true;
    output.status += " (greedy feasible fallback)";
  }
  if (has_feasible_solution) {
    const auto& values = selected_values;
    std::uint32_t module_id = 1;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (index >= values.size() || values[index] < 0.5) continue;
      const auto& candidate = candidates[index];
      const std::uint32_t columns = candidate.cells.size() == 6U ? 2U : 1U;
      for (std::size_t cell_index = 0; cell_index < candidate.cells.size();
           ++cell_index) {
        Cell cell = candidate.cells[cell_index];
        cell.row = static_cast<std::uint32_t>(cell_index / columns);
        cell.column = static_cast<std::uint32_t>(cell_index % columns);
        cell.module_id = module_id;
        cell.module_rows = 3U;
        cell.module_columns = columns;
        output.layout.cells.push_back(cell);
      }
      if (columns == 2U)
        ++output.layout.summary.two_by_three_modules;
      else
        ++output.layout.summary.one_by_three_modules;
      output.optical_energy_wh += candidate.energy_wh;
      ++module_id;
    }
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
    const OptimizationSettings& settings) {
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
  for (const RaceSample& sample : race.samples) {
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
  }
  for (ModuleCandidate& candidate : candidates) {
    for (const Cell& cell : candidate.cells) candidate.energy_wh += energy[cell_key(cell)];
  }
  OptimizedLayout result = solve_module_packing(candidates, effective_settings);
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
