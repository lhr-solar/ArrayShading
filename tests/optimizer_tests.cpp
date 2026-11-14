#include "solar/optimizer.hpp"
#include "solar/placement_surface.hpp"

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

solar::ModuleCandidate module(float x, double energy, std::uint32_t columns = 1) {
  solar::ModuleCandidate result;
  result.energy_wh = energy;
  for (std::uint32_t row = 0; row < 3U; ++row) {
    for (std::uint32_t column = 0; column < columns; ++column) {
      solar::Cell cell;
      cell.center = {x + static_cast<float>(column) * 0.128f,
                     static_cast<float>(row) * 0.128f, 0.0f};
      cell.normal = {0.0f, 0.0f, 1.0f};
      cell.u_axis = {1.0f, 0.0f, 0.0f};
      cell.v_axis = {0.0f, 1.0f, 0.0f};
      cell.width_m = 0.125f;
      cell.height_m = 0.125f;
      cell.active_area_m2 = 0.0153f;
      result.cells.push_back(cell);
    }
  }
  return result;
}

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void packing_selects_best_compatible_set() {
  solar::OptimizationSettings settings;
  settings.max_active_area_m2 = 1.0;
  const std::vector<solar::ModuleCandidate> candidates{
      module(0.0f, 10.0), module(0.07f, 9.0), module(-0.07f, 8.0)};
  const auto result = solar::solve_module_packing(candidates, settings);
  require(std::abs(result.optical_energy_wh - 17.0) < 1.0e-6,
          "MILP did not select the best compatible module set");
  require(result.layout.summary.individual_cells == 6U,
          "Selected cell count is incorrect");
}

void energy_is_primary_over_module_count() {
  solar::OptimizationSettings settings;
  settings.max_active_area_m2 = 0.10;
  settings.efficiency = 0.2;
  const std::vector<solar::ModuleCandidate> candidates{
      module(0.0f, 11.0, 2), module(1.0f, 12.0)};
  const auto result = solar::solve_module_packing(candidates, settings);
  require(std::abs(result.optical_energy_wh - 12.0) < 1.0e-6,
          "Energy objective did not prefer the higher-energy module");
  require(result.layout.summary.one_by_three_modules == 1U,
          "Selected 1x3 module was not counted");
  require(std::abs(result.electrical_energy_wh - 2.4) < 1.0e-6,
          "Electrical energy conversion is incorrect");
}

void strict_energy_tie_prefers_two_by_three() {
  solar::OptimizationSettings settings;
  settings.max_active_area_m2 = 0.10;
  const std::vector<solar::ModuleCandidate> candidates{
      module(-1.0f, 6.0), module(1.0f, 6.0), module(0.0f, 12.0, 2)};
  const auto result = solar::solve_module_packing(candidates, settings);
  require(std::abs(result.optical_energy_wh - 12.0) < 1.0e-6,
          "Tie-break changed the primary energy objective");
  require(result.layout.summary.two_by_three_modules == 1U,
          "Strict energy tie did not prefer the 2x3 grouping");
}

void energy_tie_prefers_more_active_area_over_two_by_three() {
  solar::OptimizationSettings settings;
  settings.max_active_area_m2 = 0.14;
  const std::vector<solar::ModuleCandidate> candidates{
      module(0.0f, 12.0, 2), module(-0.07f, 4.0), module(0.30f, 4.0), module(1.0f, 4.0)};
  const auto result = solar::solve_module_packing(candidates, settings);
  require(result.optical_energy_wh == 16.0 && result.layout.cells.size() == 9U,
          "fixture must preserve the higher-energy layout when its filler fits");
  // All three singles avoid each other but conflict with the double. One
  // overlaps it along Y, allowing three distinct parallel footprints.
  std::vector<solar::ModuleCandidate> tied{
      module(0.0f, 12.0, 2), module(-0.07f, 4.0), module(0.20f, 4.0), module(0.064f, 4.0)};
  for (auto& cell : tied.back().cells) cell.center.y += 0.256f;
  const auto fuller = solar::solve_module_packing(tied, settings);
  require(fuller.optical_energy_wh == 12.0 && fuller.layout.summary.one_by_three_modules == 3U &&
              fuller.status == "Optimal",
          "energy tie should use the most active area before preferring 2x3 modules");
}

void area_budget_is_enforced() {
  solar::OptimizationSettings settings;
  settings.max_active_area_m2 = 0.09;
  const std::vector<solar::ModuleCandidate> candidates{
      module(0.0f, 5.0), module(1.0f, 5.0)};
  const auto result = solar::solve_module_packing(candidates, settings);
  require(result.layout.summary.individual_cells == 3U,
          "Area budget did not limit the selected module count");
}

void conflict_search_respects_nondefault_gap() {
  solar::OptimizationSettings settings;
  settings.cell_gap_m = 1.0;
  const auto result = solar::solve_module_packing({module(0, 10), module(0.8f, 9)}, settings);
  require(std::abs(result.optical_energy_wh - 10) < 1.0e-6,
          "large required gap must not escape the conflict broadphase");
}

void packing_matches_exhaustive_small_cases() {
  std::mt19937 random(1729);
  for (int trial = 0; trial < 32; ++trial) {
    std::vector<solar::ModuleCandidate> candidates;
    for (int i = 0; i < 9; ++i)
      candidates.push_back(module(static_cast<float>(random() % 20) * 0.07f,
                                  1.0 + random() % 20, 1U + random() % 2));
    solar::OptimizationSettings settings;
    settings.max_active_area_m2 = 0.15 + static_cast<double>(random() % 5) * 0.05;
    double best_energy = -1;
    double best_area = 0.0;
    std::size_t best_two_by_three = 0;
    for (unsigned mask = 0; mask < (1U << candidates.size()); ++mask) {
      double area = 0, energy = 0;
      std::size_t twos = 0;
      bool feasible = true;
      for (std::size_t i = 0; i < candidates.size() && feasible; ++i) {
        if (!(mask & (1U << i))) continue;
        area += candidates[i].cells.size() * static_cast<double>(candidates[i].cells[0].active_area_m2);
        energy += candidates[i].energy_wh;
        twos += candidates[i].cells.size() == 6;
        for (std::size_t j = 0; j < i; ++j) {
          if (!(mask & (1U << j))) continue;
          // All fixtures are parallel rectangular strips with identical Y
          // extents, so their exact conflict test reduces to X intervals.
          const auto& a = candidates[i]; const auto& b = candidates[j];
          const float amin = a.cells.front().center.x - 0.0625f;
          const float amax = a.cells.back().center.x + 0.0625f;
          const float bmin = b.cells.front().center.x - 0.0625f;
          const float bmax = b.cells.back().center.x + 0.0625f;
          if (!(amax + 0.003f <= bmin || bmax + 0.003f <= amin)) feasible = false;
        }
      }
      if (!feasible || area > settings.max_active_area_m2) continue;
      if (energy > best_energy || (energy == best_energy &&
          (area > best_area || (area == best_area && twos > best_two_by_three)))) {
        best_energy = energy; best_area = area; best_two_by_three = twos;
      }
    }
    const auto result = solar::solve_module_packing(candidates, settings);
    require(std::abs(result.optical_energy_wh - best_energy) < 1.0e-6,
            "MILP energy differs from exhaustive feasible-subset enumeration");
    require(std::abs(result.layout.cells.size() * static_cast<double>(0.0153f) - best_area) < 1.0e-12,
            "MILP active area differs from exhaustive enumeration among energy-optimal layouts");
    require(result.layout.summary.two_by_three_modules == best_two_by_three,
            "MILP 2x3 tie-break differs from exhaustive enumeration");
  }
}

solar::TriangleMesh make_flat_mesh() {
  solar::TriangleMesh mesh;
  for (const solar::Vec3 vertex : {
           solar::Vec3{-0.75f, -0.75f, 0.5f},
           solar::Vec3{0.75f, -0.75f, 0.5f},
           solar::Vec3{-0.75f, 0.75f, 0.5f},
           solar::Vec3{0.75f, -0.75f, 0.5f},
           solar::Vec3{0.75f, 0.75f, 0.5f},
           solar::Vec3{-0.75f, 0.75f, 0.5f}}) {
    mesh.vertices.push_back(vertex);
    mesh.bounds.include(vertex);
  }
  return mesh;
}

void shell_packing_is_deterministic_and_supported() {
  auto mesh = make_flat_mesh();
  solar::TraceScene scene(mesh);
  solar::CellGrid grid;
  solar::ShellRegion region;
  region.minimum = {-1, -1, 0}; region.maximum = {1, 1, 1};
  region.use_canopy_exclusion = false;
  solar::OptimizationSettings settings;
  settings.position_step_m = 0.256;
  settings.angle_step_degrees = 90;
  settings.max_active_area_m2 = 0.01;  // Full-shell preview ignores this cap.
  const auto a = solar::pack_shell_perimeter_inward(scene, mesh.bounds, scene, grid, region, settings);
  const auto b = solar::pack_shell_perimeter_inward(scene, mesh.bounds, scene, grid, region, settings);
  require(!a.cells.empty() && a.cells.size() == b.cells.size(),
          "full-shell packing must produce a deterministic layout independent of the race area cap");
  require(a.summary.two_by_three_modules > 0 &&
          a.summary.individual_cells == a.cells.size(), "packing summary must count modules and cells");
  require(a.cells.size() >= 114,
          "an 11-by-11 flat deck should be densely filled with aligned modules");
  for (std::size_t i = 0; i < a.cells.size(); ++i) {
    require(a.cells[i].center.x == b.cells[i].center.x && a.cells[i].center.y == b.cells[i].center.y,
            "packing positions must be deterministic");
    require(a.cells[i].module_rows == 3 && (a.cells[i].module_columns == 1 || a.cells[i].module_columns == 2),
            "packing must retain complete module metadata");
    require(std::min(std::abs(a.cells[i].v_axis.x), std::abs(a.cells[i].v_axis.y)) < 1.0e-5f,
            "a rectangular deck must not use diagonal placements");
  }
}

void perimeter_packing_follows_the_rounded_nose() {
  // Flat shell with a wide, square tail at +Y and a semicircular nose at -Y.
  solar::TriangleMesh mesh;
  const auto triangle = [&](solar::Vec3 a, solar::Vec3 b, solar::Vec3 c) {
    for (const auto p : {a, b, c}) { mesh.vertices.push_back(p); mesh.bounds.include(p); }
  };
  triangle({-1,0,0.5f}, {1,0,0.5f}, {1,1.5f,0.5f});
  triangle({-1,0,0.5f}, {1,1.5f,0.5f}, {-1,1.5f,0.5f});
  solar::Vec3 previous{-1,0,0.5f};
  for (int i = 1; i <= 32; ++i) {
    const float angle = solar::kPi * (1.0f + i / 32.0f);
    const solar::Vec3 next{std::cos(angle), std::sin(angle), 0.5f};
    triangle({0,0,0.5f}, previous, next);
    previous = next;
  }
  solar::TraceScene scene(mesh);
  solar::CellGrid grid;
  solar::ShellRegion region;
  region.minimum = {-1,-1,0}; region.maximum = {1,1.5f,1};
  region.exclusion_minimum = {-0.25f, 0.2f, 0};
  region.exclusion_maximum = {0.25f, 1.0f, 1};
  const auto packed = solar::pack_shell_perimeter_inward(scene, mesh.bounds, scene, grid, region, {});
  require(!packed.cells.empty() && packed.cells.front().center.y > 0.8f,
          "perimeter packing should start at the broad rear end");
  bool follows_curve = false;
  bool left_strip = false, right_strip = false;
  for (const auto& cell : packed.cells) {
    require(solar::validate_shell_cell(cell, scene, scene, grid), "contour cell must remain on the actual surface");
    if (cell.center.y > 0.25f)
      require(std::min(std::abs(cell.v_axis.x), std::abs(cell.v_axis.y)) < 1.0e-5f,
              "the straight deck remains aligned even when the nose is curved");
    if (cell.center.y > 0.25f && cell.center.y < 0.95f) {
      left_strip |= cell.center.x < -0.3f;
      right_strip |= cell.center.x > 0.3f;
    }
    const solar::Vec3 radial{cell.center.x, cell.center.y, 0};
    if (cell.center.y < -0.25f && std::abs(cell.v_axis.x) > 0.2f && std::abs(cell.v_axis.y) > 0.2f &&
        std::abs(solar::dot(solar::normalized(radial), cell.v_axis)) < 0.3f)
      follows_curve = true;
  }
  require(follows_curve, "rounded nose should receive modules oriented along the local contour");
  require(left_strip && right_strip, "the same aligned grid should fill both strips beside a canopy exclusion");
}

void track_profiles_preserve_each_sample_and_cell() {
  auto mesh = make_flat_mesh();
  solar::TraceScene scene(mesh);
  auto cells = module(0, 0).cells;
  for (auto& cell : cells) cell.center.z = 0.501f;
  solar::RaceRun race;
  solar::RaceSample sample;
  sample.duration_seconds = 10;
  sample.sun.direction_to_sun = {0, 0, 1};
  sample.sun.dni_w_m2 = 900;
  sample.sun.dhi_w_m2 = 0;
  race.samples.push_back(sample);
  sample.duration_seconds = 20;
  sample.sun.dni_w_m2 = 450;
  race.samples.push_back(sample);
  solar::SimulationSettings settings;
  settings.worker_threads = 1;
  settings.hemisphere_samples = 8;
  settings.max_reflection_depth = 0;
  const auto profiles = solar::sample_cell_track_profiles(cells, scene, race, settings);
  require(profiles.cells.size() == 3 && profiles.irradiance_w_m2.size() == 2 &&
          profiles.race.samples[1].duration_seconds == 20,
          "track profiles must preserve cell order and sample metadata");
  for (std::size_t i = 0; i < cells.size(); ++i)
    require(std::abs(profiles.irradiance_w_m2[0][i] - 900) < 0.01 &&
            std::abs(profiles.irradiance_w_m2[1][i] - 450) < 0.01,
            "each sample must retain its own irradiance without accumulating previous samples");
}

void zone_grouping_bridges_gaps_without_skipping_the_middle() {
  solar::CellTrackProfiles profiles;
  for (int i = 0; i < 3; ++i) {
    auto candidate = module(0, 0);
    for (auto& cell : candidate.cells) {
      cell.module_id = static_cast<std::uint32_t>(i + 1);
      cell.center.y += static_cast<float>(i);
      profiles.cells.push_back(cell);
    }
  }
  solar::RaceSample sample; sample.duration_seconds = 5;
  profiles.race.samples = {sample, sample};
  profiles.irradiance_w_m2 = {{100, 100, 100, 110, 110, 110, 800, 800, 800},
                              {200, 200, 200, 210, 210, 210, 100, 100, 100}};
  const auto two = solar::group_track_profiles(profiles, 2);
  require(two[0] == two[3] && two[3] != two[6],
          "similar modules should join across empty space without requiring cell-edge contact");
  const auto three = solar::group_track_profiles(profiles, 3);
  require(three[0] != three[3] && three[0] != three[6] && three[3] != three[6],
          "three zones should contain separate modules");
  profiles.irradiance_w_m2 = {{100, 100, 100, 800, 800, 800, 100, 100, 100},
                              {200, 200, 200, 100, 100, 100, 200, 200, 200}};
  const auto ends = solar::group_track_profiles(profiles, 2);
  require(ends[0] != ends[6], "matching front and rear modules cannot jump past the intervening zone");
}

void zone_grouping_can_split_left_and_right() {
  solar::CellTrackProfiles profiles;
  for (int i = 0; i < 4; ++i) {
    auto candidate = module(i < 2 ? -0.5f : 0.5f, 0);
    for (auto& cell : candidate.cells) {
      cell.module_id = static_cast<std::uint32_t>(i+1);
      cell.center.y += (i % 2) * 0.7f;
      profiles.cells.push_back(cell);
    }
  }
  solar::RaceSample sample; sample.duration_seconds = 5;
  profiles.race.samples = {sample, sample};
  profiles.irradiance_w_m2 = {{100, 100, 100, 110, 110, 110, 800, 800, 800, 790, 790, 790},
                              {800, 800, 800, 790, 790, 790, 100, 100, 100, 110, 110, 110}};
  const auto zones = solar::group_track_profiles(profiles, 2);
  require(zones[0] == zones[3] && zones[6] == zones[9] && zones[0] != zones[6],
          "similar profiles can form coherent side-by-side zones");
}

void arbitrary_sampled_rotation_is_supported() {
  auto mesh = make_flat_mesh();
  solar::TraceScene scene(mesh);
  solar::Bounds bounds;
  bounds.minimum = {-0.5f, -0.5f, 0.5f};
  bounds.maximum = {0.5f, 0.5f, 0.5f};
  solar::CellGrid grid;
  grid.minimum_upward_normal_z = 0.75f;
  solar::ShellRegion region;
  region.minimum = {-1.0f, -1.0f, 0.0f};
  region.maximum = {1.0f, 1.0f, 1.0f};
  region.exclusion_minimum = {2.0f, 2.0f, 2.0f};
  region.exclusion_maximum = {3.0f, 3.0f, 3.0f};
  solar::OptimizationSettings settings;
  settings.position_step_m = 0.5;
  settings.angle_step_degrees = 15.0;
  const auto candidates = solar::generate_module_candidates(
      scene, bounds, grid, region, settings);
  bool found_rotated = false;
  for (const auto& candidate : candidates) {
    for (const auto& cell : candidate.cells)
      if (std::abs(cell.u_axis.y) > 0.1f) found_rotated = true;
  }
  require(found_rotated, "No rotated shell-conforming module was generated");
}

void exclusion_region_rejects_canopy_area() {
  auto mesh = make_flat_mesh();
  solar::TraceScene scene(mesh);
  solar::Bounds bounds;
  bounds.minimum = {-0.5f, -0.5f, 0.5f};
  bounds.maximum = {0.5f, 0.5f, 0.5f};
  solar::CellGrid grid;
  solar::ShellRegion region;
  region.minimum = {-1.0f, -1.0f, 0.0f};
  region.maximum = {1.0f, 1.0f, 1.0f};
  region.exclusion_minimum = {-1.0f, -1.0f, 0.0f};
  region.exclusion_maximum = {1.0f, 1.0f, 1.0f};
  solar::OptimizationSettings settings;
  settings.position_step_m = 0.25;
  const auto candidates = solar::generate_module_candidates(
      scene, bounds, grid, region, settings);
  require(candidates.empty(), "Excluded canopy area received a module");
}

void invalid_settings_are_rejected() {
  solar::OptimizationSettings settings;
  settings.position_step_m = 0.0;
  try {
    (void)solar::solve_module_packing({}, settings);
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error("Invalid optimizer settings were accepted");
}

void timed_raytracing_and_packing_match_analytical_energy() {
  auto mesh = make_flat_mesh();
  solar::TraceScene scene(mesh);
  solar::CellGrid grid;
  solar::ShellRegion region;
  region.minimum = {-1, -1, 0}; region.maximum = {1, 1, 1};
  region.exclusion_minimum = {2, 2, 2}; region.exclusion_maximum = {3, 3, 3};
  solar::OptimizationSettings settings;
  settings.max_active_area_m2 = 0.092;
  settings.position_step_m = 0.5;
  settings.angle_step_degrees = 45;
  solar::RaceRun race;
  race.duration_seconds = 7200;
  solar::RaceSample sample;
  sample.duration_seconds = 3600;
  sample.sun.direction_to_sun = {0, 0, 1};
  sample.sun.dni_w_m2 = 900;
  sample.sun.dhi_w_m2 = 0;
  race.samples.push_back(sample);
  sample.sun.dni_w_m2 = 450;
  race.samples.push_back(sample);
  solar::SimulationSettings rays;
  rays.worker_threads = 1;
  rays.hemisphere_samples = 8;
  rays.max_reflection_depth = 0;
  std::vector<solar::OptimizationProgress> updates;
  const auto result = solar::optimize_race_layout(scene, mesh.bounds, scene,
      grid, region, race, rays, settings,
      [&](solar::OptimizationProgress update) { updates.push_back(update); });
  using Phase = solar::OptimizationPhase;
  require(updates.size() >= 7 && updates.front().phase == Phase::candidates &&
          updates[4].phase == Phase::packing_setup && updates[5].phase == Phase::solving &&
          updates[5].best_electrical_wh > 0 &&
          updates.back().phase == Phase::finishing, "optimization reports phases and a feasible starting score");
  for (std::size_t i = 0; i <= race.samples.size(); ++i)
    require(updates[i + 1].phase == Phase::scoring &&
            updates[i + 1].completed_samples == i &&
            updates[i + 1].total_samples == race.samples.size(),
            "scoring progress counts completed track samples");
  require(result.layout.cells.size() == 6, "integration should select six cells under area cap");
  const double expected = 675.0 * grid.active_area_m2 * 6.0 * 2.0;
  require(std::abs(result.optical_energy_wh - expected) < 0.001,
          "time-weighted optical Wh disagrees with analytical flat-shell result");
  require(std::abs(result.electrical_energy_wh - expected * settings.efficiency) < 0.001,
          "electrical Wh disagrees with constant efficiency");
  for (const auto& cell : result.layout.cells)
    require(std::abs(cell.direct_w_m2 - 675) < 0.001,
            "rendered cells should contain race-average irradiance, not final sample");
}

void stopped_packing_preserves_feasible_start() {
  solar::OptimizationSettings settings;
  settings.max_active_area_m2 = 0.1;
  const std::vector<solar::ModuleCandidate> candidates{
      module(0.0f, 10.0), module(0.07f, 9.0), module(-0.07f, 8.0), module(1.0f, 7.0)};
  const auto result = solar::solve_module_packing(candidates, settings, {}, [] { return true; });
  require(!result.proven_optimal && result.layout.cells.size() == 6 &&
              std::abs(result.optical_energy_wh - 17.0) < 1.0e-6,
          "manual stop keeps the non-overlapping, area-limited starting layout");
}

void unsupported_cells_cannot_turn_module_into_three_cell_fragment() {
  auto source = make_flat_mesh();
  solar::TraceScene top_scene(source);
  solar::TriangleMesh car;
  const auto patch = [&](float x, float y) {
    const solar::Vec3 a{x - 0.0635f, y - 0.0635f, 0.5f};
    const solar::Vec3 b{x + 0.0635f, y - 0.0635f, 0.5f};
    const solar::Vec3 c{x - 0.0635f, y + 0.0635f, 0.5f};
    const solar::Vec3 d{x + 0.0635f, y + 0.0635f, 0.5f};
    car.vertices.insert(car.vertices.end(), {a, b, c, b, d, c});
    for (const auto& p : {a, b, c, d}) car.bounds.include(p);
  };
  patch(-0.064f, -0.128f); patch(-0.064f, 0); patch(0.064f, 0);
  solar::TraceScene full(car);
  solar::ShellRegion region;
  region.minimum = {-1, -1, 0}; region.maximum = {1, 1, 1};
  region.use_canopy_exclusion = false;
  solar::Bounds anchors; anchors.minimum = {0, 0, 0.5f}; anchors.maximum = anchors.minimum;
  solar::OptimizationSettings settings; settings.angle_step_degrees = 180;
  solar::RaceRun race; race.duration_seconds = 3600;
  solar::RaceSample sample; sample.duration_seconds = 3600;
  sample.sun.direction_to_sun = {0, 0, 1}; race.samples.push_back(sample);
  solar::SimulationSettings rays; rays.worker_threads = 1; rays.hemisphere_samples = 8;
  const auto result = solar::optimize_race_layout(top_scene, anchors, full,
      solar::CellGrid{}, region, race, rays, settings);
  require(result.layout.cells.empty(),
      "partial support must reject the whole module, not retain an invalid three-cell fragment");
}

}  // namespace

int main() {
  try {
    packing_selects_best_compatible_set();
    energy_is_primary_over_module_count();
    strict_energy_tie_prefers_two_by_three();
    energy_tie_prefers_more_active_area_over_two_by_three();
    area_budget_is_enforced();
    conflict_search_respects_nondefault_gap();
    packing_matches_exhaustive_small_cases();
    arbitrary_sampled_rotation_is_supported();
    shell_packing_is_deterministic_and_supported();
    perimeter_packing_follows_the_rounded_nose();
    track_profiles_preserve_each_sample_and_cell();
    zone_grouping_bridges_gaps_without_skipping_the_middle();
    zone_grouping_can_split_left_and_right();
    exclusion_region_rejects_canopy_area();
    invalid_settings_are_rejected();
    timed_raytracing_and_packing_match_analytical_energy();
    stopped_packing_preserves_feasible_start();
    unsupported_cells_cannot_turn_module_into_three_cell_fragment();
  } catch (const std::exception& error) {
    std::cerr << "optimizer test failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
