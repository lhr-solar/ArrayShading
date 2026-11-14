#pragma once

#include <string>
#include <filesystem>
#include <vector>

#include "solar/race.hpp"
#include "solar/simulation.hpp"

namespace solar {

struct OptimizationSettings {
  double max_active_area_m2 = 6.0;
  double position_step_m = 0.128;
  double angle_step_degrees = 15.0;
  double time_limit_seconds = 60.0;
  double cell_gap_m = 0.003;
  double efficiency = 0.243;
};

struct ModuleCandidate {
  std::vector<Cell> cells;
  double energy_wh = 0.0;
};

struct OptimizedLayout {
  GeneratedLayout layout;
  double optical_energy_wh = 0.0;
  double electrical_energy_wh = 0.0;
  double mean_electrical_power_w = 0.0;
  double energy_upper_bound_wh = 0.0;
  double relative_gap = 0.0;
  bool proven_optimal = false;
  std::size_t candidate_modules = 0;
  std::string status;
};

OptimizationSettings optimization_settings(const RaceConfig& config);

void write_optimized_race(const std::filesystem::path& path,
                          const RaceConfig& config, const RaceRun& race,
                          const OptimizedLayout& result);

std::vector<ModuleCandidate> generate_module_candidates(
    const TraceScene& placement_scene, const Bounds& bounds,
    const CellGrid& grid, const ShellRegion& region,
    const OptimizationSettings& settings);

OptimizedLayout solve_module_packing(
    const std::vector<ModuleCandidate>& candidates,
    const OptimizationSettings& settings);

OptimizedLayout optimize_race_layout(
    const TraceScene& placement_scene, const Bounds& bounds,
    const TraceScene& complete_trace_scene, const CellGrid& grid,
    const ShellRegion& region, const RaceRun& race,
    const SimulationSettings& simulation,
    const OptimizationSettings& settings);

}  // namespace solar
