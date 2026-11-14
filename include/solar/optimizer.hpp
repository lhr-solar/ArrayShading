#pragma once

#include <string>
#include <filesystem>
#include <functional>
#include <vector>

#include "solar/race.hpp"
#include "solar/simulation.hpp"

namespace solar {

struct OptimizationSettings {
  double max_active_area_m2 = 6.0;
  double position_step_m = 0.128;
  double angle_step_degrees = 15.0;
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
  double energy_upper_bound_wh = -1.0;
  double relative_gap = -1.0;
  bool proven_optimal = false;
  std::size_t candidate_modules = 0;
  std::string status;
};

enum class OptimizationPhase { candidates, scoring, packing_setup, solving, area_tie_break, module_tie_break, finishing };

struct OptimizationProgress {
  OptimizationPhase phase;
  std::size_t completed_samples = 0;
  std::size_t total_samples = 0;
  double best_electrical_wh = -1.0;
  double relative_gap = -1.0;
};

using OptimizationProgressCallback = std::function<void(OptimizationProgress)>;
using OptimizationStopCallback = std::function<bool()>;

OptimizationSettings optimization_settings(const RaceConfig& config);

void write_optimized_race(const std::filesystem::path& path,
                          const RaceConfig& config, const RaceRun& race,
                          const OptimizedLayout& result);

std::vector<ModuleCandidate> generate_module_candidates(
    const TraceScene& placement_scene, const Bounds& bounds,
    const CellGrid& grid, const ShellRegion& region,
    const OptimizationSettings& settings);

GeneratedLayout pack_shell_perimeter_inward(
    const TraceScene& placement_scene, const Bounds& bounds,
    const TraceScene& complete_trace_scene, const CellGrid& grid,
    const ShellRegion& region, const OptimizationSettings& settings);

// Sample-major irradiance profiles: row k corresponds to race.samples[k],
// and column i corresponds to cells[i]. Values are incident W/m².
struct CellTrackProfiles {
  std::vector<Cell> cells;
  RaceRun race;
  std::vector<std::vector<float>> irradiance_w_m2;
};

CellTrackProfiles sample_cell_track_profiles(
    const std::vector<Cell>& cells, const TraceScene& scene,
    const RaceRun& race, const SimulationSettings& settings,
    const OptimizationProgressCallback& progress = {},
    const OptimizationStopCallback& stop = {});

// Zone indices correspond to CellTrackProfiles::cells. K-medoids can require
// each zone to be connected by local module neighbors in XY.
std::vector<std::size_t> group_track_profiles(
    const CellTrackProfiles& profiles, std::size_t zone_count,
    bool require_connected = true);

OptimizedLayout solve_module_packing(
    const std::vector<ModuleCandidate>& candidates,
    const OptimizationSettings& settings,
    const OptimizationProgressCallback& progress = {},
    const OptimizationStopCallback& stop = {});

OptimizedLayout optimize_race_layout(
    const TraceScene& placement_scene, const Bounds& bounds,
    const TraceScene& complete_trace_scene, const CellGrid& grid,
    const ShellRegion& region, const RaceRun& race,
    const SimulationSettings& simulation,
    const OptimizationSettings& settings,
    const OptimizationProgressCallback& progress = {},
    const OptimizationStopCallback& stop = {});

}  // namespace solar
