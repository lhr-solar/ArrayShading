#pragma once

#include <filesystem>
#include <vector>

#include "solar/simulation.hpp"
#include "solar/stl_mesh.hpp"
#include "solar/trace_scene.hpp"

namespace solar {

struct ViewerGeometryInputs {
  std::filesystem::path car_path;
  std::filesystem::path top_shell_path;
  Transform transform;
  Transform top_correction{1.0f, {0, 0, 0}, {0, 0, 0}};
};

void show_viewer(const TriangleMesh& mesh, const TraceScene& scene,
                 std::vector<Cell>& cells, Sun& sun,
                 SimulationSettings& settings, SimulationSummary& summary,
                 LayoutSummary& layout_summary,
                 const CellGrid& grid, const ShellRegion& region,
                 const ViewerGeometryInputs& geometry_inputs,
                 const std::filesystem::path& race_config_path,
                 const std::filesystem::path& output_path);

}  // namespace solar
