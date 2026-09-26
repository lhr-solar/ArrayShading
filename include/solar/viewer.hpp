#pragma once

#include <filesystem>
#include <vector>

#include "solar/simulation.hpp"
#include "solar/stl_mesh.hpp"
#include "solar/trace_scene.hpp"

namespace solar {

void show_viewer(const TriangleMesh& mesh, const TraceScene& scene,
                 std::vector<Cell>& cells, Sun& sun,
                 SimulationSettings& settings, SimulationSummary& summary,
                 const std::filesystem::path& output_path);

}  // namespace solar
