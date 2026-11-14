#pragma once

#include <filesystem>
#include <memory>
#include <utility>
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

// Embree shares the vertex buffer, so the mesh must outlive its scene.
struct ViewerCar {
  TriangleMesh mesh;
  TraceScene scene;
  explicit ViewerCar(TriangleMesh geometry)
      : mesh(std::move(geometry)), scene(mesh) {}
  ViewerCar(ViewerCar&&) = delete;
};

void show_viewer(std::shared_ptr<const ViewerCar> car,
                 std::vector<Cell>& cells, Sun& sun,
                 SimulationSettings& settings, SimulationSummary& summary,
                 LayoutSummary& layout_summary,
                 const CellGrid& grid, const ShellRegion& region,
                 const ViewerGeometryInputs& geometry_inputs,
                 const std::filesystem::path& race_config_path,
                 const std::filesystem::path& output_path);

}  // namespace solar
