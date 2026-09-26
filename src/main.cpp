#include "solar/simulation.hpp"
#include "solar/stl_mesh.hpp"
#include "solar/trace_scene.hpp"
#include "solar/viewer.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

struct Options {
  std::filesystem::path stl_path;
  std::filesystem::path output_path = "results/cpp-cell-irradiance.csv";
  solar::Transform transform;
  solar::Sun sun;
  solar::SimulationSettings simulation;
  solar::CellGrid grid;
  std::size_t max_triangles = 0;
  bool headless = false;
};

float number(const char* value, std::string_view flag) {
  try {
    std::size_t used = 0;
    const float result = std::stof(value, &used);
    if (used != std::string(value).size()) throw std::invalid_argument("tail");
    return result;
  } catch (...) {
    throw std::runtime_error("Expected a number after " + std::string(flag));
  }
}

std::uint32_t positive_integer(const char* value, std::string_view flag) {
  try {
    std::size_t used = 0;
    const unsigned long result = std::stoul(value, &used);
    if (used != std::string(value).size() || result == 0 ||
        result > 0xFFFFFFFFUL)
      throw std::invalid_argument("range");
    return static_cast<std::uint32_t>(result);
  } catch (...) {
    throw std::runtime_error("Expected a positive integer after " +
                             std::string(flag));
  }
}

std::uint32_t nonnegative_integer(const char* value, std::string_view flag) {
  try {
    std::size_t used = 0;
    const unsigned long result = std::stoul(value, &used);
    if (used != std::string(value).size() || result > 0xFFFFFFFFUL)
      throw std::invalid_argument("range");
    return static_cast<std::uint32_t>(result);
  } catch (...) {
    throw std::runtime_error("Expected a non-negative integer after " +
                             std::string(flag));
  }
}

void usage(const char* executable) {
  std::cout
      << "Usage: " << executable << " --stl PATH [options]\n\n"
      << "Full-triangle solar-car irradiance simulation. Coordinates are x=east, "
         "y=north, z=up.\n\n"
      << "Geometry:\n"
      << "  --stl PATH                 Binary car STL (required)\n"
      << "  --scale N                  STL-unit to metre scale (default 0.001)\n"
      << "  --rotate-x DEG             X rotation (default 90)\n"
      << "  --rotate-y DEG             Y rotation (default 0)\n"
      << "  --rotate-z DEG             Z rotation (default 0)\n"
      << "  --translate-x M            X translation (default 0)\n"
      << "  --translate-y M            Y translation (default -0.6563)\n"
      << "  --translate-z M            Z translation (default 0.0153)\n"
      << "  --max-triangles N          Debug only: load first N triangles\n\n"
      << "Sun and integration:\n"
      << "  --azimuth DEG              Clockwise from +y/north (default 125.75)\n"
      << "  --elevation DEG            Above horizon (default 72.051)\n"
      << "  --dni W_M2                 Direct normal irradiance (default 850)\n"
      << "  --dhi W_M2                 Diffuse horizontal irradiance (default 110)\n"
      << "  --rays N                   Hemisphere rays per cell (default 64)\n"
      << "  --depth N                  Specular recursion depth (default 2)\n"
      << "  --threads N                Worker count (default hardware count)\n\n"
      << "Cell preview grid:\n"
      << "  --rows N                   Candidate rows (default 44)\n"
      << "  --columns N                Candidate columns (default 12)\n"
      << "  --cell-gap M               Footprint gap (default 0.003)\n"
      << "  --normal-radius M          Shell-normal sample distance (default 0.18)\n\n"
      << "Output:\n"
      << "  --output PATH              Per-cell CSV path\n"
      << "  --headless                 Skip the OpenGL result window\n"
      << "  --help                     Show this message\n";
}

Options parse_options(int argc, char** argv) {
  Options options;
  float azimuth = 125.75f;
  float elevation = 72.05125f;
  auto next = [&](int& index, std::string_view flag) -> const char* {
    if (index + 1 >= argc)
      throw std::runtime_error("Missing value after " + std::string(flag));
    return argv[++index];
  };
  for (int index = 1; index < argc; ++index) {
    const std::string_view flag(argv[index]);
    if (flag == "--help" || flag == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else if (flag == "--stl") {
      options.stl_path = next(index, flag);
    } else if (flag == "--output") {
      options.output_path = next(index, flag);
    } else if (flag == "--scale") {
      options.transform.scale = number(next(index, flag), flag);
    } else if (flag == "--rotate-x") {
      options.transform.rotation_degrees.x = number(next(index, flag), flag);
    } else if (flag == "--rotate-y") {
      options.transform.rotation_degrees.y = number(next(index, flag), flag);
    } else if (flag == "--rotate-z") {
      options.transform.rotation_degrees.z = number(next(index, flag), flag);
    } else if (flag == "--translate-x") {
      options.transform.translation.x = number(next(index, flag), flag);
    } else if (flag == "--translate-y") {
      options.transform.translation.y = number(next(index, flag), flag);
    } else if (flag == "--translate-z") {
      options.transform.translation.z = number(next(index, flag), flag);
    } else if (flag == "--azimuth") {
      azimuth = number(next(index, flag), flag);
    } else if (flag == "--elevation") {
      elevation = number(next(index, flag), flag);
    } else if (flag == "--dni") {
      options.sun.dni_w_m2 = number(next(index, flag), flag);
    } else if (flag == "--dhi") {
      options.sun.dhi_w_m2 = number(next(index, flag), flag);
    } else if (flag == "--rays") {
      options.simulation.hemisphere_samples =
          positive_integer(next(index, flag), flag);
    } else if (flag == "--depth") {
      options.simulation.max_reflection_depth =
          nonnegative_integer(next(index, flag), flag);
    } else if (flag == "--threads") {
      options.simulation.worker_threads =
          positive_integer(next(index, flag), flag);
    } else if (flag == "--rows") {
      options.grid.rows = positive_integer(next(index, flag), flag);
    } else if (flag == "--columns") {
      options.grid.columns = positive_integer(next(index, flag), flag);
    } else if (flag == "--cell-gap") {
      options.grid.gap_m = number(next(index, flag), flag);
    } else if (flag == "--normal-radius") {
      options.grid.normal_sample_distance_m = number(next(index, flag), flag);
    } else if (flag == "--max-triangles") {
      options.max_triangles = positive_integer(next(index, flag), flag);
    } else if (flag == "--headless") {
      options.headless = true;
    } else {
      throw std::runtime_error("Unknown option: " + std::string(flag));
    }
  }
  if (options.stl_path.empty()) throw std::runtime_error("--stl PATH is required");
  if (options.transform.scale <= 0.0f || options.sun.dni_w_m2 < 0.0f ||
      options.sun.dhi_w_m2 < 0.0f || options.grid.gap_m < 0.0f ||
      options.grid.normal_sample_distance_m <= 0.0f) {
    throw std::runtime_error("Scale/radius must be positive; irradiance/gap cannot be negative");
  }
  if (elevation < -90.0f || elevation > 90.0f)
    throw std::runtime_error("Elevation must be between -90 and 90 degrees");
  options.sun.direction_to_sun =
      solar::sun_direction_from_azimuth_elevation(azimuth, elevation);
  return options;
}

template <typename Clock = std::chrono::steady_clock>
double elapsed_seconds(const typename Clock::time_point& start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    using Clock = std::chrono::steady_clock;
    auto start = Clock::now();
    std::cout << "Loading full binary STL: " << options.stl_path << '\n';
    solar::TriangleMesh mesh = solar::load_binary_stl(
        options.stl_path, options.transform, options.max_triangles);
    std::cout << "Loaded " << mesh.triangle_count() << " triangles in "
              << std::fixed << std::setprecision(2) << elapsed_seconds(start)
              << " s\nBounds: [" << mesh.bounds.minimum.x << ", "
              << mesh.bounds.minimum.y << ", " << mesh.bounds.minimum.z
              << "] to [" << mesh.bounds.maximum.x << ", "
              << mesh.bounds.maximum.y << ", " << mesh.bounds.maximum.z
              << "] m\n";

    start = Clock::now();
    std::cout << "Building Embree BVH (this is the ray-tracing geometry)...\n";
    solar::TraceScene scene(mesh);
    std::cout << "BVH built in " << elapsed_seconds(start) << " s\n";

    start = Clock::now();
    std::vector<solar::Cell> cells =
        solar::project_cells(scene, mesh.bounds, options.grid);
    std::cout << "Projected " << cells.size()
              << " Maxeon Gen III candidate cells\n";
    solar::SimulationSummary summary =
        solar::simulate_cells(cells, scene, options.sun, options.simulation);
    std::cout << "Simulation finished in " << elapsed_seconds(start) << " s\n"
              << "  active area: " << summary.active_area_m2 << " m²\n"
              << "  area-weighted irradiance: "
              << summary.area_weighted_irradiance_w_m2 << " W/m²\n"
              << "  incident optical power: "
              << summary.incident_solar_power_w << " W\n";

    solar::write_cell_csv(options.output_path, cells);
    std::cout << "Wrote " << options.output_path << '\n';
    if (!options.headless) {
      std::cout << "Opening OpenGL viewer; shell heat map will calculate in "
                   "the background...\n";
      std::vector<float> shell_irradiance(mesh.triangle_count(), -1.0f);
      solar::Sun interactive_sun = options.sun;
      solar::SimulationSettings interactive_settings = options.simulation;
      solar::show_viewer(mesh, scene, cells, interactive_sun,
                         interactive_settings, summary, shell_irradiance,
                         options.output_path);
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\nUse --help for options.\n";
    return 1;
  }
}
