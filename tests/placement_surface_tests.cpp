#include "solar/placement_surface.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

using solar::TriangleMesh;
using solar::Vec3;

void add_quad(TriangleMesh& mesh, float xmin, float xmax, float ymin,
              float ymax, float z) {
  const Vec3 a{xmin, ymin, z};
  const Vec3 b{xmax, ymin, z};
  const Vec3 c{xmin, ymax, z};
  const Vec3 d{xmax, ymax, z};
  mesh.vertices.insert(mesh.vertices.end(), {a, b, d, a, d, c});
  for (const Vec3& point : {a, b, c, d}) mesh.bounds.include(point);
}

TriangleMesh plane(float z, float xmin = -1.0f, float xmax = 1.0f,
                   float ymin = -1.0f, float ymax = 1.0f) {
  TriangleMesh mesh;
  add_quad(mesh, xmin, xmax, ymin, ymax, z);
  return mesh;
}

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void test_alignment_and_rejection() {
  const TriangleMesh roof = plane(1.0f);
  solar::TraceScene matching(roof);
  const auto prepared = solar::prepare_placement_surface(roof, matching);
  require(prepared.mesh.triangle_count() == roof.triangle_count(),
          "aligned top-shell triangles were not retained");
  require(prepared.matched_samples == prepared.checked_samples,
          "aligned roof samples were not matched");

  const TriangleMesh shifted_roof = plane(1.1f);
  solar::TraceScene car(roof);
  bool rejected = false;
  try {
    (void)solar::prepare_placement_surface(shifted_roof, car);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "misaligned top-shell geometry was accepted");
}

void test_canopy_and_footprint_support() {
  const TriangleMesh roof = plane(1.0f);
  solar::TraceScene top(roof);
  TriangleMesh with_canopy = roof;
  add_quad(with_canopy, -0.25f, 0.25f, -0.25f, 0.25f, 1.2f);
  solar::TraceScene full(with_canopy);

  solar::Cell cell;
  cell.center = {0.0f, 0.0f, 1.001f};
  cell.normal = {0.0f, 0.0f, 1.0f};
  cell.u_axis = {1.0f, 0.0f, 0.0f};
  cell.v_axis = {0.0f, 1.0f, 0.0f};
  cell.width_m = 0.1f;
  cell.height_m = 0.1f;
  solar::CellGrid grid;
  require(!solar::validate_shell_cell(cell, top, full, grid),
          "cell under canopy was accepted");

  solar::TraceScene roof_full(roof);
  require(solar::validate_shell_cell(cell, top, roof_full, grid),
          "properly-clearanced roof cell was rejected");

  const TriangleMesh coarser_roof = plane(1.002f);
  solar::TraceScene coarser_full(coarser_roof);
  solar::Cell below_full_surface = cell;
  require(!solar::validate_shell_cell(below_full_surface, top, coarser_full,
                                      grid),
          "cell buried below the complete-car surface was accepted");
  require(solar::conform_shell_cell(below_full_surface, top, coarser_full,
                                    grid),
          "cell could not be raised to clear the complete-car surface");
  require(std::abs(below_full_surface.center.z - 1.003f) < 1.0e-5f,
          "cell conformity did not preserve the requested surface clearance");

  TriangleMesh interrupted;
  add_quad(interrupted, -1.0f, -0.025f, -1.0f, 1.0f, 1.0f);
  add_quad(interrupted, 0.025f, 1.0f, -1.0f, 1.0f, 1.0f);
  solar::TraceScene gap_scene(interrupted);
  require(!solar::validate_shell_cell(cell, gap_scene, gap_scene, grid),
          "cell spanning an unsupported gap was accepted");

  solar::Cell edge_cell = cell;
  edge_cell.center.x = 0.96f;
  edge_cell.width_m = 0.2f;
  require(!solar::validate_shell_cell(edge_cell, roof_full, roof_full, grid),
          "cell extending beyond supported shell edge was accepted");
}

}  // namespace

int main() {
  try {
    test_alignment_and_rejection();
    test_canopy_and_footprint_support();
    std::cout << "Placement-surface tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << "Placement-surface test failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
