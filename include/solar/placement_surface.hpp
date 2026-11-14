#pragma once

#include <cstddef>

#include "solar/simulation.hpp"
#include "solar/trace_scene.hpp"

namespace solar {

struct PlacementSurfaceSettings {
  float alignment_tolerance_m = 0.005f;
  float minimum_upward_normal_z = 0.75f;
};

struct PlacementSurface {
  TriangleMesh mesh;
  ShellRegion region;
  std::size_t checked_samples = 0;
  std::size_t matched_samples = 0;
  double max_alignment_error_m = 0.0;
};

// Keeps only upward-facing top-shell triangles whose centroids coincide with
// the exterior roof of the complete car mesh. Both meshes remain in the
// caller's world coordinates; no registration or scaling is performed.
PlacementSurface prepare_placement_surface(
    const TriangleMesh& top_shell, const TraceScene& full_car,
    const PlacementSurfaceSettings& settings = {});

// Confirms sampled points across the projected cell footprint have matching
// support in both the placement-only and complete-car scenes, and that the
// cell has positive clearance from the complete-car surface.
bool validate_shell_cell(const Cell& cell, const TraceScene& top_scene,
                         const TraceScene& full_scene, const CellGrid& grid,
                         float tolerance_m = 0.005f);

// Raises the cell along its normal as needed to clear the complete-car surface
// while retaining the requested surface offset. Returns false when the
// footprint is unsupported, misaligned, or would exceed the support-plane
// deviation allowance.
bool conform_shell_cell(Cell& cell, const TraceScene& top_scene,
                        const TraceScene& full_scene, const CellGrid& grid,
                        float tolerance_m = 0.005f);

}  // namespace solar
