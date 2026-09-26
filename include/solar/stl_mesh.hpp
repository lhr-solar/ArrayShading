#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include "solar/core.hpp"

namespace solar {

struct TriangleMesh {
  std::vector<Vec3> vertices;
  Bounds bounds;

  std::size_t triangle_count() const { return vertices.size() / 3U; }
};

TriangleMesh load_binary_stl(const std::filesystem::path& path,
                             const Transform& transform,
                             std::size_t max_triangles = 0);

}  // namespace solar
