#include "solar/core.hpp"
#include "solar/simulation.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

bool close(float a, float b, float tolerance = 1.0e-5f) {
  return std::abs(a - b) <= tolerance;
}

}  // namespace

int main() {
  using solar::Vec3;
  const Vec3 n = solar::normalized({3.0f, 4.0f, 0.0f});
  assert(close(n.x, 0.6f));
  assert(close(n.y, 0.8f));
  assert(close(solar::dot(solar::cross({1, 0, 0}, {0, 1, 0}),
                          {0, 0, 1}),
               1.0f));

  solar::Transform transform;
  const Vec3 transformed = transform.apply({0.0f, 0.0f, 1000.0f});
  assert(close(transformed.x, 0.0f));
  assert(close(transformed.y, -1.6563f));
  assert(close(transformed.z, 0.0153f));

  const Vec3 overhead = solar::sun_direction_from_azimuth_elevation(42.0f, 90.0f);
  assert(close(overhead.x, 0.0f));
  assert(close(overhead.y, 0.0f));
  assert(close(overhead.z, 1.0f));

  for (std::uint32_t i = 0; i < 64; ++i) {
    const Vec3 direction = solar::cosine_hemisphere(i, 64, {0, 0, 1});
    assert(close(solar::length(direction), 1.0f));
    assert(direction.z >= 0.0f);
  }
  std::cout << "core tests passed\n";
}
