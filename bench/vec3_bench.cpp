#include "math/Vec3.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

std::vector<Vec3> createVectors(std::size_t count, float offset) {
  std::vector<Vec3> vectors;
  vectors.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const float value = static_cast<float>(i) + offset;
    vectors.emplace_back(value, value * 0.5f, value * 0.25f);
  }

  return vectors;
}
} // namespace

int main() {
  constexpr std::size_t vectorCount = 1'000'000;

  const std::vector<Vec3> lhs = createVectors(vectorCount, 1.0f);
  const std::vector<Vec3> rhs = createVectors(vectorCount, 2.0f);

  float checksum = 0.0f;

  const auto start = Clock::now();

  for (std::size_t i = 0; i < vectorCount; ++i) {
    checksum += lhs[i].dot(rhs[i]);
  }

  const auto end = Clock::now();

  const auto totalNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

  const double nanosecondsPerDot =
      static_cast<double>(totalNanoseconds) / static_cast<double>(vectorCount);

  std::cout << "Vec3::dot benchmark\n";
  std::cout << "vector_count: " << vectorCount << '\n';
  std::cout << "total_ns: " << totalNanoseconds << '\n';
  std::cout << "ns_per_dot: " << nanosecondsPerDot << '\n';
  std::cout << "checksum: " << checksum << '\n';

  return 0;
}
