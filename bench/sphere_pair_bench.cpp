#include "collision/SpherePair.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

std::vector<SphereProxy> createSphereProxies(std::size_t bodyCount) {
  std::vector<SphereProxy> proxies;
  proxies.reserve(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    const float x = static_cast<float>(i % 64) * 0.75f;
    const float y = static_cast<float>(i / 64) * 0.75f;
    proxies.push_back({i, Vec3(x, y, 0.0f), 0.5f});
  }

  return proxies;
}
} // namespace

int main() {
  constexpr std::size_t bodyCount = 1024;
  constexpr int iterations = 100;

  const std::vector<SphereProxy> proxies = createSphereProxies(bodyCount);
  const std::size_t pairsPerIteration = bodyCount * (bodyCount - 1) / 2;

  std::vector<CollisionPair> pairs;
  pairs.reserve(pairsPerIteration);

  std::size_t checksum = 0;

  const auto start = Clock::now();

  for (int i = 0; i < iterations; ++i) {
    findSpherePairs(proxies, pairs);
    checksum += pairs.size();
  }

  const auto end = Clock::now();

  const auto totalNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

  const std::size_t totalPairChecks = pairsPerIteration * iterations;
  const double nanosecondsPerPairCheck =
      static_cast<double>(totalNanoseconds) / static_cast<double>(totalPairChecks);

  std::cout << "Sphere pair-check benchmark\n";
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "iterations: " << iterations << '\n';
  std::cout << "pair_checks_per_iteration: " << pairsPerIteration << '\n';
  std::cout << "total_pair_checks: " << totalPairChecks << '\n';
  std::cout << "total_ns: " << totalNanoseconds << '\n';
  std::cout << "ns_per_pair_check: " << nanosecondsPerPairCheck << '\n';
  std::cout << "checksum: " << checksum << '\n';

  return 0;
}
