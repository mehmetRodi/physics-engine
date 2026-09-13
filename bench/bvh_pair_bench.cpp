#include "collision/AABB.hpp"
#include "collision/AABBPair.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

enum class Distribution {
  SparseGrid,
  DenseGrid,
  AllOverlapping,
};

struct BVHStageTimingTotals {
  std::uint64_t buildNs = 0;
  std::uint64_t traversalNs = 0;
  std::uint64_t sortAndEmitNs = 0;

  std::uint64_t totalNs() const {
    return buildNs + traversalNs + sortAndEmitNs;
  }
};

std::string_view distributionName(Distribution distribution) {
  switch (distribution) {
  case Distribution::SparseGrid:
    return "sparse_grid";
  case Distribution::DenseGrid:
    return "dense_grid";
  case Distribution::AllOverlapping:
    return "all_overlapping";
  }

  return "unknown";
}

std::vector<AABBProxy> createAABBProxies(std::size_t bodyCount, Distribution distribution) {
  std::vector<AABBProxy> proxies;
  proxies.reserve(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    Vec3 center(0.0f, 0.0f, 0.0f);

    if (distribution != Distribution::AllOverlapping) {
      const float spacing = distribution == Distribution::SparseGrid ? 3.0f : 0.75f;
      center =
          Vec3(static_cast<float>(i % 64) * spacing, static_cast<float>(i / 64) * spacing, 0.0f);
    }

    proxies.push_back({i, makeAABBForSphere(center, 0.5f)});
  }

  return proxies;
}

std::size_t pairChecksForBodyCount(std::size_t bodyCount) {
  return bodyCount * (bodyCount - 1) / 2;
}

int iterationsForBodyCount(std::size_t bodyCount) {
  constexpr std::size_t targetPairChecks = 20'000'000;
  const std::size_t pairChecks = pairChecksForBodyCount(bodyCount);
  return std::max(1, static_cast<int>(targetPairChecks / pairChecks));
}

template <typename Function> std::uint64_t measureNanoseconds(Function&& function) {
  const auto start = Clock::now();
  function();
  const auto end = Clock::now();

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
}

void runCase(std::size_t bodyCount, Distribution distribution) {
  const std::vector<AABBProxy> proxies = createAABBProxies(bodyCount, distribution);
  const std::size_t possiblePairsPerIteration = pairChecksForBodyCount(bodyCount);
  const int iterations = iterationsForBodyCount(bodyCount);

  AABBBVHScratch scratch;
  scratch.reserve(bodyCount);

  std::vector<AABBPair> pairs;
  pairs.reserve(possiblePairsPerIteration);

  BVHStageTimingTotals timings;
  std::size_t checksum = 0;
  std::size_t lastCandidatePairs = 0;

  for (int i = 0; i < iterations; ++i) {
    AABBBVHBuildResult buildResult;

    timings.buildNs += measureNanoseconds([&]() { buildResult = buildAABBBVH(proxies, scratch); });

    timings.traversalNs += measureNanoseconds([&]() {
      if (!buildResult.empty) {
        collectAABBBVHPairCandidates(scratch, buildResult.rootNode,
                                     scratch.candidateProxyIndexPairs);
      } else {
        scratch.candidateProxyIndexPairs.clear();
      }
    });

    timings.sortAndEmitNs += measureNanoseconds([&]() {
      emitSortedAABBPairsFromProxyIndexPairs(proxies, scratch.candidateProxyIndexPairs, pairs);
    });

    lastCandidatePairs = pairs.size();
    checksum += pairs.size();
  }

  const double measuredIterations = static_cast<double>(iterations);

  std::cout << "BVH pair benchmark\n";
  std::cout << "case: " << distributionName(distribution) << '\n';
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "iterations: " << iterations << '\n';
  std::cout << "possible_pairs_per_iteration: " << possiblePairsPerIteration << '\n';
  std::cout << "last_candidate_pairs: " << lastCandidatePairs << '\n';
  std::cout << "avg_build_ns: " << static_cast<double>(timings.buildNs) / measuredIterations
            << '\n';
  std::cout << "avg_traversal_ns: " << static_cast<double>(timings.traversalNs) / measuredIterations
            << '\n';
  std::cout << "avg_sort_and_emit_ns: "
            << static_cast<double>(timings.sortAndEmitNs) / measuredIterations << '\n';
  std::cout << "avg_total_ns: " << static_cast<double>(timings.totalNs()) / measuredIterations
            << '\n';
  std::cout << "checksum: " << checksum << '\n';
  std::cout << '\n';
}
} // namespace

int main() {
  constexpr std::array<std::size_t, 3> bodyCounts = {128, 512, 1024};
  constexpr std::array<Distribution, 3> distributions = {
      Distribution::SparseGrid,
      Distribution::DenseGrid,
      Distribution::AllOverlapping,
  };

  for (const std::size_t bodyCount : bodyCounts) {
    for (const Distribution distribution : distributions) {
      runCase(bodyCount, distribution);
    }
  }

  return 0;
}
