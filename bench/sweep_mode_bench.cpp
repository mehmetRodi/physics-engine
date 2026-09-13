#include "collision/AABBPair.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
bool csvOutput = false;
using Clock = std::chrono::steady_clock;

enum class Distribution {
  Sparse,
  Dense,
  AllOverlapping,
};

std::string_view distributionName(Distribution distribution) {
  switch (distribution) {
  case Distribution::Sparse:
    return "sparse";
  case Distribution::Dense:
    return "dense";
  case Distribution::AllOverlapping:
    return "all_overlapping";
  }

  return "unknown";
}

std::string_view modeName(SweepAndPruneMode mode) {
  switch (mode) {
  case SweepAndPruneMode::ScalarIndirect:
    return "scalar_indirect";
  case SweepAndPruneMode::ScalarPacked:
    return "scalar_packed";
  case SweepAndPruneMode::Simd:
    return "simd";
  }

  return "unknown";
}

std::vector<AABBProxy> createProxies(std::size_t count, Distribution distribution) {
  std::vector<AABBProxy> proxies;
  proxies.reserve(count);

  const float spacing = distribution == Distribution::Sparse ? 2.0f : 0.4f;

  for (std::size_t i = 0; i < count; ++i) {
    Vec3 centre(0.f, 0.f, 0.f);

    if (distribution != Distribution::AllOverlapping) {
      centre =
          Vec3(static_cast<float>(i % 64) * spacing, static_cast<float>(i / 64) * spacing, 0.f);
    }

    proxies.push_back({i, makeAABBForSphere(centre, 0.25f)});
  }

  return proxies;
}

double medianNanoseconds(std::vector<std::uint64_t>& samples) {
  std::sort(samples.begin(), samples.end());
  return static_cast<double>(samples[samples.size() / 2]);
}

void runCase(std::size_t proxyCount, Distribution distribution, SweepAndPruneMode mode, int warmup,
             int iterations) {
  const std::vector<AABBProxy> proxies = createProxies(proxyCount, distribution);

  AABBSweepAndPruneScratch scratch;
  scratch.reserve(proxyCount);
  std::vector<AABBPair> pairs;

  for (int i = 0; i < warmup; ++i) {
    findAABBPairsSweepAndPrune(proxies, scratch, pairs, mode);
  }

  std::vector<std::uint64_t> samples;
  samples.reserve(static_cast<std::size_t>(iterations));

  for (int i = 0; i < iterations; ++i) {
    const auto start = Clock::now();
    findAABBPairsSweepAndPrune(proxies, scratch, pairs, mode);
    const auto end = Clock::now();
    samples.push_back(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()));
  }

  if (csvOutput) {
    for (std::size_t i = 0; i < samples.size(); ++i)
      std::cout << distributionName(distribution) << ',' << modeName(mode) << ',' << proxyCount
                << ',' << i << ',' << samples[i] << ',' << pairs.size() << '\n';
    return;
  }
  std::cout << "sweep mode benchmark\n";
  std::cout << "distribution: " << distributionName(distribution) << '\n';
  std::cout << "mode: " << modeName(mode) << '\n';
  std::cout << "proxy_count: " << proxyCount << '\n';
  std::cout << "pair_count: " << pairs.size() << '\n';
  std::cout << "median_ns: " << medianNanoseconds(samples) << '\n';
  std::cout << '\n';
}
} // namespace

int main(int argc, char** argv) {
  csvOutput = argc == 2 && std::string_view(argv[1]) == "--csv";
  if (argc > 1 && !csvOutput) {
    std::cerr << "usage: sweep_mode_bench [--csv]\n";
    return 2;
  }
  constexpr std::size_t proxyCount = 1024;
  constexpr int warmup = 50;
  constexpr int iterations = 300;

  if (csvOutput)
    std::cout << "distribution,mode,proxy_count,sample,ns,pairs\n";
  else
    std::cout << "explicit_simd_path: " << (hasSimdSweepAndPrune() ? "yes" : "no") << "\n\n";

  for (const Distribution distribution :
       {Distribution::Sparse, Distribution::Dense, Distribution::AllOverlapping}) {
    for (const SweepAndPruneMode mode :
         {SweepAndPruneMode::ScalarIndirect, SweepAndPruneMode::ScalarPacked,
          SweepAndPruneMode::Simd}) {
      runCase(proxyCount, distribution, mode, warmup, iterations);
    }
  }

  return 0;
}
