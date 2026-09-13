#pragma once
#include "physics/World.hpp"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bench {
using Clock = std::chrono::steady_clock;
struct Options {
  std::size_t bodies = 352;
  unsigned threads = 1;
  int warmup = 300;
  int samples = 600;
  int iterations = 8;
  bool warmStarting = true;
  bool csv = false;
  std::string scene = "columns";
};
inline unsigned integer(std::string_view text) {
  unsigned value = 0;
  auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc() || result.ptr != text.data() + text.size())
    throw std::invalid_argument("invalid integer");
  return value;
}
inline Options options(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string_view key = argv[i];
    if (key == "--csv") {
      o.csv = true;
      continue;
    }
    if (key == "--help") {
      std::cout << "--bodies N --threads N --warmup N --samples N --iterations N --warm-start 0|1 "
                   "--scene columns|sparse|cluster --csv\n";
      std::exit(0);
    }
    if (i + 1 == argc)
      throw std::invalid_argument("missing option value");
    const std::string_view value = argv[++i];
    if (key == "--scene") {
      o.scene = value;
      continue;
    }
    const unsigned n = integer(value);
    if (key == "--bodies")
      o.bodies = n;
    else if (key == "--threads")
      o.threads = n;
    else if (key == "--warmup")
      o.warmup = static_cast<int>(n);
    else if (key == "--samples")
      o.samples = static_cast<int>(n);
    else if (key == "--iterations")
      o.iterations = static_cast<int>(n);
    else if (key == "--warm-start" && n <= 1)
      o.warmStarting = n != 0;
    else
      throw std::invalid_argument("unknown option/value");
  }
  if (o.bodies < 2 || o.bodies > 100000 || o.threads == 0 || o.threads > 256 || o.warmup < 0 ||
      o.warmup > 1000000 || o.samples < 1 || o.samples > 1000000 || o.iterations < 1 ||
      o.iterations > 1024 || (o.scene != "columns" && o.scene != "sparse" && o.scene != "cluster"))
    throw std::invalid_argument("option outside supported range");
  return o;
}
inline World scene(const Options& o) {
  World w(o.scene == "columns" ? Vec3(0, -9.8f, 0) : Vec3());
  w.reserveRigidBodies(o.bodies);

  w.reserveCollisionCapacity(o.bodies * 8, o.bodies * 4);
  w.setWorkerThreadCount(o.threads);
  ContactSolverSettings settings;
  settings.velocityIterations = o.iterations;
  settings.warmStarting = o.warmStarting;
  w.setContactSolverSettings(settings);
  for (std::size_t i = 0; i < o.bodies; ++i) {
    const bool pedestal = o.scene == "columns" && i % 11 == 0;
    auto id = w.createRigidBody(pedestal ? 0.f : 1.f, .5f);
    auto& b = w.rigidBody(id);
    b.material.restitution = 0.f;
    b.material.friction = .4f;
    if (o.scene == "columns")
      b.position = Vec3(float(i / 11) * 2.f, -.5f + float(i % 11) * .998f, 0);
    else if (o.scene == "sparse")
      b.position = Vec3(float(i % 64) * 4.f, float(i / 64) * 4.f, 0);
    else {
      b.position = Vec3(float(i % 8) * .7f, float((i / 8) % 8) * .7f, float(i / 64) * .7f);
      b.velocity = Vec3(float(int(i % 7) - 3) * .1f, float(int(i % 5) - 2) * .1f,
                        float(int(i % 3) - 1) * .1f);
    }
  }
  return w;
}
inline float compression(const World& world, const Options& o) {
  if (o.scene != "columns")
    return 0.f;
  float worst = 0.f;
  for (std::size_t top = 10; top < world.bodyCount(); top += 11)
    worst = std::max(worst, 9.48f - world.rigidBody(top).position.y);
  return worst;
}
inline std::uint64_t elapsed(Clock::time_point start, Clock::time_point end) {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
}
inline std::uint64_t percentile(std::vector<std::uint64_t> values, double p) {
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(p * double(values.size() - 1))];
}
inline int worldBenchmark(int argc, char** argv) {
  try {
    const Options o = options(argc, argv);
    World world = scene(o);
    for (int i = 0; i < o.warmup; ++i)
      world.step(1.f / 60);
    struct Sample {
      std::uint64_t ns;
      std::size_t pairs, contacts;
      float compression;
    };
    std::vector<Sample> samples;
    samples.reserve(o.samples);
    for (int i = 0; i < o.samples; ++i) {
      const auto start = Clock::now();
      world.step(1.f / 60);
      const auto end = Clock::now();
      const auto stats = world.collisionPipelineStats();
      samples.push_back({elapsed(start, end), stats.aabbCandidatePairCount, stats.contactCount,
                         compression(world, o)});
    }
    if (o.csv) {
      std::cout << "sample,ns,pairs,contacts,compression_m,reserved_bytes\n";
      for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto& s = samples[i];
        std::cout << i << ',' << s.ns << ',' << s.pairs << ',' << s.contacts << ',' << s.compression
                  << ',' << world.reservedStorageBytes() << '\n';
      }
    } else {
      std::vector<std::uint64_t> times;
      for (const auto& s : samples)
        times.push_back(s.ns);
      std::cout << "scene=" << o.scene << " bodies=" << world.bodyCount()
                << " threads=" << o.threads << " iterations=" << o.iterations
                << " warm_start=" << o.warmStarting << " warmup=" << o.warmup
                << " samples=" << o.samples << "\nmedian_ns=" << percentile(times, .5)
                << " p95_ns=" << percentile(times, .95) << " p99_ns=" << percentile(times, .99)
                << "\ncontacts=" << samples.back().contacts
                << " compression_m=" << samples.back().compression
                << " reserved_bytes=" << world.reservedStorageBytes() << '\n';
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
} // namespace bench
