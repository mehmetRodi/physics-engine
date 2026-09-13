#include "core/JobSystem.hpp"
#include "physics/World.hpp"

#include <algorithm>
#include <atomic>
#include <barrier>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <new>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

constexpr float kDt = 1.0f / 60.0f;

World createScene(std::size_t bodyCount, unsigned threadCount) {
  World world(Vec3(0.f, -9.8f, 0.f));
  world.reserveRigidBodies(bodyCount + 1);
  world.setWorkerThreadCount(threadCount);

  const World::RigidBodyId ground = world.createRigidBody(0.0f, 2000.0f);
  world.rigidBody(ground).position = Vec3(0.f, -2000.f, 0.f);

  const std::size_t perRow = 64;
  for (std::size_t i = 0; i < bodyCount; ++i) {
    const World::RigidBodyId body = world.createRigidBody(1.0f, 0.25f);
    world.rigidBody(body).position = Vec3(static_cast<float>(i % perRow) * 0.45f,
                                          0.25f + static_cast<float>(i / perRow) * 0.45f, 0.f);
    world.rigidBody(body).material.restitution = 0.1f;
  }

  return world;
}

double medianNanoseconds(std::vector<std::uint64_t>& samples) {
  std::sort(samples.begin(), samples.end());
  return static_cast<double>(samples[samples.size() / 2]);
}

void runStageScalingCase(std::size_t bodyCount, unsigned threadCount, int warmup, int measured) {
  std::vector<RigidBody> bodies;
  bodies.reserve(bodyCount);
  for (std::size_t i = 0; i < bodyCount; ++i) {
    bodies.emplace_back(1.0f, 0.25f);
    bodies.back().angularVelocity = Vec3(0.3f, 0.1f, -0.2f);
  }

  JobSystem jobs(threadCount);

  const auto integrate = [&bodies](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      bodies[i].applyForce(Vec3(0.f, -9.8f, 0.f) * bodies[i].mass());
      bodies[i].update(kDt);
    }
  };

  for (int i = 0; i < warmup; ++i) {
    jobs.parallelFor(bodyCount, 64, integrate);
  }

  std::vector<std::uint64_t> samples;
  samples.reserve(static_cast<std::size_t>(measured));

  for (int i = 0; i < measured; ++i) {
    const auto start = Clock::now();
    jobs.parallelFor(bodyCount, 64, integrate);
    const auto end = Clock::now();
    samples.push_back(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()));
  }

  std::cout << "integration stage scaling\n";
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "threads: " << jobs.threadCount() << '\n';
  std::cout << "median_ns: " << medianNanoseconds(samples) << '\n';
  std::cout << '\n';
}

void runScalingCase(std::size_t bodyCount, unsigned threadCount, int warmup, int measured) {
  World world = createScene(bodyCount, threadCount);

  for (int i = 0; i < warmup; ++i) {
    world.step(kDt);
  }

  std::vector<std::uint64_t> samples;
  samples.reserve(static_cast<std::size_t>(measured));

  for (int i = 0; i < measured; ++i) {
    const auto start = Clock::now();
    world.step(kDt);
    const auto end = Clock::now();
    samples.push_back(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()));
  }

  std::cout << "job system scaling\n";
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "threads: " << world.workerThreadCount() << '\n';
  std::cout << "contacts: " << world.collisionPipelineStats().contactCount << '\n';
  std::cout << "median_ns: " << medianNanoseconds(samples) << '\n';
  std::cout << '\n';
}

struct PackedCounter {
  std::atomic<std::uint64_t> value{0};
};

struct alignas(128) PaddedCounter {
  std::atomic<std::uint64_t> value{0};
};

template <typename Counter>
double measureCounters(JobSystem& jobs, std::size_t iterations, int repeats) {
  std::vector<Counter> counters(jobs.threadCount());
  std::barrier rendezvous(static_cast<std::ptrdiff_t>(jobs.threadCount()));

  std::vector<std::uint64_t> samples;
  samples.reserve(static_cast<std::size_t>(repeats));

  for (int repeat = 0; repeat < repeats; ++repeat) {
    const auto start = Clock::now();
    jobs.parallelFor(jobs.threadCount(), 1, [&](std::size_t slot, std::size_t) {
      rendezvous.arrive_and_wait();

      for (std::size_t i = 0; i < iterations; ++i) {
        counters[slot].value.fetch_add(i, std::memory_order_relaxed);
      }
    });
    const auto end = Clock::now();

    samples.push_back(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()));
  }

  return medianNanoseconds(samples);
}

void runFalseSharingCase(unsigned threadCount) {
  JobSystem jobs(threadCount);

  constexpr std::size_t iterations = 500000;
  constexpr int repeats = 30;

  const double packed = measureCounters<PackedCounter>(jobs, iterations, repeats);
  const double padded = measureCounters<PaddedCounter>(jobs, iterations, repeats);

  std::cout << "false sharing\n";
  std::cout << "threads: " << jobs.threadCount() << '\n';
  std::cout << "increments_per_thread: " << iterations << '\n';
  std::cout << "packed_counters_ns: " << packed << '\n';
  std::cout << "padded_counters_ns: " << padded << '\n';
  std::cout << "packed_over_padded: " << packed / padded << '\n';
  std::cout << '\n';
}
} // namespace

int main() {
  constexpr int warmup = 100;
  constexpr int measured = 400;

  for (const std::size_t bodyCount : {std::size_t{8192}, std::size_t{65536}}) {
    for (const unsigned threadCount : {1u, 2u, 4u, 8u}) {
      runStageScalingCase(bodyCount, threadCount, warmup, measured);
    }
  }

  for (const std::size_t bodyCount : {std::size_t{1024}, std::size_t{8192}}) {
    for (const unsigned threadCount : {1u, 2u, 4u, 8u}) {
      runScalingCase(bodyCount, threadCount, warmup, measured);
    }
  }

  for (const unsigned threadCount : {2u, 4u, 8u}) {
    runFalseSharingCase(threadCount);
  }

  return 0;
}
