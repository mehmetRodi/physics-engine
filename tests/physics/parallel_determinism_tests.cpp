#include <gtest/gtest.h>

#include "physics/StateSnapshot.hpp"
#include "physics/World.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

constexpr float kDt = 1.0f / 60.0f;

World createScene(unsigned threadCount) {
  World world(Vec3(0.f, -9.8f, 0.f));
  world.setWorkerThreadCount(threadCount);

  constexpr std::size_t bodiesPerRow = 24;
  constexpr std::size_t rows = 16;
  world.reserveRigidBodies(bodiesPerRow * rows + 1);

  const World::RigidBodyId ground = world.createRigidBody(0.0f, 200.0f);
  world.rigidBody(ground).position = Vec3(0.f, -200.f, 0.f);

  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t column = 0; column < bodiesPerRow; ++column) {
      const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);

      const float jitter = static_cast<float>((row * 7 + column * 13) % 11) * 0.01f;
      world.rigidBody(body).position = Vec3(static_cast<float>(column) * 0.98f + jitter,
                                            0.5f + static_cast<float>(row) * 0.97f, jitter * 0.5f);
      world.rigidBody(body).material.restitution = 0.2f;
      world.rigidBody(body).material.friction = 0.4f;
    }
  }

  return world;
}

std::vector<BodyStateWords> run(unsigned threadCount, int steps) {
  World world = createScene(threadCount);
  std::vector<BodyStateWords> states;
  states.reserve(world.bodyCount() * static_cast<std::size_t>(steps));
  for (int step = 0; step < steps; ++step) {
    world.step(kDt);
    for (std::size_t i = 0; i < world.bodyCount(); ++i)
      states.push_back(snapshotBody(world.rigidBody(i)));
  }
  return states;
}

void expectBitIdentical(const std::vector<BodyStateWords>& actual,
                        const std::vector<BodyStateWords>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    ASSERT_EQ(actual[i], expected[i]) << "first divergent tick " << i / 385 << ", body " << i % 385;
  }
}

} // namespace

TEST(ParallelDeterminismTests, ParallelStepMatchesSerialStepExactly) {
  const std::vector<BodyStateWords> serial = run(1, 120);

  for (const unsigned threadCount : {2u, 3u, 4u, 8u}) {
    SCOPED_TRACE(threadCount);
    expectBitIdentical(run(threadCount, 120), serial);
  }
}

TEST(ParallelDeterminismTests, RepeatedParallelRunsAgreeWithEachOther) {

  const std::vector<BodyStateWords> first = run(4, 90);

  for (int attempt = 0; attempt < 5; ++attempt) {
    SCOPED_TRACE(attempt);
    expectBitIdentical(run(4, 90), first);
  }
}

TEST(ParallelDeterminismTests, ContactCountIsTheSameWhicheverWayItIsRun) {
  World serial = createScene(1);
  World parallel = createScene(4);

  for (int step = 0; step < 60; ++step) {
    serial.step(kDt);
    parallel.step(kDt);

    ASSERT_EQ(serial.collisionPipelineStats().aabbCandidatePairCount,
              parallel.collisionPipelineStats().aabbCandidatePairCount);
    ASSERT_EQ(serial.collisionPipelineStats().contactCount,
              parallel.collisionPipelineStats().contactCount);
  }
}

TEST(ParallelDeterminismTests, ThreadCountIsReportedBack) {
  World world(Vec3(0.f, 0.f, 0.f));
  EXPECT_EQ(world.workerThreadCount(), 1u);

  world.setWorkerThreadCount(4);
  EXPECT_EQ(world.workerThreadCount(), 4u);

  world.setWorkerThreadCount(1);
  EXPECT_EQ(world.workerThreadCount(), 1u);
}
