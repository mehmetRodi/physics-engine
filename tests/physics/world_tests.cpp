#include <gtest/gtest.h>

#include "physics/StateSnapshot.hpp"
#include "physics/World.hpp"

#include <array>
#include <cstddef>

namespace {

constexpr std::size_t kReplayBodyCount = 4;
constexpr float kReplayDt = 1.0f / 60.0f;
constexpr int kReplaySteps = 120;

struct ReplayScenario {
  World world;
  std::array<World::RigidBodyId, kReplayBodyCount> bodies{};
};

ReplayScenario createReplayScenario() {
  ReplayScenario scenario{World(Vec3(0.f, -9.8f, 0.f))};
  scenario.world.reserveRigidBodies(kReplayBodyCount);

  scenario.bodies[0] = scenario.world.createRigidBody(1.0f, 0.5f);
  scenario.world.rigidBody(scenario.bodies[0]).position = Vec3(-1.5f, 0.0f, 0.0f);
  scenario.world.rigidBody(scenario.bodies[0]).velocity = Vec3(2.0f, 0.0f, 0.0f);
  scenario.world.rigidBody(scenario.bodies[0]).material.restitution = 0.25f;

  scenario.bodies[1] = scenario.world.createRigidBody(2.0f, 0.5f);
  scenario.world.rigidBody(scenario.bodies[1]).position = Vec3(1.5f, 0.0f, 0.0f);
  scenario.world.rigidBody(scenario.bodies[1]).velocity = Vec3(-1.0f, 0.0f, 0.0f);
  scenario.world.rigidBody(scenario.bodies[1]).material.restitution = 0.75f;

  scenario.bodies[2] = scenario.world.createRigidBody(0.0f, 0.75f);
  scenario.world.rigidBody(scenario.bodies[2]).position = Vec3(0.0f, -3.25f, 0.0f);

  scenario.bodies[3] = scenario.world.createRigidBody(0.5f, 0.35f);
  scenario.world.rigidBody(scenario.bodies[3]).position = Vec3(0.75f, 2.0f, 0.0f);
  scenario.world.rigidBody(scenario.bodies[3]).velocity = Vec3(-0.5f, -0.25f, 0.0f);
  scenario.world.rigidBody(scenario.bodies[3]).material.restitution = 0.5f;

  return scenario;
}

} // namespace

TEST(WorldTests, GravityAffectsDynamicBodyDuringStep) {
  World world(Vec3(0.f, -9.8f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 20.0f);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).velocity.y, -9.8f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).position.y, -9.8f);
}

TEST(WorldTests, ZeroGravityLeavesRestingBodyAtRest) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 20.0f);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).position.y, 0.f);
}

TEST(WorldTests, SameInitialStateProducesSameResult) {
  World world1(Vec3(0.f, -9.8f, 0.f));
  const World::RigidBodyId body1 = world1.createRigidBody(1.0f, 20.0f);

  World world2(Vec3(0.f, -9.8f, 0.f));
  const World::RigidBodyId body2 = world2.createRigidBody(1.0f, 20.0f);

  world1.step(1.0f);
  world2.step(1.0f);

  EXPECT_FLOAT_EQ(world1.rigidBody(body1).velocity.y, world2.rigidBody(body2).velocity.y);
  EXPECT_FLOAT_EQ(world1.rigidBody(body1).position.y, world2.rigidBody(body2).position.y);
}

TEST(WorldTests, FixedScenarioReplayProducesIdenticalFinalState) {
  ReplayScenario replay1 = createReplayScenario();
  ReplayScenario replay2 = createReplayScenario();

  for (int step = 0; step < kReplaySteps; ++step) {
    replay1.world.step(kReplayDt);
    replay2.world.step(kReplayDt);
  }

  for (std::size_t i = 0; i < kReplayBodyCount; ++i) {
    const RigidBody& body1 = replay1.world.rigidBody(replay1.bodies[i]);
    const RigidBody& body2 = replay2.world.rigidBody(replay2.bodies[i]);

    EXPECT_EQ(snapshotBody(body1), snapshotBody(body2));
  }
}

TEST(WorldTests, StaticBodyDoesNotMoveUnderGravity) {
  World world(Vec3(0.f, -9.8f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(0.0f, 20.0f);
  world.step(1.0f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).position.y, 0.f);
}
