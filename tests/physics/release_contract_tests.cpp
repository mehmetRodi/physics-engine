#include "physics/StateSnapshot.hpp"
#include "physics/World.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

TEST(ReleaseContracts, CoincidentSpheresSeparateWithoutNonfiniteState) {
  World world(Vec3(0, 0, 0));
  auto a = world.createRigidBody(1.f, 1.f);
  auto b = world.createRigidBody(1.f, 1.f);
  world.step(0.f);
  EXPECT_EQ(world.collisionPipelineStats().contactCount, 1u);
  EXPECT_GT(world.rigidBody(a).position.x, world.rigidBody(b).position.x);
  EXPECT_LE((world.rigidBody(a).position - world.rigidBody(b).position).length(), .201f);
  for (int i = 0; i < 60; ++i)
    world.step(1.f / 60);
  EXPECT_GT((world.rigidBody(a).position - world.rigidBody(b).position).length(), 1.99f);
  EXPECT_NO_THROW(world.rigidBody(a).validate());
}

TEST(ReleaseContracts, NearlyCoincidentSpheresProduceFiniteResponse) {
  for (float offset : {0.f, 1e-9f, 1e-6f, 1e-4f}) {
    World world{Vec3()};
    auto a = world.createRigidBody(1.f, 1.f), b = world.createRigidBody(1.f, 1.f);
    world.rigidBody(b).position = Vec3(0, offset, 0);
    world.step(0.f);
    EXPECT_EQ(world.collisionPipelineStats().contactCount, 1u);
    EXPECT_NO_THROW(world.rigidBody(a).validate());
    EXPECT_GT((world.rigidBody(a).position - world.rigidBody(b).position).length(), offset);
  }
}

TEST(ReleaseContracts, MassAndRadiusChangesMaintainDerivedProperties) {
  RigidBody body(1.f, 1.f);
  body.setMass(2.f);
  body.applyForce(Vec3(2, 0, 0));
  body.update(1.f);
  EXPECT_FLOAT_EQ(body.velocity.x, 1.f);
  EXPECT_FLOAT_EQ(body.invMass(), .5f);
  EXPECT_FLOAT_EQ(body.invInertiaWorld().m[0][0], 1.25f);
  body.setSphereRadius(2.f);
  EXPECT_FLOAT_EQ(body.invInertiaLocal().m[0][0], .3125f);
  const auto before = snapshotBody(body);
  EXPECT_THROW(body.setMass(-1.f), std::invalid_argument);
  EXPECT_THROW(body.setSphereRadius(0.f), std::invalid_argument);
  EXPECT_EQ(snapshotBody(body), before);
}

TEST(ReleaseContracts, StaticBodiesDoNotIntegrateAssignedMotion) {
  RigidBody body(1.f, 1.f);
  body.velocity = Vec3(1, 2, 3);
  body.setMass(0.f);
  EXPECT_FLOAT_EQ(body.velocity.lengthSq(), 0.f);
  body.velocity = Vec3(1, 2, 3);
  body.angularVelocity = Vec3(3, 2, 1);
  body.acceleration = Vec3(1, 1, 1);
  body.applyForce(Vec3(10, 0, 0));
  body.applyTorque(Vec3(0, 10, 0));
  body.update(1.f);
  EXPECT_FLOAT_EQ(body.position.lengthSq(), 0.f);
  EXPECT_FLOAT_EQ(body.velocity.lengthSq(), 0.f);
  EXPECT_FLOAT_EQ(body.angularVelocity.lengthSq(), 0.f);
  body.setMass(2.f);
  EXPECT_FLOAT_EQ(body.invMass(), .5f);
  EXPECT_FLOAT_EQ(body.invInertiaWorld().m[1][1], 1.25f);
}

TEST(ReleaseContracts, InvalidInputsAreRejectedBeforeWorldAdvances) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(RigidBody(-1.f, 1.f), std::invalid_argument);
  EXPECT_THROW(RigidBody(1.f, nan), std::invalid_argument);
  EXPECT_THROW(RigidBody(nan, 1.f), std::invalid_argument);
  EXPECT_THROW(RigidBody(std::numeric_limits<float>::denorm_min(), 1.f), std::invalid_argument);
  World world{Vec3()};
  auto a = world.createRigidBody(1.f, 1.f), b = world.createRigidBody(1.f, 1.f);
  world.rigidBody(a).velocity = Vec3(1, 0, 0);
  world.rigidBody(b).material.friction = -1.f;
  EXPECT_THROW(world.step(.1f), std::invalid_argument);
  EXPECT_FLOAT_EQ(world.rigidBody(a).position.x, 0.f);
  EXPECT_THROW(world.step(-1.f), std::invalid_argument);
  EXPECT_THROW(world.step(nan), std::invalid_argument);
  EXPECT_THROW(world.rigidBody(100), std::out_of_range);
  ContactSolverSettings settings;
  settings.positionCorrectionRate = -1.f;
  EXPECT_THROW(world.setContactSolverSettings(settings), std::invalid_argument);
}

TEST(ReleaseContracts, BodyReservationIsLinearAndCollisionStorageGrowsWithoutDroppingPairs) {
  World small{Vec3()}, large{Vec3()};
  small.reserveRigidBodies(1024);
  large.reserveRigidBodies(2048);
  EXPECT_LT(small.reservedStorageBytes(), 1024u * 512u);
  EXPECT_LT(large.reservedStorageBytes(), small.reservedStorageBytes() * 3);
  World world{Vec3()};
  world.reserveRigidBodies(8);
  world.reserveCollisionCapacity(1, 1);
  const auto before = world.reservedStorageBytes();
  for (int i = 0; i < 8; ++i)
    world.createRigidBody(1.f, 1.f);
  world.step(0.f);
  EXPECT_EQ(world.collisionPipelineStats().aabbCandidatePairCount, 28u);
  EXPECT_EQ(world.collisionPipelineStats().contactCount, 28u);
  EXPECT_GT(world.reservedStorageBytes(), before);
  EXPECT_THROW(world.reserveRigidBodies(std::numeric_limits<std::size_t>::max()),
               std::length_error);
  AABBSweepAndPruneScratch scratch;
  EXPECT_THROW(scratch.reserve(std::numeric_limits<std::size_t>::max()), std::length_error);
}

TEST(ReleaseContracts, SnapshotsDistinguishSignedZero) {
  RigidBody a(1, 1), b(1, 1);
  a.position.x = 0.f;
  b.position.x = -0.f;
  EXPECT_NE(snapshotBody(a), snapshotBody(b));
}

TEST(ReleaseContracts, CoincidentPolicyMatchesAcrossWorkerCountsEveryTick) {
  World serial{Vec3()}, parallel{Vec3()};
  parallel.setWorkerThreadCount(4);
  for (std::size_t i = 0; i < 512; ++i) {
    auto a = serial.createRigidBody(1.f, 1.f);
    auto b = parallel.createRigidBody(1.f, 1.f);
    serial.rigidBody(a).position = Vec3(float(i / 2) * 4.f, 0, 0);
    parallel.rigidBody(b).position = serial.rigidBody(a).position;
  }
  for (int tick = 0; tick < 30; ++tick) {
    serial.step(1.f / 60);
    parallel.step(1.f / 60);
    for (std::size_t i = 0; i < 512; ++i)
      ASSERT_EQ(snapshotBody(serial.rigidBody(i)), snapshotBody(parallel.rigidBody(i)))
          << "tick " << tick << " body " << i;
  }
}
