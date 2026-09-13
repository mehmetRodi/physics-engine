#include <gtest/gtest.h>

#include "physics/World.hpp"

#include <cmath>

namespace {

constexpr float kDt = 1.0f / 60.0f;
constexpr float kGroundRadius = 500.0f;

World::RigidBodyId addGround(World& world) {
  const World::RigidBodyId ground = world.createRigidBody(0.0f, kGroundRadius);
  world.rigidBody(ground).position = Vec3(0.f, -kGroundRadius, 0.f);
  return ground;
}

} // namespace

TEST(AngularTests, SolidSphereInertiaMatchesTwoFifthsMassRadiusSquared) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(2.0f, 0.5f);

  const float expected = 5.0f / (2.0f * 2.0f * 0.5f * 0.5f);

  EXPECT_FLOAT_EQ(world.rigidBody(body).invInertiaLocal().m[0][0], expected);
  EXPECT_FLOAT_EQ(world.rigidBody(body).invInertiaLocal().m[1][1], expected);
  EXPECT_FLOAT_EQ(world.rigidBody(body).invInertiaLocal().m[2][2], expected);
  EXPECT_FLOAT_EQ(world.rigidBody(body).invInertiaLocal().m[0][1], 0.f);
}

TEST(AngularTests, StaticBodyHasNoInverseInertia) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(0.0f, 0.5f);

  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_FLOAT_EQ(world.rigidBody(body).invInertiaLocal().m[row][column], 0.f);
    }
  }
}

TEST(AngularTests, TorqueSpinsABodyUpAtTheRateTheInertiaAllows) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(2.0f, 0.5f);

  const float inertia = 0.4f * 2.0f * 0.5f * 0.5f;
  world.rigidBody(body).applyTorque(Vec3(0.f, 0.f, inertia));

  world.step(kDt);

  EXPECT_NEAR(world.rigidBody(body).angularVelocity.z, kDt, 1e-6f);
}

TEST(AngularTests, TorqueIsClearedAfterEachStep) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);

  world.rigidBody(body).applyTorque(Vec3(0.f, 0.f, 1.0f));
  world.step(kDt);

  const float afterFirstStep = world.rigidBody(body).angularVelocity.z;
  world.step(kDt);

  EXPECT_FLOAT_EQ(world.rigidBody(body).angularVelocity.z, afterFirstStep);
}

TEST(AngularTests, SpinPersistsWithoutTorque) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);
  world.rigidBody(body).angularVelocity = Vec3(0.f, 3.0f, 0.f);

  for (int step = 0; step < 120; ++step) {
    world.step(kDt);
  }

  EXPECT_FLOAT_EQ(world.rigidBody(body).angularVelocity.y, 3.0f);
  EXPECT_NEAR(world.rigidBody(body).orientation.lengthSq(), 1.0f, 1e-5f);
}

TEST(AngularTests, AngularDampingBleedsSpinAway) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);
  world.rigidBody(body).angularVelocity = Vec3(0.f, 3.0f, 0.f);
  world.rigidBody(body).material.angularDamping = 2.0f;

  for (int step = 0; step < 120; ++step) {
    world.step(kDt);
  }

  EXPECT_LT(world.rigidBody(body).angularVelocity.y, 0.5f);
  EXPECT_GT(world.rigidBody(body).angularVelocity.y, 0.f);
}

TEST(AngularTests, StaticBodyIgnoresTorque) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(0.0f, 0.5f);

  world.rigidBody(body).applyTorque(Vec3(1.f, 2.f, 3.f));
  world.step(kDt);

  EXPECT_FLOAT_EQ(world.rigidBody(body).angularVelocity.x, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).angularVelocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body).angularVelocity.z, 0.f);
}

TEST(AngularTests, ForceThroughTheCentreProducesNoSpin) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);

  world.rigidBody(body).applyForceAtPoint(Vec3(5.f, 0.f, 0.f), world.rigidBody(body).position);
  world.step(kDt);

  EXPECT_NEAR(world.rigidBody(body).angularVelocity.lengthSq(), 0.f, 1e-9f);
  EXPECT_NEAR(world.rigidBody(body).velocity.x, 5.f * kDt, 1e-6f);
}

TEST(AngularTests, OffCentreForceProducesBothTranslationAndSpin) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);

  world.rigidBody(body).applyForceAtPoint(Vec3(5.f, 0.f, 0.f), Vec3(0.f, 0.5f, 0.f));
  world.step(kDt);

  EXPECT_NEAR(world.rigidBody(body).velocity.x, 5.f * kDt, 1e-6f);
  EXPECT_LT(world.rigidBody(body).angularVelocity.z, 0.f);
}

TEST(AngularTests, WorldTensorFollowsTheOrientationForAnIsotropicBody) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);
  world.rigidBody(body).angularVelocity = Vec3(1.3f, -0.7f, 2.1f);

  for (int step = 0; step < 60; ++step) {
    world.step(kDt);
  }

  const RigidBody& value = world.rigidBody(body);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_NEAR(value.invInertiaWorld().m[row][column], value.invInertiaLocal().m[row][column],
                  1e-4f);
    }
  }
}

TEST(AngularTests, IsotropicTensorIsLeftAloneRatherThanRotated) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body = world.createRigidBody(1.0f, 0.5f);
  world.rigidBody(body).angularVelocity = Vec3(1.3f, -0.7f, 2.1f);

  const Mat3 before = world.rigidBody(body).invInertiaWorld();

  for (int step = 0; step < 60; ++step) {
    world.step(kDt);
  }

  const Mat3& after = world.rigidBody(body).invInertiaWorld();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_FLOAT_EQ(after.m[row][column], before.m[row][column]);
    }
  }
}

TEST(AngularTests, SphereSlidingOnTheGroundEndsUpRolling) {
  World world(Vec3(0.f, -9.8f, 0.f));
  addGround(world);

  const World::RigidBodyId ball = world.createRigidBody(1.0f, 0.5f);
  world.rigidBody(ball).position = Vec3(0.f, 0.5f, 0.f);
  world.rigidBody(ball).velocity = Vec3(4.f, 0.f, 0.f);
  world.rigidBody(ball).material.restitution = 0.f;
  world.rigidBody(ball).material.friction = 0.6f;

  for (int step = 0; step < 240; ++step) {
    world.step(kDt);
  }

  const RigidBody& body = world.rigidBody(ball);

  EXPECT_GT(body.velocity.x, 0.f);
  EXPECT_LT(body.angularVelocity.z, 0.f);
  EXPECT_NEAR(body.velocity.x, -body.angularVelocity.z * body.shape().sphereRadius, 0.05f);
}

TEST(AngularTests, FrictionlessSphereNeverStartsRolling) {
  World world(Vec3(0.f, -9.8f, 0.f));
  addGround(world);

  const World::RigidBodyId ball = world.createRigidBody(1.0f, 0.5f);
  world.rigidBody(ball).position = Vec3(0.f, 0.5f, 0.f);
  world.rigidBody(ball).velocity = Vec3(4.f, 0.f, 0.f);
  world.rigidBody(ball).material.restitution = 0.f;
  world.rigidBody(ball).material.friction = 0.f;

  for (int step = 0; step < 240; ++step) {
    world.step(kDt);
  }

  EXPECT_NEAR(world.rigidBody(ball).angularVelocity.lengthSq(), 0.f, 1e-6f);

  EXPECT_GE(world.rigidBody(ball).velocity.x, 4.f);
}
