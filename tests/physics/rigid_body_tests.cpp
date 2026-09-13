#include "physics/RigidBody.hpp"
#include <gtest/gtest.h>

TEST(RigidBodyTests, PositiveMassStoresInverseMass) {
  const RigidBody body(2.0f, 20.0f);
  EXPECT_FLOAT_EQ(body.invMass(), 0.5f);
}

TEST(RigidBodyTests, ZeroMassBodyHasZeroInverseMass) {
  const RigidBody body(0.0f, 20.0f);
  EXPECT_FLOAT_EQ(body.invMass(), 0.0f);
}

TEST(RigidBodyTests, ApplyForceChangesAccelerationByForceTimesInvMass) {
  RigidBody body(2.0f, 20.0f);
  body.applyForce(Vec3(4.0f, 0.f, 0.f));
  EXPECT_FLOAT_EQ(body.acceleration.x, 2.0f);
  EXPECT_FLOAT_EQ(body.acceleration.y, 0.0f);
  EXPECT_FLOAT_EQ(body.acceleration.z, 0.0f);
}

TEST(RigidBodyTests, UpdateUsesSemiImplicitEuler) {
  RigidBody body(2.0f, 20.0f);
  body.acceleration = Vec3(4.0f, 0.f, 0.f);
  body.velocity = Vec3(1.0f, 0.f, 0.f);
  body.update(1.0f);
  EXPECT_FLOAT_EQ(body.velocity.x, 5.0f);
  EXPECT_FLOAT_EQ(body.position.x, 5.0f);
}

TEST(RigidBodyTests, UpdateClearsAccelerationAfterStep) {
  RigidBody body(2.0f, 20.0f);
  body.acceleration = Vec3(4.0f, 0.f, 0.f);
  body.update(1.0f);
  EXPECT_FLOAT_EQ(body.acceleration.x, 0.0f);
  EXPECT_FLOAT_EQ(body.acceleration.y, 0.0f);
  EXPECT_FLOAT_EQ(body.acceleration.z, 0.0f);
}

TEST(RigidBodyTests, LinearDampingReducesVelocityDuringUpdate) {
  RigidBody body(1.0f, 20.0f);
  body.velocity = Vec3(10.0f, 0.0f, 0.0f);
  body.material.linearDamping = 0.25f;

  body.update(1.0f);

  EXPECT_FLOAT_EQ(body.velocity.x, 7.5f);
  EXPECT_FLOAT_EQ(body.position.x, 7.5f);
}

TEST(RigidBodyTests, LinearDampingDoesNotReverseVelocity) {
  RigidBody body(1.0f, 20.0f);
  body.velocity = Vec3(10.0f, 0.0f, 0.0f);
  body.material.linearDamping = 2.0f;

  body.update(1.0f);

  EXPECT_FLOAT_EQ(body.velocity.x, 0.0f);
  EXPECT_FLOAT_EQ(body.position.x, 0.0f);
}
