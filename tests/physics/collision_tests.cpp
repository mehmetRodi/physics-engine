#include "physics/World.hpp"
#include <gtest/gtest.h>

TEST(CollisionTests, SeparatedBodiesDoNotChangeVelocity) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(3.f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, -1.f);
}

TEST(CollisionTests, OverlappingBodiesMovingTogetherDoNotReceiveImpulse) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, 1.f);
}

TEST(CollisionTests, HeadOnEqualMassCollisionSwapsVelocities) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, -1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, 1.f);
}

TEST(CollisionTests, DynamicBodyBouncesOffStaticBody) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId dynamicBody = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(dynamicBody).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(dynamicBody).velocity = Vec3(1.f, 0.f, 0.f);

  const World::RigidBodyId staticBody = world.createRigidBody(0.0f, 1.0f);
  world.rigidBody(staticBody).position = Vec3(0.5f, 0.f, 0.f);
  world.rigidBody(staticBody).velocity = Vec3(0.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(dynamicBody).velocity.x, -1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(staticBody).velocity.x, 0.f);
}

TEST(CollisionTests, ExactlyTouchingBodiesDoNotCollide) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(2.f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, -1.f);
}

TEST(CollisionTests, IdenticalPositionsDoNotApplyAmbiguousImpulse) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(1.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, 1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.z, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, -1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.z, 0.f);
}

TEST(CollisionTests, OverlappingStaticBodiesDoNotChangeVelocity) {
  World world(Vec3(0.f, 0.f, 0.f));
  const World::RigidBodyId body1 = world.createRigidBody(0.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(0.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(0.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(0.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.z, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.y, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.z, 0.f);
}

TEST(CollisionTests, OverlappingDynamicBodiesAreSeparatedGradually) {
  World world(Vec3(0.f, 0.f, 0.f));

  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(1.5f, 0.f, 0.f);

  world.step(0.0f);

  const float x1 = world.rigidBody(body1).position.x;
  const float x2 = world.rigidBody(body2).position.x;

  EXPECT_FLOAT_EQ(-x1, x2 - 1.5f);
  EXPECT_GT(x2 - x1, 1.5f);
  EXPECT_LT(x2 - x1, 2.f);
}

TEST(CollisionTests, RepeatedStepsSeparateOverlappingBodiesDownToTheSlop) {
  World world(Vec3(0.f, 0.f, 0.f));

  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(1.5f, 0.f, 0.f);

  for (int step = 0; step < 200; ++step) {
    world.step(0.0f);
  }

  const float gap = world.rigidBody(body2).position.x - world.rigidBody(body1).position.x;
  EXPECT_NEAR(gap, 2.f - world.contactSolverSettings().penetrationSlop, 1e-4f);
}

TEST(CollisionTests, ZeroRestitutionEqualMassCollisionStopsBothBodies) {
  World world(Vec3(0.f, 0.f, 0.f));

  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(1.f, 0.f, 0.f);
  world.rigidBody(body1).material.restitution = 0.f;

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(-1.f, 0.f, 0.f);
  world.rigidBody(body2).material.restitution = 0.f;

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, 0.f);
}

TEST(CollisionTests, RestitutionUsesBouncierMaterial) {
  World world(Vec3(0.f, 0.f, 0.f));

  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(body1).velocity = Vec3(1.f, 0.f, 0.f);
  world.rigidBody(body1).material.restitution = 0.f;

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(0.5f, 0.f, 0.f);
  world.rigidBody(body2).velocity = Vec3(-1.f, 0.f, 0.f);
  world.rigidBody(body2).material.restitution = 1.f;

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).velocity.x, -1.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).velocity.x, 1.f);
}

TEST(CollisionTests, HeadOnDifferentMassCollisionUsesInverseMass) {
  World world(Vec3(0.f, 0.f, 0.f));

  const World::RigidBodyId lightBody = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(lightBody).position = Vec3(0.f, 0.f, 0.f);
  world.rigidBody(lightBody).velocity = Vec3(1.f, 0.f, 0.f);

  const World::RigidBodyId heavyBody = world.createRigidBody(3.0f, 1.0f);
  world.rigidBody(heavyBody).position = Vec3(0.5f, 0.f, 0.f);
  world.rigidBody(heavyBody).velocity = Vec3(-1.f, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(lightBody).velocity.x, -2.f);
  EXPECT_FLOAT_EQ(world.rigidBody(heavyBody).velocity.x, 0.f);
}

TEST(CollisionTests, OnlyTheDynamicBodyMovesWhenSeparatingFromAStaticBody) {
  World world(Vec3(0.f, 0.f, 0.f));

  const World::RigidBodyId dynamicBody = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(dynamicBody).position = Vec3(0.f, 0.f, 0.f);

  const World::RigidBodyId staticBody = world.createRigidBody(0.0f, 1.0f);
  world.rigidBody(staticBody).position = Vec3(1.5f, 0.f, 0.f);

  for (int step = 0; step < 200; ++step) {
    world.step(0.0f);
  }

  EXPECT_FLOAT_EQ(world.rigidBody(staticBody).position.x, 1.5f);
  EXPECT_NEAR(world.rigidBody(dynamicBody).position.x,
              -0.5f + world.contactSolverSettings().penetrationSlop, 1e-4f);
}

TEST(CollisionTests, ShallowOverlapInsideTheSlopIsLeftAlone) {
  World world(Vec3(0.f, 0.f, 0.f));

  const float slop = ContactSolverSettings{}.penetrationSlop;

  const World::RigidBodyId body1 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body1).position = Vec3(0.f, 0.f, 0.f);

  const World::RigidBodyId body2 = world.createRigidBody(1.0f, 1.0f);
  world.rigidBody(body2).position = Vec3(2.f - 0.5f * slop, 0.f, 0.f);

  world.step(0.0f);

  EXPECT_FLOAT_EQ(world.rigidBody(body1).position.x, 0.f);
  EXPECT_FLOAT_EQ(world.rigidBody(body2).position.x, 2.f - 0.5f * slop);
}
