#include <gtest/gtest.h>

#include "physics/RigidBodySystem.hpp"

#include <vector>

class RigidBodySystemTestAccess {
public:
  static const std::vector<AABBProxy>& aabbProxies(const RigidBodySystem& system) {
    return system.m_aabbProxies;
  }

  static const std::vector<AABBPair>& aabbPairs(const RigidBodySystem& system) {
    return system.m_aabbPairs;
  }

  static const std::vector<RigidBodyContact>& contacts(const RigidBodySystem& system) {
    return system.m_contacts;
  }
};

TEST(RigidBodyContactTests, BuildsAABBProxiesFromRigidBodySpheres) {
  RigidBodySystem system;
  system.reserveRigidBodies(2);

  const RigidBodySystem::RigidBodyId body0 = system.createRigidBody(1.0f, 0.5f);
  const RigidBodySystem::RigidBodyId body1 = system.createRigidBody(1.0f, 1.25f);

  EXPECT_EQ(system.rigidBody(body0).shape().type, ShapeType::Sphere);
  EXPECT_FLOAT_EQ(system.rigidBody(body0).shape().sphereRadius, 0.5f);
  EXPECT_EQ(system.rigidBody(body1).shape().type, ShapeType::Sphere);
  EXPECT_FLOAT_EQ(system.rigidBody(body1).shape().sphereRadius, 1.25f);

  system.rigidBody(body0).position = Vec3(2.0f, -3.0f, 4.0f);
  system.rigidBody(body1).position = Vec3(-1.0f, 0.5f, 3.0f);

  system.step(0.0f, Vec3(0.0f, 0.0f, 0.0f));

  const std::vector<AABBProxy>& proxies = RigidBodySystemTestAccess::aabbProxies(system);

  ASSERT_EQ(proxies.size(), 2u);

  EXPECT_EQ(proxies[0].id, body0);
  EXPECT_FLOAT_EQ(proxies[0].bounds.min.x, 1.5f);
  EXPECT_FLOAT_EQ(proxies[0].bounds.min.y, -3.5f);
  EXPECT_FLOAT_EQ(proxies[0].bounds.min.z, 3.5f);
  EXPECT_FLOAT_EQ(proxies[0].bounds.max.x, 2.5f);
  EXPECT_FLOAT_EQ(proxies[0].bounds.max.y, -2.5f);
  EXPECT_FLOAT_EQ(proxies[0].bounds.max.z, 4.5f);

  EXPECT_EQ(proxies[1].id, body1);
  EXPECT_FLOAT_EQ(proxies[1].bounds.min.x, -2.25f);
  EXPECT_FLOAT_EQ(proxies[1].bounds.min.y, -0.75f);
  EXPECT_FLOAT_EQ(proxies[1].bounds.min.z, 1.75f);
  EXPECT_FLOAT_EQ(proxies[1].bounds.max.x, 0.25f);
  EXPECT_FLOAT_EQ(proxies[1].bounds.max.y, 1.75f);
  EXPECT_FLOAT_EQ(proxies[1].bounds.max.z, 4.25f);
}

TEST(RigidBodyContactTests, AABBTouchingCandidatesCanBeFilteredBySphereNarrowphase) {
  RigidBodySystem system;
  system.reserveRigidBodies(2);

  const RigidBodySystem::RigidBodyId body0 = system.createRigidBody(1.0f, 1.0f);
  const RigidBodySystem::RigidBodyId body1 = system.createRigidBody(1.0f, 1.0f);

  system.rigidBody(body0).position = Vec3(0.0f, 0.0f, 0.0f);
  system.rigidBody(body1).position = Vec3(2.0f, 0.0f, 0.0f);

  system.step(0.0f, Vec3(0.0f, 0.0f, 0.0f));

  const std::vector<AABBPair>& broadphasePairs = RigidBodySystemTestAccess::aabbPairs(system);
  const std::vector<RigidBodyContact>& contacts = RigidBodySystemTestAccess::contacts(system);

  ASSERT_EQ(broadphasePairs.size(), 1u);
  EXPECT_EQ(broadphasePairs[0].a, body0);
  EXPECT_EQ(broadphasePairs[0].b, body1);
  EXPECT_TRUE(contacts.empty());
}

TEST(RigidBodyContactTests, ContactsFollowCollisionPairOrder) {
  RigidBodySystem system;
  system.reserveRigidBodies(3);

  const RigidBodySystem::RigidBodyId body0 = system.createRigidBody(1.0f, 1.0f);
  const RigidBodySystem::RigidBodyId body1 = system.createRigidBody(1.0f, 1.0f);
  const RigidBodySystem::RigidBodyId body2 = system.createRigidBody(1.0f, 1.0f);

  system.rigidBody(body0).position = Vec3(0.0f, 0.0f, 0.0f);
  system.rigidBody(body1).position = Vec3(1.5f, 0.0f, 0.0f);
  system.rigidBody(body2).position = Vec3(0.0f, 1.5f, 0.0f);

  system.rigidBody(body0).material.restitution = 0.25f;
  system.rigidBody(body1).material.restitution = 0.5f;
  system.rigidBody(body2).material.restitution = 0.75f;

  system.step(0.0f, Vec3(0.0f, 0.0f, 0.0f));

  const std::vector<RigidBodyContact>& contacts = RigidBodySystemTestAccess::contacts(system);

  ASSERT_EQ(contacts.size(), 2u);

  EXPECT_EQ(contacts[0].a, body0);
  EXPECT_EQ(contacts[0].b, body1);
  EXPECT_FLOAT_EQ(contacts[0].normal.x, -1.0f);
  EXPECT_FLOAT_EQ(contacts[0].normal.y, 0.0f);
  EXPECT_FLOAT_EQ(contacts[0].penetration, 0.5f);
  EXPECT_FLOAT_EQ(contacts[0].restitution, 0.5f);

  EXPECT_EQ(contacts[1].a, body0);
  EXPECT_EQ(contacts[1].b, body2);
  EXPECT_FLOAT_EQ(contacts[1].normal.x, 0.0f);
  EXPECT_FLOAT_EQ(contacts[1].normal.y, -1.0f);
  EXPECT_FLOAT_EQ(contacts[1].penetration, 0.5f);
  EXPECT_FLOAT_EQ(contacts[1].restitution, 0.75f);
}
