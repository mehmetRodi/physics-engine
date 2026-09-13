#include <gtest/gtest.h>

#include "physics/ContactSolver.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr float kRadius = 1.0f;
constexpr float kPenetration = 0.1f;

std::vector<RigidBody> makeVerticalStack(std::size_t dynamicCount, float fallSpeed) {
  std::vector<RigidBody> bodies;
  bodies.reserve(dynamicCount + 1);

  bodies.emplace_back(0.0f, kRadius);
  bodies.back().position = Vec3(0.f, 0.f, 0.f);
  bodies.back().material.restitution = 0.f;

  for (std::size_t i = 0; i < dynamicCount; ++i) {
    const float height = static_cast<float>(i + 1) * (2.f * kRadius - kPenetration);
    bodies.emplace_back(1.0f, kRadius);
    bodies.back().position = Vec3(0.f, height, 0.f);
    bodies.back().velocity = Vec3(0.f, -fallSpeed, 0.f);
    bodies.back().material.restitution = 0.f;
  }

  return bodies;
}

std::vector<RigidBodyContact> makeStackContacts(std::size_t dynamicCount, float friction = 0.f) {
  std::vector<RigidBodyContact> contacts;
  contacts.reserve(dynamicCount);

  for (std::size_t i = 0; i < dynamicCount; ++i) {

    const float centreHeight = (static_cast<float>(i) + 0.5f) * (2.f * kRadius - kPenetration);
    contacts.push_back(
        {i + 1, i, Vec3(0.f, 1.f, 0.f), Vec3(0.f, centreHeight, 0.f), kPenetration, 0.f, friction});
  }

  return contacts;
}

float closingSpeed(const std::vector<RigidBody>& bodies, const RigidBodyContact& contact) {
  return (bodies[contact.a].velocity - bodies[contact.b].velocity).dot(contact.normal);
}

} // namespace

TEST(ContactSolverTests, SingleIterationLeavesTheTopOfAStackStillClosing) {
  std::vector<RigidBody> bodies = makeVerticalStack(2, 2.0f);
  const std::vector<RigidBodyContact> contacts = makeStackContacts(2);

  ContactSolver solver;
  ContactSolverSettings settings;
  settings.velocityIterations = 1;
  solver.setSettings(settings);

  solver.solve(bodies, contacts);

  EXPECT_LT(closingSpeed(bodies, contacts[0]), -0.5f);
}

namespace {

float worstClosingSpeed(std::size_t dynamicCount, float fallSpeed, int velocityIterations) {
  std::vector<RigidBody> bodies = makeVerticalStack(dynamicCount, fallSpeed);
  const std::vector<RigidBodyContact> contacts = makeStackContacts(dynamicCount);

  ContactSolver solver;
  ContactSolverSettings settings;
  settings.velocityIterations = velocityIterations;
  solver.setSettings(settings);

  solver.solve(bodies, contacts);

  float worst = 0.f;
  for (const RigidBodyContact& contact : contacts) {
    worst = std::min(worst, closingSpeed(bodies, contact));
  }

  return worst;
}

} // namespace

TEST(ContactSolverTests, MoreIterationsLeaveLessClosingSpeedInAStack) {

  const float twoPasses = worstClosingSpeed(3, 2.0f, 2);
  const float eightPasses = worstClosingSpeed(3, 2.0f, 8);
  const float thirtyTwoPasses = worstClosingSpeed(3, 2.0f, 32);

  EXPECT_GT(eightPasses, twoPasses);
  EXPECT_GT(thirtyTwoPasses, eightPasses);
  EXPECT_LE(thirtyTwoPasses, 0.f);
}

TEST(ContactSolverTests, DefaultIterationCountRemovesMostOfTheClosingSpeed) {

  EXPECT_GT(worstClosingSpeed(3, 2.0f, 8), -0.2f);
}

TEST(ContactSolverTests, AccumulatedImpulseIsNeverNegative) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  bodies[1].velocity = Vec3(0.f, 3.0f, 0.f);

  const std::vector<RigidBodyContact> contacts = makeStackContacts(1);

  ContactSolver solver;
  solver.solve(bodies, contacts);

  EXPECT_FLOAT_EQ(bodies[1].velocity.y, 3.0f);
}

TEST(ContactSolverTests, ContactClosingBelowTheThresholdDoesNotBounce) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 0.2f);
  bodies[0].material.restitution = 1.f;
  bodies[1].material.restitution = 1.f;

  std::vector<RigidBodyContact> contacts = makeStackContacts(1);
  contacts[0].restitution = 1.f;

  ContactSolver solver;
  ASSERT_GT(solver.settings().restitutionThreshold, 0.2f);

  solver.solve(bodies, contacts);

  EXPECT_NEAR(bodies[1].velocity.y, 0.f, 1e-5f);
}

TEST(ContactSolverTests, ContactClosingAboveTheThresholdStillBounces) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 4.0f);
  std::vector<RigidBodyContact> contacts = makeStackContacts(1);
  contacts[0].restitution = 1.f;

  ContactSolver solver;
  ASSERT_LT(solver.settings().restitutionThreshold, 4.0f);

  solver.solve(bodies, contacts);

  EXPECT_NEAR(bodies[1].velocity.y, 4.0f, 1e-5f);
}

TEST(ContactSolverTests, ContactBetweenTwoStaticBodiesIsSkipped) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  bodies[1].setMass(0.f);

  const std::vector<RigidBodyContact> contacts = makeStackContacts(1);
  const Vec3 positionBefore = bodies[1].position;

  ContactSolver solver;
  solver.solve(bodies, contacts);

  EXPECT_FLOAT_EQ(bodies[1].velocity.y, 0.0f);
  EXPECT_FLOAT_EQ(bodies[1].position.y, positionBefore.y);
}

TEST(ContactSolverTests, FrictionlessContactLeavesTheSlidingSpeedAlone) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  bodies[1].velocity = Vec3(3.f, -2.f, 0.f);

  const std::vector<RigidBodyContact> contacts = makeStackContacts(1, 0.f);

  ContactSolver solver;
  solver.solve(bodies, contacts);

  EXPECT_FLOAT_EQ(bodies[1].velocity.x, 3.f);
}

TEST(ContactSolverTests, FrictionSpinsASlidingSphereUpIntoRolling) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  bodies[1].velocity = Vec3(0.5f, -2.f, 0.f);

  std::vector<RigidBodyContact> contacts = makeStackContacts(1, 0.5f);

  contacts[0].point = bodies[1].position - Vec3(0.f, kRadius, 0.f);

  const Vec3 offset = contacts[0].point - bodies[1].position;

  ContactSolver solver;
  solver.solve(bodies, contacts);

  EXPECT_NEAR(bodies[1].velocity.x, 0.5f * 5.f / 7.f, 1e-4f);

  const Vec3 contactVelocity = bodies[1].velocity + bodies[1].angularVelocity.cross(offset);

  EXPECT_NEAR(contactVelocity.x, 0.f, 1e-4f);
  EXPECT_NEAR(contactVelocity.z, 0.f, 1e-4f);
}

TEST(ContactSolverTests, FrictionOnACentredNormalDoesNotChangeTheNormalResponse) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  bodies[1].velocity = Vec3(0.5f, -2.f, 0.f);

  const std::vector<RigidBodyContact> contacts = makeStackContacts(1, 0.5f);

  ContactSolver solver;
  solver.solve(bodies, contacts);

  EXPECT_NEAR(bodies[1].velocity.y, 0.f, 1e-5f);
}

TEST(ContactSolverTests, FrictionImpulseIsCappedByTheNormalImpulse) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  bodies[1].velocity = Vec3(10.f, -2.f, 0.f);

  const std::vector<RigidBodyContact> contacts = makeStackContacts(1, 0.5f);

  ContactSolver solver;
  solver.solve(bodies, contacts);

  EXPECT_NEAR(bodies[1].velocity.x, 9.f, 1e-5f);
}

TEST(ContactSolverTests, FrictionActsOnBothAxesOfTheContactPlane) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  bodies[1].velocity = Vec3(0.3f, -2.f, 0.4f);

  std::vector<RigidBodyContact> contacts = makeStackContacts(1, 0.5f);
  contacts[0].point = bodies[1].position - Vec3(0.f, kRadius, 0.f);

  ContactSolver solver;
  solver.solve(bodies, contacts);

  EXPECT_NEAR(bodies[1].velocity.x, 0.3f * 5.f / 7.f, 1e-4f);
  EXPECT_NEAR(bodies[1].velocity.z, 0.4f * 5.f / 7.f, 1e-4f);
}

namespace {

float restingResidual(bool warmStarting) {
  std::vector<RigidBody> bodies = makeVerticalStack(3, 0.f);
  const std::vector<RigidBodyContact> contacts = makeStackContacts(3);

  ContactSolverSettings settings;
  settings.velocityIterations = 2;
  settings.warmStarting = warmStarting;
  settings.positionCorrectionRate = 0.f;

  ContactSolver solver;
  solver.setSettings(settings);

  constexpr float dt = 1.f / 60.f;
  float worst = 0.f;

  for (int tick = 0; tick < 30; ++tick) {
    for (std::size_t i = 1; i < bodies.size(); ++i) {
      bodies[i].velocity.y -= 9.8f * dt;
    }

    solver.solve(bodies, contacts);

    worst = 0.f;
    for (const RigidBodyContact& contact : contacts) {
      worst = std::min(worst, closingSpeed(bodies, contact));
    }
  }

  return worst;
}

} // namespace

TEST(ContactSolverTests, WarmStartingHoldsARestingStackBetterAtLowIterationCounts) {
  EXPECT_GT(restingResidual(true), restingResidual(false));
}

TEST(ContactSolverTests, WarmStartingReappliesTheImpulseFromThePreviousTick) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  const std::vector<RigidBodyContact> contacts = makeStackContacts(1);

  ContactSolver solver;
  solver.solve(bodies, contacts);

  ContactSolverSettings settings;
  settings.velocityIterations = 0;
  solver.setSettings(settings);

  bodies[1].velocity = Vec3(0.f, 0.f, 0.f);
  solver.solve(bodies, contacts);

  EXPECT_FLOAT_EQ(bodies[1].velocity.y, 2.0f);
}

TEST(ContactSolverTests, ImpulsesAreDroppedOnceAContactStopsBeingReported) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  const std::vector<RigidBodyContact> contacts = makeStackContacts(1);

  ContactSolver solver;
  solver.solve(bodies, contacts);
  solver.solve(bodies, {});

  ContactSolverSettings settings;
  settings.velocityIterations = 0;
  solver.setSettings(settings);

  bodies[1].velocity = Vec3(0.f, 0.f, 0.f);
  solver.solve(bodies, contacts);

  EXPECT_FLOAT_EQ(bodies[1].velocity.y, 0.f);
}

TEST(ContactSolverTests, WarmStartingCanBeTurnedOff) {
  std::vector<RigidBody> bodies = makeVerticalStack(1, 2.0f);
  const std::vector<RigidBodyContact> contacts = makeStackContacts(1);

  ContactSolverSettings settings;
  settings.warmStarting = false;

  ContactSolver solver;
  solver.setSettings(settings);
  solver.solve(bodies, contacts);

  settings.velocityIterations = 0;
  solver.setSettings(settings);

  bodies[1].velocity = Vec3(0.f, 0.f, 0.f);
  solver.solve(bodies, contacts);

  EXPECT_FLOAT_EQ(bodies[1].velocity.y, 0.f);
}

TEST(ContactSolverTests, WarmStartReprojectsTangentAcrossBasisSwitch) {
  std::vector<RigidBody> bodies;
  bodies.emplace_back(0.f, 1.f);
  bodies.emplace_back(1.f, 1.f);
  const Vec3 n1 = Vec3(.5773f, .816532f, 0.f).normalized();
  const Vec3 n2 = Vec3(.5774f, .816461f, 0.f).normalized();
  bodies[1].velocity = n1 * -1.f + Vec3(0, 0, 2);
  std::vector<RigidBodyContact> contacts{{1, 0, n1, Vec3(), .01f, 0.f, .5f}};
  ContactSolver solver;
  ContactSolverSettings settings;
  settings.positionCorrectionRate = 0.f;
  solver.setSettings(settings);
  solver.solve(bodies, contacts);
  const float tangentImpulse = bodies[1].velocity.z - 2.f;
  ASSERT_LT(tangentImpulse, -.4f);
  bodies[1].velocity = Vec3();
  bodies[1].angularVelocity = Vec3();
  contacts[0].normal = n2;
  settings.velocityIterations = 0;
  solver.setSettings(settings);
  solver.solve(bodies, contacts);
  EXPECT_NEAR(bodies[1].velocity.z, tangentImpulse, 1e-5f);
  EXPECT_NEAR(bodies[1].velocity.dot(n2), 1.f, 1e-4f);
  bodies[1].velocity = Vec3();
  contacts[0].normal = Vec3(1, 0, 0);
  solver.solve(bodies, contacts);
  EXPECT_NEAR(bodies[1].velocity.lengthSq(), 0.f, 1e-8f);
}

TEST(ContactSolverTests, DiagonalFrictionStaysInsideCoulombDisk) {
  std::vector<RigidBody> bodies;
  bodies.emplace_back(0.f, 1.f);
  bodies.emplace_back(1.f, 1.f);
  bodies[1].velocity = Vec3(10, -2, 10);
  const std::vector<RigidBodyContact> contacts{{1, 0, Vec3(0, 1, 0), Vec3(), .01f, 0.f, .5f}};
  ContactSolver solver;
  solver.solve(bodies, contacts);
  const float tx = bodies[1].velocity.x - 10.f, tz = bodies[1].velocity.z - 10.f;
  EXPECT_NEAR(std::hypot(tx, tz), 1.f, 1e-5f);
  EXPECT_LT(bodies[1].velocity.lengthSq(), 204.f);
}
