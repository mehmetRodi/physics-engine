#include "math/Quat.hpp"
#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;

void expectVec3Near(const Vec3& actual, const Vec3& expected, float tolerance = 1e-5f) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

} // namespace

TEST(QuatTests, DefaultConstructedQuaternionIsTheIdentityRotation) {
  const Vec3 v(1.0f, 2.0f, 3.0f);

  expectVec3Near(Quat().rotate(v), v);
}

TEST(QuatTests, QuarterTurnAboutZMapsXOntoY) {
  const Quat rotation = Quat::fromAxisAngle(Vec3(0.f, 0.f, 1.f), 0.5f * kPi);

  expectVec3Near(rotation.rotate(Vec3(1.f, 0.f, 0.f)), Vec3(0.f, 1.f, 0.f));
}

TEST(QuatTests, RotationAboutAnAxisLeavesThatAxisFixed) {
  const Vec3 axis = Vec3(1.f, 2.f, -0.5f).normalized();
  const Quat rotation = Quat::fromAxisAngle(axis, 1.1f);

  expectVec3Near(rotation.rotate(axis), axis);
}

TEST(QuatTests, ProductAppliesTheRightHandRotationFirst) {
  const Quat aboutZ = Quat::fromAxisAngle(Vec3(0.f, 0.f, 1.f), 0.5f * kPi);
  const Quat aboutX = Quat::fromAxisAngle(Vec3(1.f, 0.f, 0.f), 0.5f * kPi);
  const Vec3 v(1.f, 0.f, 0.f);

  expectVec3Near((aboutX * aboutZ).rotate(v), aboutX.rotate(aboutZ.rotate(v)));
}

TEST(QuatTests, MatrixFormAgreesWithDirectRotation) {
  const Quat rotation = Quat::fromAxisAngle(Vec3(0.3f, -1.f, 0.7f), 2.0f);
  const Vec3 v(0.5f, -1.5f, 2.0f);

  expectVec3Near(rotation.toMat3() * v, rotation.rotate(v));
}

TEST(QuatTests, RotationMatrixIsOrthonormal) {
  const Quat rotation = Quat::fromAxisAngle(Vec3(0.3f, -1.f, 0.7f), 2.0f);
  const Mat3 product = rotation.toMat3() * rotation.toMat3().transposed();
  const Mat3 identity = Mat3::identity();

  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_NEAR(product.m[row][column], identity.m[row][column], 1e-5f);
    }
  }
}

TEST(QuatTests, IntegrationTurnsAtTheRequestedRate) {
  const Vec3 angularVelocity(0.f, 0.f, 1.0f);
  Quat orientation;

  constexpr int steps = 20000;
  constexpr float dt = (0.5f * kPi) / static_cast<float>(steps);

  for (int i = 0; i < steps; ++i) {
    orientation = orientation.integrated(angularVelocity, dt);
  }

  expectVec3Near(orientation.rotate(Vec3(1.f, 0.f, 0.f)), Vec3(0.f, 1.f, 0.f), 1e-3f);
}

TEST(QuatTests, IntegrationKeepsTheQuaternionNormalised) {
  Quat orientation;

  for (int i = 0; i < 1000; ++i) {
    orientation = orientation.integrated(Vec3(1.3f, -0.7f, 2.1f), 1.f / 60.f);
  }

  EXPECT_NEAR(orientation.lengthSq(), 1.0f, 1e-5f);
}

TEST(QuatTests, ZeroAngularVelocityLeavesOrientationUnchanged) {
  const Quat orientation = Quat::fromAxisAngle(Vec3(0.f, 1.f, 0.f), 0.8f);
  const Quat stepped = orientation.integrated(Vec3(0.f, 0.f, 0.f), 1.f / 60.f);

  EXPECT_FLOAT_EQ(stepped.w, orientation.w);
  EXPECT_FLOAT_EQ(stepped.x, orientation.x);
  EXPECT_FLOAT_EQ(stepped.y, orientation.y);
  EXPECT_FLOAT_EQ(stepped.z, orientation.z);
}
