#include "math/Vec3.hpp"
#include <gtest/gtest.h>

namespace {

void expectVec3Near(const Vec3& actual, const Vec3& expected, float tolerance = 1e-6f) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

} // namespace

TEST(Vec3Tests, DefaultConstructorStartsAtOrigin) {
  const Vec3 value;

  expectVec3Near(value, Vec3(0.0f, 0.0f, 0.0f));
}

TEST(Vec3Tests, AdditionIsComponentWise) {
  const Vec3 a(1.5f, -2.0f, 0.5f);
  const Vec3 b(-0.5f, 3.0f, -1.0f);
  const Vec3 result = a + b;

  expectVec3Near(result, Vec3(1.0f, 1.0f, -0.5f));
}

TEST(Vec3Tests, SubtractionIsComponentWise) {
  const Vec3 a(1.5f, -2.0f, 0.5f);
  const Vec3 b(-0.5f, 3.0f, -1.0f);
  const Vec3 result = a - b;

  expectVec3Near(result, Vec3(2.0f, -5.0f, 1.5f));
}

TEST(Vec3Tests, ScalarMultiplicationScalesEveryComponent) {
  const Vec3 a(1.5f, -2.0f, 0.5f);
  const float scalar = 2.0f;
  const Vec3 result = a * scalar;

  expectVec3Near(result, Vec3(3.0f, -4.0f, 1.0f));
}

TEST(Vec3Tests, ScalarDivisionScalesEveryComponentByReciprocal) {
  const Vec3 a(3.0f, -4.0f, 1.0f);
  const float scalar = 2.0f;
  const Vec3 result = a / scalar;

  expectVec3Near(result, Vec3(1.5f, -2.0f, 0.5f));
}

TEST(Vec3Tests, DotProductReturnsExpectedScalar) {
  const Vec3 a(1.0f, 2.0f, 3.0f);
  const Vec3 b(4.0f, -5.0f, 6.0f);
  const float result = a.dot(b);

  EXPECT_FLOAT_EQ(result, 12.0f);
}

TEST(Vec3Tests, CrossProductReturnsPerpendicularVector) {
  const Vec3 xAxis(1.0f, 0.0f, 0.0f);
  const Vec3 yAxis(0.0f, 1.0f, 0.0f);
  const Vec3 zAxis(0.0f, 0.0f, 1.0f);

  const Vec3 result = xAxis.cross(yAxis);

  expectVec3Near(result, zAxis);
}

TEST(Vec3Tests, LengthSqReturnsSquaredLength) {
  const Vec3 value(3.0f, 4.0f, 12.0f);
  const float result = value.lengthSq();

  EXPECT_FLOAT_EQ(result, 169.0f);
}

TEST(Vec3Tests, LengthReturnsEuclideanLength) {
  const Vec3 value(3.0f, 4.0f, 0.0f);
  const float result = value.length();

  EXPECT_NEAR(result, 5.0f, 1e-6f);
}

TEST(Vec3Tests, NormalizedReturnsUnitLengthVector) {
  const Vec3 value(10.0f, 0.0f, 0.0f);
  const Vec3 result = value.normalized();

  expectVec3Near(result, Vec3(1.0f, 0.0f, 0.0f));
  EXPECT_NEAR(result.length(), 1.0f, 1e-6f);
}

TEST(Vec3Tests, NormalizedReturnsZeroForZeroVector) {
  const Vec3 value(0.0f, 0.0f, 0.0f);
  const Vec3 result = value.normalized();

  expectVec3Near(result, Vec3(0.0f, 0.0f, 0.0f));
}
