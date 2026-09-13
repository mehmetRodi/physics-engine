#include "math/Mat3.hpp"
#include <gtest/gtest.h>

namespace {

void expectVec3Near(const Vec3& actual, const Vec3& expected, float tolerance = 1e-6f) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

Mat3 counting() {
  Mat3 value{};
  float next = 1.0f;

  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      value.m[row][column] = next;
      next += 1.0f;
    }
  }

  return value;
}

} // namespace

TEST(Mat3Tests, IdentityLeavesAVectorAlone) {
  const Vec3 v(1.5f, -2.0f, 0.25f);

  expectVec3Near(Mat3::identity() * v, v);
}

TEST(Mat3Tests, DiagonalScalesEachAxisIndependently) {
  const Mat3 scale = Mat3::diagonal(Vec3(2.0f, 3.0f, -1.0f));

  expectVec3Near(scale * Vec3(1.0f, 1.0f, 1.0f), Vec3(2.0f, 3.0f, -1.0f));
}

TEST(Mat3Tests, MultiplyingAVectorTakesTheDotProductOfEachRow) {
  expectVec3Near(counting() * Vec3(1.0f, 2.0f, 3.0f), Vec3(14.0f, 32.0f, 50.0f));
}

TEST(Mat3Tests, TransposeSwapsRowsAndColumns) {
  const Mat3 transposed = counting().transposed();

  EXPECT_FLOAT_EQ(transposed.m[0][1], 4.0f);
  EXPECT_FLOAT_EQ(transposed.m[1][0], 2.0f);
  EXPECT_FLOAT_EQ(transposed.m[2][0], 3.0f);
}

TEST(Mat3Tests, MultiplicationMatchesApplyingBothMatricesInTurn) {
  const Mat3 a = counting();
  const Mat3 b = Mat3::diagonal(Vec3(2.0f, -1.0f, 0.5f));
  const Vec3 v(1.0f, 2.0f, 3.0f);

  expectVec3Near((a * b) * v, a * (b * v));
}

TEST(Mat3Tests, MultiplyingByIdentityChangesNothing) {
  const Mat3 a = counting();
  const Mat3 result = a * Mat3::identity();

  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_FLOAT_EQ(result.m[row][column], a.m[row][column]);
    }
  }
}

TEST(Mat3Tests, SimilarityTransformRotatesAnisotropicTensor) {
  Mat3 rotation{};
  rotation.m[0][1] = -1.f;
  rotation.m[1][0] = 1.f;
  rotation.m[2][2] = 1.f;
  const Mat3 result = rotation * Mat3::diagonal(Vec3(1.f, 2.f, 3.f)) * rotation.transposed();
  EXPECT_FLOAT_EQ(result.m[0][0], 2.f);
  EXPECT_FLOAT_EQ(result.m[1][1], 1.f);
  EXPECT_FLOAT_EQ(result.m[2][2], 3.f);
}
