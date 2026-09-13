#include "collision/SpherePair.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(SpherePairTests, SeparatedSpheresProduceNoPairs) {
  const std::vector<SphereProxy> spheres = {
      {0, Vec3(0.f, 0.f, 0.f), 1.f},
      {1, Vec3(3.f, 0.f, 0.f), 1.f},
  };

  const std::vector<CollisionPair> pairs = findSpherePairs(spheres);

  EXPECT_TRUE(pairs.empty());
}

TEST(SpherePairTests, OverlappingSpheresProduceOnePair) {
  const std::vector<SphereProxy> spheres = {
      {0, Vec3(0.f, 0.f, 0.f), 1.f},
      {1, Vec3(1.5f, 0.f, 0.f), 1.f},
  };

  const std::vector<CollisionPair> pairs = findSpherePairs(spheres);

  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].a, 0u);
  EXPECT_EQ(pairs[0].b, 1u);
}

TEST(SpherePairTests, TouchingSpheresProduceNoPairs) {
  const std::vector<SphereProxy> spheres = {
      {0, Vec3(0.f, 0.f, 0.f), 1.f},
      {1, Vec3(2.f, 0.f, 0.f), 1.f},
  };

  const std::vector<CollisionPair> pairs = findSpherePairs(spheres);

  EXPECT_TRUE(pairs.empty());
}

TEST(SpherePairTests, PairOrderIsDeterministic) {
  const std::vector<SphereProxy> spheres = {
      {10, Vec3(0.f, 0.f, 0.f), 1.f},
      {20, Vec3(1.5f, 0.f, 0.f), 1.f},
      {30, Vec3(0.75f, 1.0f, 0.f), 1.f},
  };

  const std::vector<CollisionPair> pairs = findSpherePairs(spheres);

  ASSERT_EQ(pairs.size(), 3u);

  EXPECT_EQ(pairs[0].a, 10u);
  EXPECT_EQ(pairs[0].b, 20u);

  EXPECT_EQ(pairs[1].a, 10u);
  EXPECT_EQ(pairs[1].b, 30u);

  EXPECT_EQ(pairs[2].a, 20u);
  EXPECT_EQ(pairs[2].b, 30u);
}
