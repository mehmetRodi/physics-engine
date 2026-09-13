#include "collision/AABBPair.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace {
void expectPairsEqual(const std::vector<AABBPair>& actual, const std::vector<AABBPair>& expected) {
  ASSERT_EQ(actual.size(), expected.size());

  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(actual[i].a, expected[i].a);
    EXPECT_EQ(actual[i].b, expected[i].b);
  }
}
} // namespace

TEST(AABBPairTests, SeparatedProxiesProduceNoPairs) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(2.f, 2.f, 2.f), Vec3(3.f, 3.f, 3.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  EXPECT_TRUE(pairs.empty());
}

TEST(AABBPairTests, SweepAndPruneMatchesBaselineForSeparatedProxies) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(2.f, 2.f, 2.f), Vec3(3.f, 3.f, 3.f)}},
  };

  expectPairsEqual(findAABBPairsSweepAndPrune(proxies), findAABBPairs(proxies));
}

TEST(AABBPairTests, OverlappingProxiesProduceOnePair) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {1, {Vec3(1.f, 1.f, 1.f), Vec3(3.f, 3.f, 3.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].a, 0u);
  EXPECT_EQ(pairs[0].b, 1u);
}

TEST(AABBPairTests, SweepAndPruneMatchesBaselineForOverlappingProxies) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {1, {Vec3(1.f, 1.f, 1.f), Vec3(3.f, 3.f, 3.f)}},
  };

  expectPairsEqual(findAABBPairsSweepAndPrune(proxies), findAABBPairs(proxies));
}

TEST(AABBPairTests, TouchingProxiesProduceOnePair) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(1.f, 0.f, 0.f), Vec3(2.f, 1.f, 1.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].a, 0u);
  EXPECT_EQ(pairs[0].b, 1u);
}

TEST(AABBPairTests, SweepAndPruneMatchesBaselineForTouchingProxies) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(1.f, 0.f, 0.f), Vec3(2.f, 1.f, 1.f)}},
  };

  expectPairsEqual(findAABBPairsSweepAndPrune(proxies), findAABBPairs(proxies));
}

TEST(AABBPairTests, PairOrderIsDeterministic) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {20, {Vec3(1.f, 0.f, 0.f), Vec3(3.f, 2.f, 2.f)}},
      {30, {Vec3(0.f, 1.f, 0.f), Vec3(2.f, 3.f, 2.f)}},
  };

  const std::vector<AABBPair> pairs = findAABBPairs(proxies);

  ASSERT_EQ(pairs.size(), 3u);

  EXPECT_EQ(pairs[0].a, 10u);
  EXPECT_EQ(pairs[0].b, 20u);

  EXPECT_EQ(pairs[1].a, 10u);
  EXPECT_EQ(pairs[1].b, 30u);

  EXPECT_EQ(pairs[2].a, 20u);
  EXPECT_EQ(pairs[2].b, 30u);
}

TEST(AABBPairTests, SweepAndPruneMatchesBaselineOrderForMixedProxies) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(4.f, 0.f, 0.f), Vec3(6.f, 2.f, 2.f)}},
      {20, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {30, {Vec3(1.f, 1.f, 0.f), Vec3(3.f, 3.f, 2.f)}},
      {40, {Vec3(5.f, 1.f, 0.f), Vec3(7.f, 3.f, 2.f)}},
      {50, {Vec3(8.f, 8.f, 0.f), Vec3(9.f, 9.f, 1.f)}},
  };

  expectPairsEqual(findAABBPairsSweepAndPrune(proxies), findAABBPairs(proxies));
}

TEST(AABBPairTests, BVHMatchesBaselineOrderForMixedProxies) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(4.f, 0.f, 0.f), Vec3(6.f, 2.f, 2.f)}},
      {20, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {30, {Vec3(1.f, 1.f, 0.f), Vec3(3.f, 3.f, 2.f)}},
      {40, {Vec3(5.f, 1.f, 0.f), Vec3(7.f, 3.f, 2.f)}},
      {50, {Vec3(8.f, 8.f, 0.f), Vec3(9.f, 9.f, 1.f)}},
  };

  expectPairsEqual(findAABBPairsBVH(proxies), findAABBPairs(proxies));
}

TEST(AABBPairTests, SweepAndPruneScratchOverloadMatchesBaselineForMixedProxies) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(4.f, 0.f, 0.f), Vec3(6.f, 2.f, 2.f)}},
      {20, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {30, {Vec3(1.f, 1.f, 0.f), Vec3(3.f, 3.f, 2.f)}},
      {40, {Vec3(5.f, 1.f, 0.f), Vec3(7.f, 3.f, 2.f)}},
      {50, {Vec3(8.f, 8.f, 0.f), Vec3(9.f, 9.f, 1.f)}},
  };

  AABBSweepAndPruneScratch scratch;
  scratch.reserve(proxies.size());

  std::vector<AABBPair> pairs;
  findAABBPairsSweepAndPrune(proxies, scratch, pairs);

  expectPairsEqual(pairs, findAABBPairs(proxies));
}

TEST(AABBPairTests, BVHScratchOverloadMatchesBaselineForMixedProxies) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(4.f, 0.f, 0.f), Vec3(6.f, 2.f, 2.f)}},
      {20, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {30, {Vec3(1.f, 1.f, 0.f), Vec3(3.f, 3.f, 2.f)}},
      {40, {Vec3(5.f, 1.f, 0.f), Vec3(7.f, 3.f, 2.f)}},
      {50, {Vec3(8.f, 8.f, 0.f), Vec3(9.f, 9.f, 1.f)}},
  };

  AABBBVHScratch scratch;
  scratch.reserve(proxies.size());

  std::vector<AABBPair> pairs;
  findAABBPairsBVH(proxies, scratch, pairs);

  expectPairsEqual(pairs, findAABBPairs(proxies));
}

TEST(AABBPairTests, BVHStagedAPIProducesSamePairsAsConvenienceAPI) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(4.f, 0.f, 0.f), Vec3(6.f, 2.f, 2.f)}},
      {20, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {30, {Vec3(1.f, 1.f, 0.f), Vec3(3.f, 3.f, 2.f)}},
      {40, {Vec3(5.f, 1.f, 0.f), Vec3(7.f, 3.f, 2.f)}},
      {50, {Vec3(8.f, 8.f, 0.f), Vec3(9.f, 9.f, 1.f)}},
  };

  AABBBVHScratch scratch;
  scratch.reserve(proxies.size());

  const AABBBVHBuildResult buildResult = buildAABBBVH(proxies, scratch);
  ASSERT_FALSE(buildResult.empty);

  std::vector<AABBPair> proxyIndexPairs;
  collectAABBBVHPairCandidates(scratch, buildResult.rootNode, proxyIndexPairs);

  std::vector<AABBPair> pairs;
  emitSortedAABBPairsFromProxyIndexPairs(proxies, proxyIndexPairs, pairs);

  expectPairsEqual(pairs, findAABBPairsBVH(proxies));
  expectPairsEqual(pairs, findAABBPairs(proxies));
}

TEST(AABBPairTests, OutputVectorOverloadClearsPreviousContents) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(3.f, 3.f, 3.f), Vec3(4.f, 4.f, 4.f)}},
  };

  std::vector<AABBPair> pairs = {{100, 200}};
  findAABBPairs(proxies, pairs);

  EXPECT_TRUE(pairs.empty());
}

TEST(AABBPairTests, SweepAndPruneOutputVectorOverloadClearsPreviousContents) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(3.f, 3.f, 3.f), Vec3(4.f, 4.f, 4.f)}},
  };

  std::vector<AABBPair> pairs = {{100, 200}};
  findAABBPairsSweepAndPrune(proxies, pairs);

  EXPECT_TRUE(pairs.empty());
}

TEST(AABBPairTests, BVHEmptyInputProducesNoPairs) {
  const std::vector<AABBProxy> proxies;

  EXPECT_TRUE(findAABBPairsBVH(proxies).empty());
}

TEST(AABBPairTests, BVHSingleProxyProducesNoPairs) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
  };

  EXPECT_TRUE(findAABBPairsBVH(proxies).empty());
}

TEST(AABBPairTests, BVHSeparatedTwoProxyInputProducesNoPairs) {
  const std::vector<AABBProxy> proxies = {
      {0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}},
      {1, {Vec3(2.f, 2.f, 2.f), Vec3(3.f, 3.f, 3.f)}},
  };

  expectPairsEqual(findAABBPairsBVH(proxies), findAABBPairs(proxies));
}

TEST(AABBPairTests, BVHOverlappingTwoProxyInputProducesOnePair) {
  const std::vector<AABBProxy> proxies = {
      {10, {Vec3(0.f, 0.f, 0.f), Vec3(2.f, 2.f, 2.f)}},
      {20, {Vec3(1.f, 1.f, 1.f), Vec3(3.f, 3.f, 3.f)}},
  };

  expectPairsEqual(findAABBPairsBVH(proxies), findAABBPairs(proxies));
}

TEST(AABBPairTests, BVHOutputVectorOverloadClearsPreviousContents) {
  const std::vector<AABBProxy> proxies;

  std::vector<AABBPair> pairs = {{100, 200}};
  AABBBVHScratch scratch;
  findAABBPairsBVH(proxies, scratch, pairs);

  EXPECT_TRUE(pairs.empty());
}
