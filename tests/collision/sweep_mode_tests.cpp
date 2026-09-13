#include "collision/AABBPair.hpp"
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

struct Lcg {
  std::uint32_t state = 0x12345678u;

  float next(float low, float high) {
    state = state * 1664525u + 1013904223u;
    const float unit = static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
    return low + unit * (high - low);
  }
};

std::vector<AABBProxy> makeProxies(std::size_t count, float spread, float size) {
  Lcg random;
  std::vector<AABBProxy> proxies;
  proxies.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const Vec3 centre(random.next(-spread, spread), random.next(-spread, spread),
                      random.next(-spread, spread));
    const float radius = random.next(0.1f, size);
    proxies.push_back({i, makeAABBForSphere(centre, radius)});
  }

  return proxies;
}

std::vector<AABBPair> run(const std::vector<AABBProxy>& proxies, SweepAndPruneMode mode) {
  AABBSweepAndPruneScratch scratch;
  std::vector<AABBPair> pairs;
  findAABBPairsSweepAndPrune(proxies, scratch, pairs, mode);
  return pairs;
}

void expectSamePairs(const std::vector<AABBPair>& actual, const std::vector<AABBPair>& expected) {
  ASSERT_EQ(actual.size(), expected.size());

  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(actual[i].a, expected[i].a);
    EXPECT_EQ(actual[i].b, expected[i].b);
  }
}

} // namespace

TEST(SweepModeTests, AllThreeModesAgreeAcrossProxyCounts) {

  for (const std::size_t count :
       {std::size_t{0}, std::size_t{1}, std::size_t{2}, std::size_t{3}, std::size_t{4},
        std::size_t{5}, std::size_t{7}, std::size_t{8}, std::size_t{9}, std::size_t{17},
        std::size_t{64}, std::size_t{100}, std::size_t{257}}) {
    const std::vector<AABBProxy> proxies = makeProxies(count, 10.f, 1.5f);

    const std::vector<AABBPair> indirect = run(proxies, SweepAndPruneMode::ScalarIndirect);

    SCOPED_TRACE(count);
    expectSamePairs(run(proxies, SweepAndPruneMode::ScalarPacked), indirect);
    expectSamePairs(run(proxies, SweepAndPruneMode::Simd), indirect);
  }
}

TEST(SweepModeTests, AllThreeModesAgreeWhenEverythingOverlaps) {

  const std::vector<AABBProxy> proxies = makeProxies(70, 0.05f, 4.f);
  const std::vector<AABBPair> indirect = run(proxies, SweepAndPruneMode::ScalarIndirect);

  EXPECT_EQ(indirect.size(), 70u * 69u / 2u);
  expectSamePairs(run(proxies, SweepAndPruneMode::ScalarPacked), indirect);
  expectSamePairs(run(proxies, SweepAndPruneMode::Simd), indirect);
}

TEST(SweepModeTests, AllThreeModesAgreeWhenNothingOverlaps) {
  const std::vector<AABBProxy> proxies = makeProxies(70, 500.f, 0.5f);
  const std::vector<AABBPair> indirect = run(proxies, SweepAndPruneMode::ScalarIndirect);

  EXPECT_TRUE(indirect.empty());
  expectSamePairs(run(proxies, SweepAndPruneMode::ScalarPacked), indirect);
  expectSamePairs(run(proxies, SweepAndPruneMode::Simd), indirect);
}

TEST(SweepModeTests, ModesAgreeWithTheBruteForceBaseline) {
  const std::vector<AABBProxy> proxies = makeProxies(120, 6.f, 1.f);
  const std::vector<AABBPair> bruteForce = findAABBPairs(proxies);

  expectSamePairs(run(proxies, SweepAndPruneMode::ScalarIndirect), bruteForce);
  expectSamePairs(run(proxies, SweepAndPruneMode::ScalarPacked), bruteForce);
  expectSamePairs(run(proxies, SweepAndPruneMode::Simd), bruteForce);
}

TEST(SweepModeTests, BoxesTouchingExactlyAreReportedIdenticallyByEveryMode) {

  std::vector<AABBProxy> proxies;
  proxies.push_back({0, {Vec3(0.f, 0.f, 0.f), Vec3(1.f, 1.f, 1.f)}});
  proxies.push_back({1, {Vec3(1.f, 0.f, 0.f), Vec3(2.f, 1.f, 1.f)}});
  proxies.push_back({2, {Vec3(2.000001f, 0.f, 0.f), Vec3(3.f, 1.f, 1.f)}});

  const std::vector<AABBPair> indirect = run(proxies, SweepAndPruneMode::ScalarIndirect);

  ASSERT_EQ(indirect.size(), 1u);
  EXPECT_EQ(indirect[0].a, 0u);
  EXPECT_EQ(indirect[0].b, 1u);

  expectSamePairs(run(proxies, SweepAndPruneMode::ScalarPacked), indirect);
  expectSamePairs(run(proxies, SweepAndPruneMode::Simd), indirect);
}

TEST(SweepModeTests, BuildReportsWhetherAVectorPathExists) {
#if defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(__SSE2__) || defined(_M_X64)
  EXPECT_TRUE(hasSimdSweepAndPrune());
#else
  EXPECT_FALSE(hasSimdSweepAndPrune());
#endif
}
