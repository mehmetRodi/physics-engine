#include "collision/AABBPair.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#define PHYSICS_SWEEP_NEON 1
#include <arm_neon.h>
#elif defined(__SSE2__) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2) || defined(_M_X64)
#define PHYSICS_SWEEP_SSE2 1
#include <emmintrin.h>
#endif

namespace {
AABB mergeAABBs(const AABB& a, const AABB& b) {
  return {
      Vec3(std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y), std::min(a.min.z, b.min.z)),
      Vec3(std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y), std::max(a.max.z, b.max.z)),
  };
}

Vec3 aabbCentroid(const AABB& bounds) {
  return (bounds.min + bounds.max) * 0.5f;
}

float centroidComponent(const AABBProxy& proxy, int axis) {
  const Vec3 centroid = aabbCentroid(proxy.bounds);

  if (axis == 0) {
    return centroid.x;
  }

  if (axis == 1) {
    return centroid.y;
  }

  return centroid.z;
}

int longestCentroidAxis(const std::vector<AABBProxy>& proxies,
                        const std::vector<std::size_t>& proxyIndices, std::size_t begin,
                        std::size_t end) {
  Vec3 minCentroid = aabbCentroid(proxies[proxyIndices[begin]].bounds);
  Vec3 maxCentroid = minCentroid;

  for (std::size_t i = begin + 1; i < end; ++i) {
    const Vec3 centroid = aabbCentroid(proxies[proxyIndices[i]].bounds);

    minCentroid.x = std::min(minCentroid.x, centroid.x);
    minCentroid.y = std::min(minCentroid.y, centroid.y);
    minCentroid.z = std::min(minCentroid.z, centroid.z);

    maxCentroid.x = std::max(maxCentroid.x, centroid.x);
    maxCentroid.y = std::max(maxCentroid.y, centroid.y);
    maxCentroid.z = std::max(maxCentroid.z, centroid.z);
  }

  const Vec3 extent = maxCentroid - minCentroid;

  if (extent.x >= extent.y && extent.x >= extent.z) {
    return 0;
  }

  if (extent.y >= extent.z) {
    return 1;
  }

  return 2;
}

std::size_t buildBVHNode(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch,
                         std::size_t begin, std::size_t end) {
  if (end - begin == 1) {
    const std::size_t proxyIndex = scratch.proxyIndices[begin];
    const std::size_t nodeIndex = scratch.nodes.size();
    scratch.nodes.push_back({proxies[proxyIndex].bounds, 0, 0, proxyIndex, true});
    return nodeIndex;
  }

  const int axis = longestCentroidAxis(proxies, scratch.proxyIndices, begin, end);
  const std::size_t mid = begin + (end - begin) / 2;

  std::nth_element(scratch.proxyIndices.begin() + static_cast<std::ptrdiff_t>(begin),
                   scratch.proxyIndices.begin() + static_cast<std::ptrdiff_t>(mid),
                   scratch.proxyIndices.begin() + static_cast<std::ptrdiff_t>(end),
                   [&proxies, axis](std::size_t lhs, std::size_t rhs) {
                     const float lhsCentroid = centroidComponent(proxies[lhs], axis);
                     const float rhsCentroid = centroidComponent(proxies[rhs], axis);

                     if (lhsCentroid != rhsCentroid) {
                       return lhsCentroid < rhsCentroid;
                     }

                     return lhs < rhs;
                   });

  const std::size_t leftChild = buildBVHNode(proxies, scratch, begin, mid);
  const std::size_t rightChild = buildBVHNode(proxies, scratch, mid, end);

  const std::size_t nodeIndex = scratch.nodes.size();
  scratch.nodes.push_back({
      mergeAABBs(scratch.nodes[leftChild].bounds, scratch.nodes[rightChild].bounds),
      leftChild,
      rightChild,
      0,
      false,
  });
  return nodeIndex;
}

void emitOverlappingLeafPairs(const AABBBVHScratch& scratch, std::size_t leftNodeIndex,
                              std::size_t rightNodeIndex,
                              std::vector<AABBPair>& outProxyIndexPairs) {
  const AABBBVHNode& left = scratch.nodes[leftNodeIndex];
  const AABBBVHNode& right = scratch.nodes[rightNodeIndex];

  if (!left.bounds.overlaps(right.bounds)) {
    return;
  }

  if (left.leaf && right.leaf) {
    const std::size_t firstIndex = std::min(left.proxyIndex, right.proxyIndex);
    const std::size_t secondIndex = std::max(left.proxyIndex, right.proxyIndex);
    outProxyIndexPairs.push_back({firstIndex, secondIndex});
    return;
  }

  if (left.leaf) {
    emitOverlappingLeafPairs(scratch, leftNodeIndex, right.leftChild, outProxyIndexPairs);
    emitOverlappingLeafPairs(scratch, leftNodeIndex, right.rightChild, outProxyIndexPairs);
    return;
  }

  if (right.leaf) {
    emitOverlappingLeafPairs(scratch, left.leftChild, rightNodeIndex, outProxyIndexPairs);
    emitOverlappingLeafPairs(scratch, left.rightChild, rightNodeIndex, outProxyIndexPairs);
    return;
  }

  emitOverlappingLeafPairs(scratch, left.leftChild, right.leftChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, left.leftChild, right.rightChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, left.rightChild, right.leftChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, left.rightChild, right.rightChild, outProxyIndexPairs);
}

void collectBVHPairs(const AABBBVHScratch& scratch, std::size_t nodeIndex,
                     std::vector<AABBPair>& outProxyIndexPairs) {
  const AABBBVHNode& node = scratch.nodes[nodeIndex];

  if (node.leaf) {
    return;
  }

  collectBVHPairs(scratch, node.leftChild, outProxyIndexPairs);
  collectBVHPairs(scratch, node.rightChild, outProxyIndexPairs);
  emitOverlappingLeafPairs(scratch, node.leftChild, node.rightChild, outProxyIndexPairs);
}

void sortPairsByProxyIndex(std::vector<AABBPair>& pairs) {
  std::sort(pairs.begin(), pairs.end(), [](const AABBPair& lhs, const AABBPair& rhs) {
    if (lhs.a != rhs.a) {
      return lhs.a < rhs.a;
    }

    return lhs.b < rhs.b;
  });
}
} // namespace

void AABBSweepAndPruneScratch::reserve(std::size_t proxyCapacity, std::size_t pairCapacity) {
  if (proxyCapacity > sortedMinX.max_size() - 3)
    throw std::length_error("proxy capacity overflow");
  sortedProxyIndices.reserve(proxyCapacity);
  candidateProxyIndexPairs.reserve(pairCapacity);

  const std::size_t padded = proxyCapacity + 3;
  sortedMinX.reserve(padded);
  sortedMaxX.reserve(padded);
  sortedMinY.reserve(padded);
  sortedMaxY.reserve(padded);
  sortedMinZ.reserve(padded);
  sortedMaxZ.reserve(padded);
}

bool hasSimdSweepAndPrune() {
#if defined(PHYSICS_SWEEP_NEON) || defined(PHYSICS_SWEEP_SSE2)
  return true;
#else
  return false;
#endif
}

void AABBBVHScratch::reserve(std::size_t proxyCapacity, std::size_t pairCapacity) {
  if (proxyCapacity > nodes.max_size() / 2)
    throw std::length_error("BVH capacity overflow");
  proxyIndices.reserve(proxyCapacity);
  nodes.reserve(proxyCapacity == 0 ? 0 : proxyCapacity * 2 - 1);
  candidateProxyIndexPairs.reserve(pairCapacity);
}

void findAABBPairs(const std::vector<AABBProxy>& proxies, std::vector<AABBPair>& outPairs) {
  outPairs.clear();

  for (std::size_t i = 0; i < proxies.size(); ++i) {
    const AABBProxy& a = proxies[i];

    for (std::size_t j = i + 1; j < proxies.size(); ++j) {
      const AABBProxy& b = proxies[j];

      if (a.bounds.overlaps(b.bounds)) {
        outPairs.push_back({a.id, b.id});
      }
    }
  }
}

std::vector<AABBPair> findAABBPairs(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  findAABBPairs(proxies, pairs);
  return pairs;
}

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                std::vector<AABBPair>& outPairs) {
  AABBSweepAndPruneScratch scratch;
  findAABBPairsSweepAndPrune(proxies, scratch, outPairs);
}

namespace {

void sortProxyIndicesByMinX(const std::vector<AABBProxy>& proxies,
                            std::vector<std::size_t>& sortedProxyIndices) {
  sortedProxyIndices.resize(proxies.size());
  std::iota(sortedProxyIndices.begin(), sortedProxyIndices.end(), std::size_t{0});

  std::sort(sortedProxyIndices.begin(), sortedProxyIndices.end(),
            [&proxies](std::size_t lhs, std::size_t rhs) {
              const float lhsMinX = proxies[lhs].bounds.min.x;
              const float rhsMinX = proxies[rhs].bounds.min.x;

              if (lhsMinX != rhsMinX) {
                return lhsMinX < rhsMinX;
              }

              return lhs < rhs;
            });
}

void packSortedIntervals(const std::vector<AABBProxy>& proxies, AABBSweepAndPruneScratch& scratch) {
  const std::size_t count = proxies.size();

  scratch.sortedMinX.resize(count + 3);
  scratch.sortedMaxX.resize(count + 3);
  scratch.sortedMinY.resize(count + 3);
  scratch.sortedMaxY.resize(count + 3);
  scratch.sortedMinZ.resize(count + 3);
  scratch.sortedMaxZ.resize(count + 3);

  for (std::size_t i = 0; i < count; ++i) {
    const AABB& bounds = proxies[scratch.sortedProxyIndices[i]].bounds;

    scratch.sortedMinX[i] = bounds.min.x;
    scratch.sortedMaxX[i] = bounds.max.x;
    scratch.sortedMinY[i] = bounds.min.y;
    scratch.sortedMaxY[i] = bounds.max.y;
    scratch.sortedMinZ[i] = bounds.min.z;
    scratch.sortedMaxZ[i] = bounds.max.z;
  }

  for (std::size_t i = count; i < count + 3; ++i) {
    scratch.sortedMinX[i] = std::numeric_limits<float>::infinity();
    scratch.sortedMaxX[i] = std::numeric_limits<float>::infinity();
    scratch.sortedMinY[i] = std::numeric_limits<float>::infinity();
    scratch.sortedMaxY[i] = std::numeric_limits<float>::infinity();
    scratch.sortedMinZ[i] = std::numeric_limits<float>::infinity();
    scratch.sortedMaxZ[i] = std::numeric_limits<float>::infinity();
  }
}

void emitCandidate(const AABBSweepAndPruneScratch& scratch, std::size_t sortedI,
                   std::size_t sortedJ, std::vector<AABBPair>& candidates) {
  const std::size_t proxyI = scratch.sortedProxyIndices[sortedI];
  const std::size_t proxyJ = scratch.sortedProxyIndices[sortedJ];

  candidates.push_back({std::min(proxyI, proxyJ), std::max(proxyI, proxyJ)});
}

void sweepIndirect(const std::vector<AABBProxy>& proxies, AABBSweepAndPruneScratch& scratch) {
  const std::size_t count = scratch.sortedProxyIndices.size();

  for (std::size_t sortedI = 0; sortedI < count; ++sortedI) {
    const AABBProxy& a = proxies[scratch.sortedProxyIndices[sortedI]];

    for (std::size_t sortedJ = sortedI + 1; sortedJ < count; ++sortedJ) {
      const AABBProxy& b = proxies[scratch.sortedProxyIndices[sortedJ]];

      if (b.bounds.min.x > a.bounds.max.x) {
        break;
      }

      if (a.bounds.overlaps(b.bounds)) {
        emitCandidate(scratch, sortedI, sortedJ, scratch.candidateProxyIndexPairs);
      }
    }
  }
}

void sweepPackedScalar(AABBSweepAndPruneScratch& scratch, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    const float maxXi = scratch.sortedMaxX[i];
    const float minXi = scratch.sortedMinX[i];
    const float maxYi = scratch.sortedMaxY[i];
    const float minYi = scratch.sortedMinY[i];
    const float maxZi = scratch.sortedMaxZ[i];
    const float minZi = scratch.sortedMinZ[i];

    for (std::size_t j = i + 1; j < count; ++j) {
      if (scratch.sortedMinX[j] > maxXi) {
        break;
      }

      if (scratch.sortedMaxX[j] >= minXi && scratch.sortedMinY[j] <= maxYi &&
          scratch.sortedMaxY[j] >= minYi && scratch.sortedMinZ[j] <= maxZi &&
          scratch.sortedMaxZ[j] >= minZi) {
        emitCandidate(scratch, i, j, scratch.candidateProxyIndexPairs);
      }
    }
  }
}

#if defined(PHYSICS_SWEEP_NEON) || defined(PHYSICS_SWEEP_SSE2)
void sweepPackedSimd(AABBSweepAndPruneScratch& scratch, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
#if defined(PHYSICS_SWEEP_NEON)
    const float32x4_t maxXi = vdupq_n_f32(scratch.sortedMaxX[i]);
    const float32x4_t minXi = vdupq_n_f32(scratch.sortedMinX[i]);
    const float32x4_t maxYi = vdupq_n_f32(scratch.sortedMaxY[i]);
    const float32x4_t minYi = vdupq_n_f32(scratch.sortedMinY[i]);
    const float32x4_t maxZi = vdupq_n_f32(scratch.sortedMaxZ[i]);
    const float32x4_t minZi = vdupq_n_f32(scratch.sortedMinZ[i]);
    const uint32x4_t laneBits = {1u, 2u, 4u, 8u};
#else
    const __m128 maxXi = _mm_set1_ps(scratch.sortedMaxX[i]);
    const __m128 minXi = _mm_set1_ps(scratch.sortedMinX[i]);
    const __m128 maxYi = _mm_set1_ps(scratch.sortedMaxY[i]);
    const __m128 minYi = _mm_set1_ps(scratch.sortedMinY[i]);
    const __m128 maxZi = _mm_set1_ps(scratch.sortedMaxZ[i]);
    const __m128 minZi = _mm_set1_ps(scratch.sortedMinZ[i]);
#endif

    for (std::size_t j = i + 1; j < count; j += 4) {
      const std::size_t remaining = count - j;

#if defined(PHYSICS_SWEEP_NEON)
      const float32x4_t minXj = vld1q_f32(&scratch.sortedMinX[j]);
      const uint32x4_t withinSweep = vcleq_f32(minXj, maxXi);

      uint32x4_t overlap = withinSweep;
      overlap = vandq_u32(overlap, vcgeq_f32(vld1q_f32(&scratch.sortedMaxX[j]), minXi));
      overlap = vandq_u32(overlap, vcleq_f32(vld1q_f32(&scratch.sortedMinY[j]), maxYi));
      overlap = vandq_u32(overlap, vcgeq_f32(vld1q_f32(&scratch.sortedMaxY[j]), minYi));
      overlap = vandq_u32(overlap, vcleq_f32(vld1q_f32(&scratch.sortedMinZ[j]), maxZi));
      overlap = vandq_u32(overlap, vcgeq_f32(vld1q_f32(&scratch.sortedMaxZ[j]), minZi));

      const std::uint32_t sweepMask = vaddvq_u32(vandq_u32(withinSweep, laneBits));
      std::uint32_t overlapMask = vaddvq_u32(vandq_u32(overlap, laneBits));
#else
      const __m128 minXj = _mm_loadu_ps(&scratch.sortedMinX[j]);
      const __m128 withinSweep = _mm_cmple_ps(minXj, maxXi);

      __m128 overlap = withinSweep;
      overlap = _mm_and_ps(overlap, _mm_cmpge_ps(_mm_loadu_ps(&scratch.sortedMaxX[j]), minXi));
      overlap = _mm_and_ps(overlap, _mm_cmple_ps(_mm_loadu_ps(&scratch.sortedMinY[j]), maxYi));
      overlap = _mm_and_ps(overlap, _mm_cmpge_ps(_mm_loadu_ps(&scratch.sortedMaxY[j]), minYi));
      overlap = _mm_and_ps(overlap, _mm_cmple_ps(_mm_loadu_ps(&scratch.sortedMinZ[j]), maxZi));
      overlap = _mm_and_ps(overlap, _mm_cmpge_ps(_mm_loadu_ps(&scratch.sortedMaxZ[j]), minZi));

      const std::uint32_t sweepMask = static_cast<std::uint32_t>(_mm_movemask_ps(withinSweep));
      std::uint32_t overlapMask = static_cast<std::uint32_t>(_mm_movemask_ps(overlap));
#endif

      const std::uint32_t validLanes =
          remaining >= 4 ? 0xfu : (1u << static_cast<std::uint32_t>(remaining)) - 1u;
      overlapMask &= validLanes;

      while (overlapMask != 0) {
        const std::uint32_t lane = static_cast<std::uint32_t>(std::countr_zero(overlapMask));
        overlapMask &= overlapMask - 1;
        emitCandidate(scratch, i, j + lane, scratch.candidateProxyIndexPairs);
      }

      if ((sweepMask & validLanes) != validLanes) {
        break;
      }
    }
  }
}
#endif

} // namespace

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                AABBSweepAndPruneScratch& scratch, std::vector<AABBPair>& outPairs,
                                SweepAndPruneMode mode) {
  outPairs.clear();
  scratch.sortedProxyIndices.clear();
  scratch.candidateProxyIndexPairs.clear();

  if (proxies.size() < 2) {
    return;
  }

  sortProxyIndicesByMinX(proxies, scratch.sortedProxyIndices);

  if (mode == SweepAndPruneMode::ScalarIndirect) {
    sweepIndirect(proxies, scratch);
  } else {
    packSortedIntervals(proxies, scratch);

#if defined(PHYSICS_SWEEP_NEON) || defined(PHYSICS_SWEEP_SSE2)
    if (mode == SweepAndPruneMode::Simd) {
      sweepPackedSimd(scratch, proxies.size());
    } else {
      sweepPackedScalar(scratch, proxies.size());
    }
#else
    sweepPackedScalar(scratch, proxies.size());
#endif
  }

  std::sort(scratch.candidateProxyIndexPairs.begin(), scratch.candidateProxyIndexPairs.end(),
            [](const AABBPair& lhs, const AABBPair& rhs) {
              if (lhs.a != rhs.a) {
                return lhs.a < rhs.a;
              }

              return lhs.b < rhs.b;
            });

  outPairs.reserve(scratch.candidateProxyIndexPairs.size());
  for (const AABBPair& candidate : scratch.candidateProxyIndexPairs) {
    outPairs.push_back({proxies[candidate.a].id, proxies[candidate.b].id});
  }
}

std::vector<AABBPair> findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  findAABBPairsSweepAndPrune(proxies, pairs);
  return pairs;
}

void findAABBPairsBVH(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch,
                      std::vector<AABBPair>& outPairs) {
  outPairs.clear();
  scratch.candidateProxyIndexPairs.clear();

  const AABBBVHBuildResult buildResult = buildAABBBVH(proxies, scratch);
  if (buildResult.empty) {
    return;
  }

  collectAABBBVHPairCandidates(scratch, buildResult.rootNode, scratch.candidateProxyIndexPairs);
  emitSortedAABBPairsFromProxyIndexPairs(proxies, scratch.candidateProxyIndexPairs, outPairs);
}

std::vector<AABBPair> findAABBPairsBVH(const std::vector<AABBProxy>& proxies) {
  std::vector<AABBPair> pairs;
  AABBBVHScratch scratch;
  findAABBPairsBVH(proxies, scratch, pairs);
  return pairs;
}

AABBBVHBuildResult buildAABBBVH(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch) {
  scratch.proxyIndices.clear();
  scratch.nodes.clear();

  if (proxies.size() < 2) {
    return {};
  }

  scratch.proxyIndices.resize(proxies.size());
  std::iota(scratch.proxyIndices.begin(), scratch.proxyIndices.end(), std::size_t{0});

  return {buildBVHNode(proxies, scratch, 0, scratch.proxyIndices.size()), false};
}

void collectAABBBVHPairCandidates(const AABBBVHScratch& scratch, std::size_t rootNode,
                                  std::vector<AABBPair>& outProxyIndexPairs) {
  outProxyIndexPairs.clear();
  collectBVHPairs(scratch, rootNode, outProxyIndexPairs);
}

void emitSortedAABBPairsFromProxyIndexPairs(const std::vector<AABBProxy>& proxies,
                                            std::vector<AABBPair>& proxyIndexPairs,
                                            std::vector<AABBPair>& outPairs) {
  outPairs.clear();
  sortPairsByProxyIndex(proxyIndexPairs);
  outPairs.reserve(proxyIndexPairs.size());

  for (const AABBPair& candidate : proxyIndexPairs) {
    outPairs.push_back({proxies[candidate.a].id, proxies[candidate.b].id});
  }
}
