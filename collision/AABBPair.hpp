#pragma once

#include "collision/AABB.hpp"

#include <cstddef>
#include <vector>

struct AABBProxy {
  std::size_t id;
  AABB bounds;
};

struct AABBPair {
  std::size_t a;
  std::size_t b;
};

enum class SweepAndPruneMode {

  ScalarIndirect,

  ScalarPacked,

  Simd,
};

struct AABBSweepAndPruneScratch {
  std::vector<std::size_t> sortedProxyIndices;

  std::vector<AABBPair> candidateProxyIndexPairs;

  std::vector<float> sortedMinX;
  std::vector<float> sortedMaxX;
  std::vector<float> sortedMinY;
  std::vector<float> sortedMaxY;
  std::vector<float> sortedMinZ;
  std::vector<float> sortedMaxZ;

  void reserve(std::size_t proxyCapacity, std::size_t pairCapacity = 0);
};

struct AABBBVHNode {
  AABB bounds;
  std::size_t leftChild = 0;
  std::size_t rightChild = 0;
  std::size_t proxyIndex = 0;
  bool leaf = false;
};

struct AABBBVHScratch {
  std::vector<std::size_t> proxyIndices;
  std::vector<AABBBVHNode> nodes;

  std::vector<AABBPair> candidateProxyIndexPairs;

  void reserve(std::size_t proxyCapacity, std::size_t pairCapacity = 0);
};

struct AABBBVHBuildResult {
  std::size_t rootNode = 0;
  bool empty = true;
};

void findAABBPairs(const std::vector<AABBProxy>& proxies, std::vector<AABBPair>& outPairs);

std::vector<AABBPair> findAABBPairs(const std::vector<AABBProxy>& proxies);

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                std::vector<AABBPair>& outPairs);

void findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies,
                                AABBSweepAndPruneScratch& scratch, std::vector<AABBPair>& outPairs,
                                SweepAndPruneMode mode = SweepAndPruneMode::Simd);

bool hasSimdSweepAndPrune();

std::vector<AABBPair> findAABBPairsSweepAndPrune(const std::vector<AABBProxy>& proxies);

void findAABBPairsBVH(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch,
                      std::vector<AABBPair>& outPairs);

std::vector<AABBPair> findAABBPairsBVH(const std::vector<AABBProxy>& proxies);

AABBBVHBuildResult buildAABBBVH(const std::vector<AABBProxy>& proxies, AABBBVHScratch& scratch);

void collectAABBBVHPairCandidates(const AABBBVHScratch& scratch, std::size_t rootNode,
                                  std::vector<AABBPair>& outProxyIndexPairs);

void emitSortedAABBPairsFromProxyIndexPairs(const std::vector<AABBProxy>& proxies,
                                            std::vector<AABBPair>& proxyIndexPairs,
                                            std::vector<AABBPair>& outPairs);
