#include <gtest/gtest.h>

#include "physics/World.hpp"

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <new>

#if defined(__SANITIZE_ADDRESS__)
#define PHYSICS_ALLOCATION_TRACKING 0
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
#define PHYSICS_ALLOCATION_TRACKING 0
#endif
#endif
#ifndef PHYSICS_ALLOCATION_TRACKING
#define PHYSICS_ALLOCATION_TRACKING 1
#endif

#if PHYSICS_ALLOCATION_TRACKING

namespace {
std::atomic<bool> g_track{false};
std::atomic<std::size_t> g_allocations{0};
void countAllocation() {
  if (g_track.load(std::memory_order_relaxed))
    g_allocations.fetch_add(1, std::memory_order_relaxed);
}
struct Tracking {
  Tracking() {
    g_allocations.store(0);
    g_track.store(true);
  }
  ~Tracking() {
    g_track.store(false);
  }
};
} // namespace
void* operator new(std::size_t size) {
  countAllocation();
  if (void* p = std::malloc(size ? size : 1))
    return p;
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
  return ::operator new(size);
}
void operator delete(void* p) noexcept {
  std::free(p);
}
void operator delete[](void* p) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}
void* operator new(std::size_t size, std::align_val_t alignment) {
  countAllocation();
  const auto a = static_cast<std::size_t>(alignment);
  size = size ? size : 1;
  if (size > std::numeric_limits<std::size_t>::max() - (a - 1))
    throw std::bad_alloc();
  if (void* p = std::aligned_alloc(a, (size + a - 1) / a * a))
    return p;
  throw std::bad_alloc();
}
void* operator new[](std::size_t size, std::align_val_t a) {
  return ::operator new(size, a);
}
void operator delete(void* p, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}

TEST(WorldAllocationTests, ReservedFirstAndSteadyStateTicksDoNotAllocateAcrossWorkerCounts) {
  for (unsigned threads : {1u, 2u, 4u, 8u}) {
    SCOPED_TRACE(threads);
    World world{Vec3()};
    world.reserveRigidBodies(512);
    world.reserveCollisionCapacity(256, 256);
    world.setWorkerThreadCount(threads);
    for (int i = 0; i < 512; ++i) {
      auto id = world.createRigidBody(1.f, .5f);
      world.rigidBody(id).position = Vec3(float(i / 2) * 4.f, (i % 2) * .9f, 0);
    }
    for (int step = 0; step < 20; ++step) {

      for (std::size_t i = 0; i < 512; ++i) {
        world.rigidBody(i).position.y = (i % 2) * (step % 2 ? 3.f : .9f);
        world.rigidBody(i).velocity = Vec3();
      }
      {
        Tracking tracking;
        world.step(1.f / 60);
      }
      EXPECT_EQ(g_allocations.load(), 0u) << "tick " << step;
    }
  }
}

TEST(WorldAllocationTests, CounterObservesAlignedAllocationsFromWorkers) {
  JobSystem jobs(4);
  {
    Tracking tracking;
    jobs.parallelFor(100, 1, [](std::size_t begin, std::size_t end) {
      for (auto i = begin; i < end; ++i) {
        void* p = ::operator new(128, std::align_val_t{128});
        ::operator delete(p, std::align_val_t{128});
      }
    });
  }
  EXPECT_EQ(g_allocations.load(), 100u);
}
#endif
