#pragma once

#include "collision/AABBPair.hpp"
#include "core/JobSystem.hpp"
#include "math/Vec3.hpp"
#include "physics/ContactSolver.hpp"
#include "physics/RigidBody.hpp"
#include "physics/RigidBodyContact.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

class RigidBodySystemTestAccess;
class RigidBodySystemInstrumentationAccess;

struct RigidBodyCollisionPipelineStats {
  std::size_t bodyCount = 0;
  std::size_t aabbCandidatePairCount = 0;
  std::size_t contactCount = 0;
};

class RigidBodySystem {
public:
  using RigidBodyId = std::size_t;

  RigidBodySystem() = default;
  RigidBodySystem(RigidBodySystem&&) = default;
  RigidBodySystem& operator=(RigidBodySystem&&) = default;
  RigidBodySystem(const RigidBodySystem&) = delete;
  RigidBodySystem& operator=(const RigidBodySystem&) = delete;
  ~RigidBodySystem() = default;

  RigidBodyId createRigidBody(float mass, float radius);
  RigidBody& rigidBody(RigidBodyId id);
  const RigidBody& rigidBody(RigidBodyId id) const;

  void reserveRigidBodies(std::size_t capacity);

  void reserveCollisionCapacity(std::size_t candidatePairs, std::size_t contacts);
  std::size_t reservedStorageBytes() const;
  std::size_t bodyCount() const;
  RigidBodyCollisionPipelineStats collisionPipelineStats() const;
  const ContactSolverSettings& contactSolverSettings() const;
  void setContactSolverSettings(const ContactSolverSettings& settings);

  unsigned workerThreadCount() const;
  void setWorkerThreadCount(unsigned threadCount);

  void step(float dt, const Vec3& gravity);

private:
  friend class RigidBodySystemTestAccess;
  friend class RigidBodySystemInstrumentationAccess;

  void integrate(float dt, const Vec3& gravity);
  void buildAABBProxies();
  void resolveCollisions();
  void buildContacts();

  template <typename Fn> void forRange(std::size_t count, std::size_t grainSize, Fn&& fn);

  std::vector<RigidBody> m_bodies;
  std::vector<AABBProxy> m_aabbProxies;
  std::vector<AABBPair> m_aabbPairs;
  AABBSweepAndPruneScratch m_aabbSweepScratch;

  std::vector<RigidBodyContact> m_contactSlots;
  std::vector<std::uint8_t> m_contactSlotValid;
  std::vector<RigidBodyContact> m_contacts;
  ContactSolver m_contactSolver;

  std::unique_ptr<JobSystem> m_jobs;
};
