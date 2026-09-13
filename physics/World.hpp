#pragma once
#include "math/Vec3.hpp"

#include "physics/RigidBodySystem.hpp"

#include <cstddef>
class World {
public:
  using RigidBodyId = RigidBodySystem::RigidBodyId;

  World(Vec3 gravity);
  World(World&&) = default;
  World(const World&) = delete;
  World& operator=(World&&) = default;
  World& operator=(const World&) = delete;
  ~World() = default;

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

  void step(float dt);

private:
  Vec3 m_gravity;

  RigidBodySystem m_rigidBodySystem;
};
