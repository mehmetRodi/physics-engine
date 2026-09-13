#include "physics/World.hpp"
#include <cmath>
#include <stdexcept>

World::World(Vec3 gravity) : m_gravity(gravity) {
  if (!std::isfinite(gravity.x) || !std::isfinite(gravity.y) || !std::isfinite(gravity.z))
    throw std::invalid_argument("gravity must be finite");
}

World::RigidBodyId World::createRigidBody(float mass, float radius) {
  return m_rigidBodySystem.createRigidBody(mass, radius);
}

RigidBody& World::rigidBody(RigidBodyId id) {
  return m_rigidBodySystem.rigidBody(id);
}

const RigidBody& World::rigidBody(RigidBodyId id) const {
  return m_rigidBodySystem.rigidBody(id);
}

void World::reserveRigidBodies(std::size_t capacity) {
  m_rigidBodySystem.reserveRigidBodies(capacity);
}

std::size_t World::bodyCount() const {
  return m_rigidBodySystem.bodyCount();
}

RigidBodyCollisionPipelineStats World::collisionPipelineStats() const {
  return m_rigidBodySystem.collisionPipelineStats();
}

const ContactSolverSettings& World::contactSolverSettings() const {
  return m_rigidBodySystem.contactSolverSettings();
}

void World::setContactSolverSettings(const ContactSolverSettings& settings) {
  m_rigidBodySystem.setContactSolverSettings(settings);
}

unsigned World::workerThreadCount() const {
  return m_rigidBodySystem.workerThreadCount();
}

void World::setWorkerThreadCount(unsigned threadCount) {
  m_rigidBodySystem.setWorkerThreadCount(threadCount);
}

void World::step(float dt) {
  m_rigidBodySystem.step(dt, m_gravity);
}

void World::reserveCollisionCapacity(std::size_t pairs, std::size_t contacts) {
  m_rigidBodySystem.reserveCollisionCapacity(pairs, contacts);
}
std::size_t World::reservedStorageBytes() const {
  return m_rigidBodySystem.reservedStorageBytes();
}
