#include "physics/RigidBodySystem.hpp"

#include "collision/AABB.hpp"
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
bool buildSphereSphereContact(RigidBodySystem::RigidBodyId aId, const RigidBody& a,
                              RigidBodySystem::RigidBodyId bId, const RigidBody& b,
                              RigidBodyContact& outContact) {
  const Vec3 offset = a.position - b.position;
  const float distanceSq = offset.lengthSq();

  const float distance = offset.length();
  const float penetration = a.shape().sphereRadius + b.shape().sphereRadius - distance;

  if (penetration <= 0.0f) {
    return false;
  }

  const Vec3 normal =
      distanceSq >= 1e-12f ? offset / distance : Vec3(aId < bId ? 1.f : -1.f, 0.f, 0.f);

  outContact = {
      aId,
      bId,
      normal,

      ((a.position - normal * a.shape().sphereRadius) +
       (b.position + normal * b.shape().sphereRadius)) *
          0.5f,
      penetration,
      std::max(a.material.restitution, b.material.restitution),
      static_cast<float>(std::sqrt(double(a.material.friction) * b.material.friction)),
  };
  return true;
}

bool buildContactForShapes(RigidBodySystem::RigidBodyId aId, const RigidBody& a,
                           RigidBodySystem::RigidBodyId bId, const RigidBody& b,
                           RigidBodyContact& outContact) {
  if (a.shape().type == ShapeType::Sphere && b.shape().type == ShapeType::Sphere) {
    return buildSphereSphereContact(aId, a, bId, b, outContact);
  }

  return false;
}

AABB buildAABBForShape(const RigidBody& body) {
  switch (body.shape().type) {
  case ShapeType::Sphere:
    return makeAABBForSphere(body.position, body.shape().sphereRadius);
  }

  return makeAABBForSphere(body.position, 0.0f);
}
} // namespace

constexpr std::size_t kIntegrateGrain = 256;
constexpr std::size_t kProxyGrain = 256;
constexpr std::size_t kContactGrain = 128;

RigidBodySystem::RigidBodyId RigidBodySystem::createRigidBody(float mass, float radius) {
  RigidBodyId id = m_bodies.size();
  m_bodies.emplace_back(mass, radius);
  return id;
}

RigidBody& RigidBodySystem::rigidBody(RigidBodyId id) {
  return m_bodies.at(id);
}

const RigidBody& RigidBodySystem::rigidBody(RigidBodyId id) const {
  return m_bodies.at(id);
}

void RigidBodySystem::reserveRigidBodies(std::size_t capacity) {
  m_bodies.reserve(capacity);
  m_aabbProxies.reserve(capacity);

  m_aabbSweepScratch.reserve(capacity);
}

void RigidBodySystem::reserveCollisionCapacity(std::size_t candidatePairs, std::size_t contacts) {
  m_aabbPairs.reserve(candidatePairs);
  m_aabbSweepScratch.candidateProxyIndexPairs.reserve(candidatePairs);
  m_contactSlots.reserve(candidatePairs);
  m_contactSlotValid.reserve(candidatePairs);
  m_contacts.reserve(contacts);
  m_contactSolver.reserveContacts(contacts);
}

std::size_t RigidBodySystem::reservedStorageBytes() const {
  const auto bytes = [](const auto& v) {
    return v.capacity() * sizeof(typename std::decay_t<decltype(v)>::value_type);
  };
  const auto& s = m_aabbSweepScratch;
  return bytes(m_bodies) + bytes(m_aabbProxies) + bytes(m_aabbPairs) + bytes(s.sortedProxyIndices) +
         bytes(s.candidateProxyIndexPairs) + bytes(s.sortedMinX) + bytes(s.sortedMaxX) +
         bytes(s.sortedMinY) + bytes(s.sortedMaxY) + bytes(s.sortedMinZ) + bytes(s.sortedMaxZ) +
         bytes(m_contactSlots) + bytes(m_contactSlotValid) + bytes(m_contacts) +
         m_contactSolver.reservedStorageBytes();
}

std::size_t RigidBodySystem::bodyCount() const {
  return m_bodies.size();
}

RigidBodyCollisionPipelineStats RigidBodySystem::collisionPipelineStats() const {
  return {
      m_bodies.size(),
      m_aabbPairs.size(),
      m_contacts.size(),
  };
}

const ContactSolverSettings& RigidBodySystem::contactSolverSettings() const {
  return m_contactSolver.settings();
}

void RigidBodySystem::setContactSolverSettings(const ContactSolverSettings& settings) {
  m_contactSolver.setSettings(settings);
}

unsigned RigidBodySystem::workerThreadCount() const {
  return m_jobs ? m_jobs->threadCount() : 1u;
}

void RigidBodySystem::setWorkerThreadCount(unsigned threadCount) {
  if (threadCount == 0)
    threadCount = std::max(1u, std::thread::hardware_concurrency());
  if (threadCount == 1) {
    m_jobs.reset();
    return;
  }

  if (m_jobs && m_jobs->threadCount() == threadCount) {
    return;
  }

  m_jobs.reset();
  m_jobs = std::make_unique<JobSystem>(threadCount);
}

template <typename Fn>
void RigidBodySystem::forRange(std::size_t count, std::size_t grainSize, Fn&& fn) {
  if (m_jobs) {
    m_jobs->parallelFor(count, grainSize, fn);
  } else {
    fn(std::size_t{0}, count);
  }
}

void RigidBodySystem::step(float dt, const Vec3& gravity) {
  if (!std::isfinite(dt) || dt < 0.f || !std::isfinite(gravity.x) || !std::isfinite(gravity.y) ||
      !std::isfinite(gravity.z))
    throw std::invalid_argument("dt must be finite/nonnegative and gravity finite");
  for (const auto& body : m_bodies)
    body.validate();
  integrate(dt, gravity);
  resolveCollisions();
}

void RigidBodySystem::integrate(float dt, const Vec3& gravity) {

  forRange(m_bodies.size(), kIntegrateGrain,
           [this, dt, &gravity](std::size_t begin, std::size_t end) {
             for (std::size_t i = begin; i < end; ++i) {
               RigidBody& body = m_bodies[i];
               if (!body.isStatic())
                 body.acceleration += gravity;
               body.update(dt);
             }
           });
}

void RigidBodySystem::buildAABBProxies() {

  m_aabbProxies.resize(m_bodies.size());

  forRange(m_bodies.size(), kProxyGrain, [this](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      m_bodies[i].validate();
      const AABB bounds = buildAABBForShape(m_bodies[i]);
      if (!std::isfinite(bounds.min.x) || !std::isfinite(bounds.min.y) ||
          !std::isfinite(bounds.min.z) || !std::isfinite(bounds.max.x) ||
          !std::isfinite(bounds.max.y) || !std::isfinite(bounds.max.z))
        throw std::overflow_error("nonfinite collision bounds");
      m_aabbProxies[i] = {i, bounds};
    }
  });
}

void RigidBodySystem::resolveCollisions() {
  buildAABBProxies();

  findAABBPairsSweepAndPrune(m_aabbProxies, m_aabbSweepScratch, m_aabbPairs);
  buildContacts();

  m_contactSolver.solve(m_bodies, m_contacts);
}

void RigidBodySystem::buildContacts() {
  const std::size_t pairCount = m_aabbPairs.size();
  m_contactSlots.resize(pairCount);
  m_contactSlotValid.resize(pairCount);

  forRange(pairCount, kContactGrain, [this](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      const AABBPair& pair = m_aabbPairs[i];
      m_contactSlotValid[i] = buildContactForShapes(pair.a, m_bodies[pair.a], pair.b,
                                                    m_bodies[pair.b], m_contactSlots[i])
                                  ? 1
                                  : 0;
    }
  });

  m_contacts.clear();
  for (std::size_t i = 0; i < pairCount; ++i) {
    if (m_contactSlotValid[i]) {
      m_contacts.push_back(m_contactSlots[i]);
    }
  }
}
