#include "physics/ContactSolver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

float reciprocal(float value) {
  return value > 0.f ? 1.f / value : 0.f;
}

Vec3 orthogonalAxis(const Vec3& normal) {

  const Vec3 reference = std::abs(normal.x) < 0.57735f ? Vec3(1.f, 0.f, 0.f) : Vec3(0.f, 1.f, 0.f);
  return normal.cross(reference).normalized();
}

Vec3 velocityAtOffset(const RigidBody& body, const Vec3& offset) {
  return body.velocity + body.angularVelocity.cross(offset);
}

float inverseEffectiveMass(const RigidBody& body1, const Vec3& offset1, const RigidBody& body2,
                           const Vec3& offset2, const Vec3& axis) {
  const Vec3 angular1 = (body1.invInertiaWorld() * offset1.cross(axis)).cross(offset1);
  const Vec3 angular2 = (body2.invInertiaWorld() * offset2.cross(axis)).cross(offset2);

  return body1.invMass() + body2.invMass() + axis.dot(angular1) + axis.dot(angular2);
}

void applyImpulse(RigidBody& body1, const Vec3& offset1, RigidBody& body2, const Vec3& offset2,
                  const Vec3& impulse) {
  body1.velocity += impulse * body1.invMass();
  body1.angularVelocity += body1.invInertiaWorld() * offset1.cross(impulse);

  body2.velocity -= impulse * body2.invMass();
  body2.angularVelocity -= body2.invInertiaWorld() * offset2.cross(impulse);
}

void applyFrictionAxis(RigidBody& body1, const Vec3& offset1, RigidBody& body2, const Vec3& offset2,
                       const Vec3& tangent, float tangentMass, float maxImpulse,
                       float& accumulatedImpulse) {
  const float slidingSpeed =
      (velocityAtOffset(body1, offset1) - velocityAtOffset(body2, offset2)).dot(tangent);
  float impulse = -slidingSpeed * tangentMass;

  const float previousImpulse = accumulatedImpulse;
  accumulatedImpulse = std::clamp(previousImpulse + impulse, -maxImpulse, maxImpulse);
  impulse = accumulatedImpulse - previousImpulse;

  applyImpulse(body1, offset1, body2, offset2, tangent * impulse);
}

} // namespace

const ContactSolverSettings& ContactSolver::settings() const {
  return m_settings;
}

void ContactSolver::setSettings(const ContactSolverSettings& settings) {
  if (settings.velocityIterations < 0 || settings.velocityIterations > 1024 ||
      !std::isfinite(settings.restitutionThreshold) || settings.restitutionThreshold < 0.f ||
      !std::isfinite(settings.positionCorrectionRate) || settings.positionCorrectionRate < 0.f ||
      settings.positionCorrectionRate > 1.f || !std::isfinite(settings.penetrationSlop) ||
      settings.penetrationSlop < 0.f || !std::isfinite(settings.maxPositionCorrection) ||
      settings.maxPositionCorrection < 0.f)
    throw std::invalid_argument("invalid contact solver settings");
  m_settings = settings;
  if (!settings.warmStarting)
    m_previousImpulses.clear();
}

void ContactSolver::reserveContacts(std::size_t capacity) {
  m_constraints.reserve(capacity);
  m_previousImpulses.reserve(capacity);
}

std::size_t ContactSolver::reservedStorageBytes() const {
  return m_constraints.capacity() * sizeof(VelocityConstraint) +
         m_previousImpulses.capacity() * sizeof(CachedImpulse);
}

void ContactSolver::solve(std::vector<RigidBody>& bodies,
                          const std::vector<RigidBodyContact>& contacts) {
  prepare(bodies, contacts);
  warmStart(bodies);

  for (int iteration = 0; iteration < m_settings.velocityIterations; ++iteration) {
    solveVelocityConstraints(bodies);
  }

  storeImpulses();

  correctPositions(bodies, contacts);
}

void ContactSolver::prepare(const std::vector<RigidBody>& bodies,
                            const std::vector<RigidBodyContact>& contacts) {
  m_constraints.clear();
  m_constraints.reserve(contacts.size());

  for (const RigidBodyContact& contact : contacts) {
    const RigidBody& body1 = bodies[contact.a];
    const RigidBody& body2 = bodies[contact.b];

    const float inverseMassSum = body1.invMass() + body2.invMass();
    if (inverseMassSum <= 0.f) {
      continue;
    }

    const Vec3 offset1 = contact.point - body1.position;
    const Vec3 offset2 = contact.point - body2.position;

    const float closingSpeed =
        (velocityAtOffset(body1, offset1) - velocityAtOffset(body2, offset2)).dot(contact.normal);
    float restitutionBias = 0.f;
    if (closingSpeed < -m_settings.restitutionThreshold) {
      restitutionBias = -contact.restitution * closingSpeed;
    }

    const Vec3 tangent1 = orthogonalAxis(contact.normal);
    const Vec3 tangent2 = contact.normal.cross(tangent1);
    const CachedImpulse* cached = m_settings.warmStarting ? findCachedImpulse(contact) : nullptr;

    if (cached && cached->normal.dot(contact.normal) < 0.95f)
      cached = nullptr;
    const float normalImpulse = cached ? cached->normalImpulse : 0.f;
    float tangentImpulse1 = cached ? cached->tangentImpulse.dot(tangent1) : 0.f;
    float tangentImpulse2 = cached ? cached->tangentImpulse.dot(tangent2) : 0.f;
    const float tangentLength = std::hypot(tangentImpulse1, tangentImpulse2);
    const float limit = contact.friction * normalImpulse;
    if (tangentLength > limit && tangentLength > 0.f) {
      tangentImpulse1 *= limit / tangentLength;
      tangentImpulse2 *= limit / tangentLength;
    }

    m_constraints.push_back({
        contact.a,
        contact.b,
        contact.normal,
        tangent1,
        tangent2,
        offset1,
        offset2,
        contact.friction,
        tangentImpulse1,
        tangentImpulse2,
        reciprocal(inverseEffectiveMass(body1, offset1, body2, offset2, contact.normal)),
        reciprocal(inverseEffectiveMass(body1, offset1, body2, offset2, tangent1)),
        reciprocal(inverseEffectiveMass(body1, offset1, body2, offset2, tangent2)),
        restitutionBias,
        normalImpulse,
    });
  }
}

void ContactSolver::warmStart(std::vector<RigidBody>& bodies) {
  for (const VelocityConstraint& constraint : m_constraints) {
    RigidBody& body1 = bodies[constraint.a];
    RigidBody& body2 = bodies[constraint.b];

    const Vec3 impulse = constraint.normal * constraint.normalImpulse +
                         constraint.tangent1 * constraint.tangentImpulse1 +
                         constraint.tangent2 * constraint.tangentImpulse2;

    applyImpulse(body1, constraint.offset1, body2, constraint.offset2, impulse);
  }
}

void ContactSolver::storeImpulses() {
  m_previousImpulses.clear();

  if (!m_settings.warmStarting) {
    return;
  }

  for (const VelocityConstraint& constraint : m_constraints) {
    m_previousImpulses.push_back({
        constraint.a,
        constraint.b,
        constraint.normalImpulse,
        constraint.normal,
        constraint.tangent1 * constraint.tangentImpulse1 +
            constraint.tangent2 * constraint.tangentImpulse2,
    });
  }

  const auto byBodyPair = [](const CachedImpulse& left, const CachedImpulse& right) {
    return left.a != right.a ? left.a < right.a : left.b < right.b;
  };

  if (!std::is_sorted(m_previousImpulses.begin(), m_previousImpulses.end(), byBodyPair)) {
    std::sort(m_previousImpulses.begin(), m_previousImpulses.end(), byBodyPair);
  }
}

const ContactSolver::CachedImpulse*
ContactSolver::findCachedImpulse(const RigidBodyContact& contact) const {
  const auto found =
      std::lower_bound(m_previousImpulses.begin(), m_previousImpulses.end(), contact,
                       [](const CachedImpulse& cached, const RigidBodyContact& target) {
                         return cached.a != target.a ? cached.a < target.a : cached.b < target.b;
                       });

  if (found == m_previousImpulses.end() || found->a != contact.a || found->b != contact.b) {
    return nullptr;
  }

  return &*found;
}

void ContactSolver::solveVelocityConstraints(std::vector<RigidBody>& bodies) {
  for (VelocityConstraint& constraint : m_constraints) {
    RigidBody& body1 = bodies[constraint.a];
    RigidBody& body2 = bodies[constraint.b];

    const float closingSpeed =
        (velocityAtOffset(body1, constraint.offset1) - velocityAtOffset(body2, constraint.offset2))
            .dot(constraint.normal);
    float impulse = (constraint.restitutionBias - closingSpeed) * constraint.normalMass;

    const float previousImpulse = constraint.normalImpulse;
    constraint.normalImpulse = std::max(previousImpulse + impulse, 0.f);
    impulse = constraint.normalImpulse - previousImpulse;

    applyImpulse(body1, constraint.offset1, body2, constraint.offset2, constraint.normal * impulse);

    const float maxFriction = constraint.friction * constraint.normalImpulse;
    applyFrictionAxis(body1, constraint.offset1, body2, constraint.offset2, constraint.tangent1,
                      constraint.tangentMass1, maxFriction, constraint.tangentImpulse1);
    applyFrictionAxis(body1, constraint.offset1, body2, constraint.offset2, constraint.tangent2,
                      constraint.tangentMass2, maxFriction, constraint.tangentImpulse2);
    const float tangentLength = std::hypot(constraint.tangentImpulse1, constraint.tangentImpulse2);
    if (tangentLength > maxFriction && tangentLength > 0.f) {
      const float scale = maxFriction / tangentLength;
      const Vec3 correction = constraint.tangent1 * (constraint.tangentImpulse1 * (scale - 1.f)) +
                              constraint.tangent2 * (constraint.tangentImpulse2 * (scale - 1.f));
      constraint.tangentImpulse1 *= scale;
      constraint.tangentImpulse2 *= scale;
      applyImpulse(body1, constraint.offset1, body2, constraint.offset2, correction);
    }
  }
}

void ContactSolver::correctPositions(std::vector<RigidBody>& bodies,
                                     const std::vector<RigidBodyContact>& contacts) {
  for (const RigidBodyContact& contact : contacts) {
    RigidBody& body1 = bodies[contact.a];
    RigidBody& body2 = bodies[contact.b];

    const float inverseMassSum = body1.invMass() + body2.invMass();
    if (inverseMassSum <= 0.f) {
      continue;
    }

    const float excess = contact.penetration - m_settings.penetrationSlop;
    if (excess <= 0.f) {
      continue;
    }

    const float distance =
        std::min(m_settings.positionCorrectionRate * excess, m_settings.maxPositionCorrection);
    const Vec3 correction = contact.normal * (distance / inverseMassSum);

    body1.position += correction * body1.invMass();
    body2.position -= correction * body2.invMass();
  }
}
