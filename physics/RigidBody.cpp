#include "physics/RigidBody.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
} // namespace

RigidBody::RigidBody(float mass, float radius) {
  setMassProperties(mass, radius);
}

void RigidBody::setMassProperties(float mass, float radius) {
  if (!std::isfinite(mass) || mass < 0.f || !std::isfinite(radius) || radius <= 0.f)
    throw std::invalid_argument("mass must be finite and nonnegative; radius finite and positive");
  const double inverseMass = mass > 0.f ? 1.0 / mass : 0.0;
  const double inverseInertia = mass > 0.f ? 2.5 / (double(mass) * radius * radius) : 0.0;
  const double largest = std::numeric_limits<float>::max();
  const double smallest = std::numeric_limits<float>::min();
  if (mass > 0.f && (inverseMass > largest || inverseMass < smallest || inverseInertia > largest ||
                     inverseInertia < smallest))
    throw std::invalid_argument("mass/radius produce unrepresentable inverse mass or inertia");

  m_mass = mass;
  m_invMass = static_cast<float>(inverseMass);
  const float k = static_cast<float>(inverseInertia);
  m_invInertia = Mat3::diagonal(Vec3(k, k, k));
  m_shape = {ShapeType::Sphere, radius};
  acceleration = Vec3();
  torque = Vec3();
  if (isStatic()) {
    velocity = Vec3();
    angularVelocity = Vec3();
  }
}

void RigidBody::setMass(float mass) {
  setMassProperties(mass, m_shape.sphereRadius);
}
void RigidBody::setSphereRadius(float radius) {
  setMassProperties(m_mass, radius);
}

void RigidBody::validate() const {
  const float qLength = orientation.lengthSq();
  if (!finite(position) || !finite(velocity) || !finite(acceleration) || !finite(angularVelocity) ||
      !finite(torque) || !std::isfinite(qLength) || qLength < 1e-12f ||
      !std::isfinite(material.restitution) || material.restitution < 0.f ||
      material.restitution > 1.f || !std::isfinite(material.friction) || material.friction < 0.f ||
      !std::isfinite(material.linearDamping) || material.linearDamping < 0.f ||
      !std::isfinite(material.angularDamping) || material.angularDamping < 0.f)
    throw std::invalid_argument("invalid rigid-body state or material");
}

void RigidBody::applyForce(const Vec3& force) {
  if (!finite(force))
    throw std::invalid_argument("force must be finite");
  if (isStatic())
    return;
  const Vec3 next = acceleration + force * m_invMass;
  if (!finite(next))
    throw std::overflow_error("force accumulation overflow");
  acceleration = next;
}

void RigidBody::applyForceAtPoint(const Vec3& force, const Vec3& worldPoint) {
  if (!finite(worldPoint))
    throw std::invalid_argument("force point must be finite");
  applyForce(force);
  applyTorque((worldPoint - position).cross(force));
}

void RigidBody::applyTorque(const Vec3& value) {
  if (!finite(value))
    throw std::invalid_argument("torque must be finite");
  if (isStatic())
    return;
  const Vec3 next = torque + value;
  if (!finite(next))
    throw std::overflow_error("torque accumulation overflow");
  torque = next;
}

void RigidBody::update(float dt) {
  if (!std::isfinite(dt) || dt < 0.f)
    throw std::invalid_argument("dt must be finite and nonnegative");
  validate();
  if (isStatic()) {
    velocity = Vec3();
    angularVelocity = Vec3();
    acceleration = Vec3();
    torque = Vec3();
    return;
  }

  velocity =
      (velocity + acceleration * dt) * std::clamp(1.f - material.linearDamping * dt, 0.f, 1.f);
  position += velocity * dt;
  angularVelocity = (angularVelocity + (m_invInertia * torque) * dt) *
                    std::clamp(1.f - material.angularDamping * dt, 0.f, 1.f);
  orientation = orientation.integrated(angularVelocity, dt);
  acceleration = Vec3();
  torque = Vec3();
}
