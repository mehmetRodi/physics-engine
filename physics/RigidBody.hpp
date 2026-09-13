#pragma once
#include "math/Mat3.hpp"
#include "math/Quat.hpp"
#include "math/Vec3.hpp"

struct BodyMaterial {
  float restitution = 1.0f;
  float linearDamping = 0.0f;
  float angularDamping = 0.0f;

  float friction = 0.2f;
};

enum class ShapeType {
  Sphere,
};

struct RigidBodyShape {
  ShapeType type = ShapeType::Sphere;
  float sphereRadius = 0.0f;
};

struct RigidBody {
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  Quat orientation;
  Vec3 angularVelocity;
  Vec3 torque;
  BodyMaterial material;

  RigidBody(float mass, float radius);

  float mass() const {
    return m_mass;
  }
  float invMass() const {
    return m_invMass;
  }
  const RigidBodyShape& shape() const {
    return m_shape;
  }
  const Mat3& invInertiaLocal() const {
    return m_invInertia;
  }

  const Mat3& invInertiaWorld() const {
    return m_invInertia;
  }
  void setMass(float mass);
  void setSphereRadius(float radius);
  bool isStatic() const {
    return m_invMass == 0.f;
  }

  void validate() const;

  void applyForce(const Vec3& force);

  void applyForceAtPoint(const Vec3& force, const Vec3& worldPoint);
  void applyTorque(const Vec3& value);

  void update(float dt);

private:
  void setMassProperties(float mass, float radius);
  float m_mass = 0.f;
  float m_invMass = 0.f;
  Mat3 m_invInertia{};
  RigidBodyShape m_shape;
};
