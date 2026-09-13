#pragma once

#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
#include "physics/RigidBodyContact.hpp"

#include <cstddef>
#include <vector>

struct ContactSolverSettings {

  int velocityIterations = 8;

  float restitutionThreshold = 0.5f;

  float positionCorrectionRate = 0.4f;

  float penetrationSlop = 0.005f;

  float maxPositionCorrection = 0.2f;

  bool warmStarting = true;
};

class ContactSolver {
public:
  const ContactSolverSettings& settings() const;
  void setSettings(const ContactSolverSettings& settings);

  void reserveContacts(std::size_t capacity);
  std::size_t reservedStorageBytes() const;

  void solve(std::vector<RigidBody>& bodies, const std::vector<RigidBodyContact>& contacts);

private:
  struct VelocityConstraint {
    std::size_t a;
    std::size_t b;
    Vec3 normal;

    Vec3 tangent1;
    Vec3 tangent2;

    Vec3 offset1;
    Vec3 offset2;
    float friction;
    float tangentImpulse1;
    float tangentImpulse2;

    float normalMass;
    float tangentMass1;
    float tangentMass2;

    float restitutionBias;

    float normalImpulse;
  };

  struct CachedImpulse {
    std::size_t a;
    std::size_t b;
    float normalImpulse;
    Vec3 normal;
    Vec3 tangentImpulse;
  };

  void prepare(const std::vector<RigidBody>& bodies, const std::vector<RigidBodyContact>& contacts);
  void warmStart(std::vector<RigidBody>& bodies);
  void storeImpulses();
  const CachedImpulse* findCachedImpulse(const RigidBodyContact& contact) const;
  void solveVelocityConstraints(std::vector<RigidBody>& bodies);
  void correctPositions(std::vector<RigidBody>& bodies,
                        const std::vector<RigidBodyContact>& contacts);

  ContactSolverSettings m_settings;
  std::vector<VelocityConstraint> m_constraints;
  std::vector<CachedImpulse> m_previousImpulses;
};
