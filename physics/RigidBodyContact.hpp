#pragma once

#include "math/Vec3.hpp"

#include <cstddef>

struct RigidBodyContact {
  std::size_t a;
  std::size_t b;
  Vec3 normal;

  Vec3 point;
  float penetration;
  float restitution;
  float friction;
};
