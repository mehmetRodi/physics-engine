#pragma once

#include "math/Mat3.hpp"
#include "math/Vec3.hpp"

struct Quat {
  float w, x, y, z;

  Quat();
  Quat(float w, float x, float y, float z);

  static Quat fromAxisAngle(const Vec3& axis, float radians);

  Quat operator*(const Quat& other) const;
  Quat operator+(const Quat& other) const;
  Quat operator*(float scalar) const;

  float lengthSq() const;
  Quat normalized() const;
  Mat3 toMat3() const;
  Vec3 rotate(const Vec3& v) const;

  Quat integrated(const Vec3& angularVelocity, float dt) const;
};
