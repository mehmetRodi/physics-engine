#pragma once

#include "math/Vec3.hpp"

struct Mat3 {
  float m[3][3];

  static Mat3 identity();
  static Mat3 diagonal(const Vec3& d);

  Vec3 operator*(const Vec3& v) const;
  Mat3 operator*(const Mat3& other) const;
  Mat3 transposed() const;
};
