#include "math/Quat.hpp"

#include <cmath>

Quat::Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

Quat::Quat(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quat Quat::fromAxisAngle(const Vec3& axis, float radians) {
  const Vec3 unitAxis = axis.normalized();
  const float half = 0.5f * radians;
  const float sine = std::sin(half);

  return Quat(std::cos(half), unitAxis.x * sine, unitAxis.y * sine, unitAxis.z * sine);
}

Quat Quat::operator*(const Quat& other) const {
  return Quat(w * other.w - x * other.x - y * other.y - z * other.z,
              w * other.x + x * other.w + y * other.z - z * other.y,
              w * other.y - x * other.z + y * other.w + z * other.x,
              w * other.z + x * other.y - y * other.x + z * other.w);
}

Quat Quat::operator+(const Quat& other) const {
  return Quat(w + other.w, x + other.x, y + other.y, z + other.z);
}

Quat Quat::operator*(float scalar) const {
  return Quat(w * scalar, x * scalar, y * scalar, z * scalar);
}

float Quat::lengthSq() const {
  return w * w + x * x + y * y + z * z;
}

Quat Quat::normalized() const {
  const float squared = lengthSq();

  if (squared < 1e-12f) {
    return Quat();
  }

  return *this * (1.0f / std::sqrt(squared));
}

Mat3 Quat::toMat3() const {
  const float xx = x * x;
  const float yy = y * y;
  const float zz = z * z;
  const float xy = x * y;
  const float xz = x * z;
  const float yz = y * z;
  const float wx = w * x;
  const float wy = w * y;
  const float wz = w * z;

  Mat3 result{};

  result.m[0][0] = 1.0f - 2.0f * (yy + zz);
  result.m[0][1] = 2.0f * (xy - wz);
  result.m[0][2] = 2.0f * (xz + wy);

  result.m[1][0] = 2.0f * (xy + wz);
  result.m[1][1] = 1.0f - 2.0f * (xx + zz);
  result.m[1][2] = 2.0f * (yz - wx);

  result.m[2][0] = 2.0f * (xz - wy);
  result.m[2][1] = 2.0f * (yz + wx);
  result.m[2][2] = 1.0f - 2.0f * (xx + yy);

  return result;
}

Vec3 Quat::rotate(const Vec3& v) const {

  const Vec3 axis(x, y, z);
  const Vec3 crossed = axis.cross(v);

  return v + crossed * (2.0f * w) + axis.cross(crossed) * 2.0f;
}

Quat Quat::integrated(const Vec3& angularVelocity, float dt) const {
  const Quat spin(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);
  return (*this + spin * *this * (0.5f * dt)).normalized();
}
