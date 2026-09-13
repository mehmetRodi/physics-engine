#include "math/Mat3.hpp"

Mat3 Mat3::identity() {
  return diagonal(Vec3(1.0f, 1.0f, 1.0f));
}

Mat3 Mat3::diagonal(const Vec3& d) {
  Mat3 result{};
  result.m[0][0] = d.x;
  result.m[1][1] = d.y;
  result.m[2][2] = d.z;
  return result;
}

Vec3 Mat3::operator*(const Vec3& v) const {
  return Vec3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
              m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
              m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
}

Mat3 Mat3::operator*(const Mat3& other) const {
  Mat3 result{};

  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.m[row][column] = m[row][0] * other.m[0][column] + m[row][1] * other.m[1][column] +
                              m[row][2] * other.m[2][column];
    }
  }

  return result;
}

Mat3 Mat3::transposed() const {
  Mat3 result{};

  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.m[row][column] = m[column][row];
    }
  }

  return result;
}
