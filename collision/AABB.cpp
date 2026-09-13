#include "collision/AABB.hpp"

bool AABB::overlaps(const AABB& other) const {
  return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y &&
         max.y >= other.min.y && min.z <= other.max.z && max.z >= other.min.z;
}

AABB makeAABBForSphere(const Vec3& center, float radius) {
  const Vec3 extent(radius, radius, radius);
  return {center - extent, center + extent};
}
