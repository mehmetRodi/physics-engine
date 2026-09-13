#pragma once
#include "physics/RigidBody.hpp"
#include <array>
#include <bit>
#include <cstdint>

using BodyStateWords = std::array<std::uint32_t, 25>;
inline BodyStateWords snapshotBody(const RigidBody& b) {
  const std::array<float, 25> values = {b.position.x,
                                        b.position.y,
                                        b.position.z,
                                        b.velocity.x,
                                        b.velocity.y,
                                        b.velocity.z,
                                        b.angularVelocity.x,
                                        b.angularVelocity.y,
                                        b.angularVelocity.z,
                                        b.orientation.w,
                                        b.orientation.x,
                                        b.orientation.y,
                                        b.orientation.z,
                                        b.acceleration.x,
                                        b.acceleration.y,
                                        b.acceleration.z,
                                        b.torque.x,
                                        b.torque.y,
                                        b.torque.z,
                                        b.mass(),
                                        b.shape().sphereRadius,
                                        b.material.restitution,
                                        b.material.friction,
                                        b.material.linearDamping,
                                        b.material.angularDamping};
  BodyStateWords result{};
  for (std::size_t i = 0; i < values.size(); ++i)
    result[i] = std::bit_cast<std::uint32_t>(values[i]);
  return result;
}
