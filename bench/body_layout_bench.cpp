#include "math/Mat3.hpp"
#include "math/Quat.hpp"
#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

constexpr float kDt = 1.0f / 60.0f;

struct SoABodies {
  std::vector<Vec3> positions;
  std::vector<Vec3> velocities;
  std::vector<Vec3> accelerations;
  std::vector<Quat> orientations;
  std::vector<Vec3> angularVelocities;
  std::vector<Vec3> torques;
  std::vector<Mat3> invInertiaLocal;
  std::vector<Mat3> invInertiaWorld;
  std::vector<float> radii;
  std::vector<float> invMasses;
  std::vector<float> linearDamping;
  std::vector<float> angularDamping;

  void reserve(std::size_t capacity) {
    positions.reserve(capacity);
    velocities.reserve(capacity);
    accelerations.reserve(capacity);
    orientations.reserve(capacity);
    angularVelocities.reserve(capacity);
    torques.reserve(capacity);
    invInertiaLocal.reserve(capacity);
    invInertiaWorld.reserve(capacity);
    radii.reserve(capacity);
    invMasses.reserve(capacity);
    linearDamping.reserve(capacity);
    angularDamping.reserve(capacity);
  }

  void push(const RigidBody& body) {
    positions.push_back(body.position);
    velocities.push_back(body.velocity);
    accelerations.push_back(body.acceleration);
    orientations.push_back(body.orientation);
    angularVelocities.push_back(body.angularVelocity);
    torques.push_back(body.torque);
    invInertiaLocal.push_back(body.invInertiaLocal());
    invInertiaWorld.push_back(body.invInertiaWorld());
    radii.push_back(body.shape().sphereRadius);
    invMasses.push_back(body.invMass());
    linearDamping.push_back(body.material.linearDamping);
    angularDamping.push_back(body.material.angularDamping);
  }

  std::size_t size() const {
    return positions.size();
  }
};

std::vector<RigidBody> createBodies(std::size_t bodyCount) {
  std::vector<RigidBody> bodies;
  bodies.reserve(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    bodies.emplace_back(1.0f + static_cast<float>(i % 7) * 0.25f,
                        0.25f + static_cast<float>(i % 5) * 0.1f);
    RigidBody& body = bodies.back();
    body.position = Vec3(static_cast<float>(i % 64), static_cast<float>(i / 64), 0.f);
    body.velocity = Vec3(0.1f, float(int(i % 17) - 8) * .2f, 0.05f);
    body.angularVelocity = Vec3(0.3f, 0.1f, -0.2f);
  }

  return bodies;
}

void integrateAoS(std::vector<RigidBody>& bodies) {
  for (RigidBody& body : bodies) {
    body.velocity += body.acceleration * kDt;
    body.velocity =
        body.velocity * std::clamp(1.0f - body.material.linearDamping * kDt, 0.0f, 1.0f);
    body.position += body.velocity * kDt;

    body.angularVelocity += (body.invInertiaWorld() * body.torque) * kDt;
    body.angularVelocity =
        body.angularVelocity * std::clamp(1.0f - body.material.angularDamping * kDt, 0.0f, 1.0f);
    body.orientation = body.orientation.integrated(body.angularVelocity, kDt);

    body.acceleration = Vec3(0.0f, 0.0f, 0.0f);
    body.torque = Vec3(0.0f, 0.0f, 0.0f);
  }
}

void integrateSoA(SoABodies& bodies) {
  const std::size_t count = bodies.size();

  for (std::size_t i = 0; i < count; ++i) {
    bodies.velocities[i] += bodies.accelerations[i] * kDt;
    bodies.velocities[i] =
        bodies.velocities[i] * std::clamp(1.0f - bodies.linearDamping[i] * kDt, 0.0f, 1.0f);
    bodies.positions[i] += bodies.velocities[i] * kDt;

    bodies.angularVelocities[i] += (bodies.invInertiaWorld[i] * bodies.torques[i]) * kDt;
    bodies.angularVelocities[i] =
        bodies.angularVelocities[i] * std::clamp(1.0f - bodies.angularDamping[i] * kDt, 0.0f, 1.0f);
    bodies.orientations[i] = bodies.orientations[i].integrated(bodies.angularVelocities[i], kDt);

    bodies.accelerations[i] = Vec3(0.0f, 0.0f, 0.0f);
    bodies.torques[i] = Vec3(0.0f, 0.0f, 0.0f);
  }
}

float proxyChecksumAoS(const std::vector<RigidBody>& bodies) {
  float checksum = 0.f;

  for (const RigidBody& body : bodies) {
    checksum += body.position.x + body.position.y + body.position.z + body.shape().sphereRadius;
  }

  return checksum;
}

float proxyChecksumSoA(const SoABodies& bodies) {
  float checksum = 0.f;
  const std::size_t count = bodies.size();

  for (std::size_t i = 0; i < count; ++i) {
    checksum +=
        bodies.positions[i].x + bodies.positions[i].y + bodies.positions[i].z + bodies.radii[i];
  }

  return checksum;
}

struct ContactPair {
  std::size_t a;
  std::size_t b;
  Vec3 normal;
};

std::vector<ContactPair> createContacts(std::size_t bodyCount) {
  std::vector<ContactPair> contacts;
  contacts.reserve(bodyCount);

  for (std::size_t i = 0; i < bodyCount; ++i) {
    const std::size_t other = (i * 7919 + 13) % bodyCount;
    if (other == i) {
      continue;
    }

    contacts.push_back({i, other, Vec3(0.f, 1.f, 0.f)});
  }

  return contacts;
}

void solveScatterAoS(std::vector<RigidBody>& bodies, const std::vector<ContactPair>& contacts) {
  for (const ContactPair& contact : contacts) {
    RigidBody& body1 = bodies[contact.a];
    RigidBody& body2 = bodies[contact.b];

    body1.velocity.y += .01f;
    const float closing = (body1.velocity - body2.velocity).dot(contact.normal);
    const float impulse = -closing / (body1.invMass() + body2.invMass());

    body1.velocity += contact.normal * (impulse * body1.invMass());
    body1.angularVelocity += body1.invInertiaWorld() * contact.normal * impulse;
    body2.velocity -= contact.normal * (impulse * body2.invMass());
    body2.angularVelocity -= body2.invInertiaWorld() * contact.normal * impulse;
  }
}

void solveScatterSoA(SoABodies& bodies, const std::vector<ContactPair>& contacts) {
  for (const ContactPair& contact : contacts) {
    const std::size_t a = contact.a;
    const std::size_t b = contact.b;

    bodies.velocities[a].y += .01f;
    const float closing = (bodies.velocities[a] - bodies.velocities[b]).dot(contact.normal);
    const float impulse = -closing / (bodies.invMasses[a] + bodies.invMasses[b]);

    bodies.velocities[a] += contact.normal * (impulse * bodies.invMasses[a]);
    bodies.angularVelocities[a] += bodies.invInertiaWorld[a] * contact.normal * impulse;
    bodies.velocities[b] -= contact.normal * (impulse * bodies.invMasses[b]);
    bodies.angularVelocities[b] -= bodies.invInertiaWorld[b] * contact.normal * impulse;
  }
}

template <typename Fn> double measureNanoseconds(int warmup, int iterations, Fn&& body) {
  for (int i = 0; i < warmup; ++i) {
    body();
  }

  std::vector<std::uint64_t> samples;
  samples.reserve(static_cast<std::size_t>(iterations));

  for (int i = 0; i < iterations; ++i) {
    const auto start = Clock::now();
    body();
    const auto end = Clock::now();
    samples.push_back(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()));
  }

  std::sort(samples.begin(), samples.end());
  return static_cast<double>(samples[samples.size() / 2]);
}

void runCase(std::size_t bodyCount) {
  std::vector<RigidBody> aos = createBodies(bodyCount);

  SoABodies soa;
  soa.reserve(bodyCount);
  for (const RigidBody& body : aos) {
    soa.push(body);
  }

  constexpr int warmup = 50;
  constexpr int iterations = 500;

  const double integrateAoSNs = measureNanoseconds(warmup, iterations, [&] { integrateAoS(aos); });
  const double integrateSoANs = measureNanoseconds(warmup, iterations, [&] { integrateSoA(soa); });

  const std::vector<ContactPair> contacts = createContacts(bodyCount);

  const double solveAoSNs =
      measureNanoseconds(warmup, iterations, [&] { solveScatterAoS(aos, contacts); });
  const double solveSoANs =
      measureNanoseconds(warmup, iterations, [&] { solveScatterSoA(soa, contacts); });

  float sink = 0.f;
  const double proxyAoSNs =
      measureNanoseconds(warmup, iterations, [&] { sink += proxyChecksumAoS(aos); });
  const double proxySoANs =
      measureNanoseconds(warmup, iterations, [&] { sink += proxyChecksumSoA(soa); });

  double stateChecksum = 0.0;
  for (std::size_t i = 0; i < bodyCount; ++i) {
    const float actual[] = {
        aos[i].position.x,        aos[i].position.y,        aos[i].position.z,
        aos[i].velocity.x,        aos[i].velocity.y,        aos[i].velocity.z,
        aos[i].angularVelocity.x, aos[i].angularVelocity.y, aos[i].angularVelocity.z,
        aos[i].orientation.w,     aos[i].orientation.x,     aos[i].orientation.y,
        aos[i].orientation.z};
    const float expected[] = {
        soa.positions[i].x,         soa.positions[i].y,         soa.positions[i].z,
        soa.velocities[i].x,        soa.velocities[i].y,        soa.velocities[i].z,
        soa.angularVelocities[i].x, soa.angularVelocities[i].y, soa.angularVelocities[i].z,
        soa.orientations[i].w,      soa.orientations[i].x,      soa.orientations[i].y,
        soa.orientations[i].z};
    for (std::size_t j = 0; j < 13; ++j) {
      if (!std::isfinite(actual[j]) || actual[j] != expected[j])
        throw std::runtime_error("AoS/SoA kernel output mismatch");
      stateChecksum += actual[j];
    }
  }
  std::cout << "verified_state_checksum: " << stateChecksum << '\n';
  const double perBody = static_cast<double>(bodyCount);

  std::cout << "body layout benchmark\n";
  std::cout << "body_count: " << bodyCount << '\n';
  std::cout << "aos_bytes_per_body: " << sizeof(RigidBody) << '\n';
  std::cout << "integrate_aos_ns_per_body: " << integrateAoSNs / perBody << '\n';
  std::cout << "integrate_soa_ns_per_body: " << integrateSoANs / perBody << '\n';
  std::cout << "proxy_scan_aos_ns_per_body: " << proxyAoSNs / perBody << '\n';
  std::cout << "proxy_scan_soa_ns_per_body: " << proxySoANs / perBody << '\n';
  std::cout << "solve_scatter_aos_ns_per_contact: "
            << solveAoSNs / static_cast<double>(contacts.size()) << '\n';
  std::cout << "solve_scatter_soa_ns_per_contact: "
            << solveSoANs / static_cast<double>(contacts.size()) << '\n';
  std::cout << "checksum_sink: " << sink << '\n';
  std::cout << '\n';
}
} // namespace

int main() {
  for (const std::size_t bodyCount : {std::size_t{256}, std::size_t{4096}, std::size_t{65536}}) {
    runCase(bodyCount);
  }

  return 0;
}
