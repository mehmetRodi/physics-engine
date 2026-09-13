#include "physics/StateSnapshot.hpp"
#include "physics/World.hpp"
#include <charconv>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {
World scene(unsigned threads) {
  World world(Vec3(0, -9.8f, 0));
  world.reserveRigidBodies(384);
  world.reserveCollisionCapacity(4096, 4096);
  world.setWorkerThreadCount(threads);
  for (int column = 0; column < 32; ++column) {
    for (int level = 0; level < 12; ++level) {
      auto id = world.createRigidBody(level ? 1.f : 0.f, .5f);
      auto& b = world.rigidBody(id);
      b.position = Vec3(column * 2.f, level * .99f, 0);
      b.material.restitution = 0.f;
    }
  }
  return world;
}
unsigned number(std::string_view s) {
  unsigned result = 0;
  auto parsed = std::from_chars(s.data(), s.data() + s.size(), result);
  if (parsed.ec != std::errc() || parsed.ptr != s.data() + s.size() || result == 0)
    throw std::invalid_argument("expected positive integer");
  return result;
}
} // namespace
int main(int argc, char** argv) {
  try {
    unsigned threads = 4, steps = 240;
    for (int i = 1; i < argc; ++i) {
      const std::string_view arg = argv[i];
      if (arg == "--help") {
        std::cout << "headless_replay [--threads N] [--steps N]\n";
        return 0;
      }
      if ((arg != "--threads" && arg != "--steps") || i + 1 >= argc)
        throw std::invalid_argument("usage: headless_replay [--threads N] [--steps N]");
      const unsigned value = number(argv[++i]);
      if (arg == "--threads") {
        if (value > 256)
          throw std::invalid_argument("maximum 256 threads");
        threads = value;
      } else
        steps = value;
    }
    World serial = scene(1), parallel = scene(threads);
    std::uint64_t digest = 14695981039346656037ull;
    for (unsigned tick = 0; tick < steps; ++tick) {
      serial.step(1.f / 60);
      parallel.step(1.f / 60);
      for (std::size_t body = 0; body < serial.bodyCount(); ++body) {
        const auto expected = snapshotBody(serial.rigidBody(body));
        const auto actual = snapshotBody(parallel.rigidBody(body));
        for (std::size_t field = 0; field < expected.size(); ++field) {
          if (actual[field] != expected[field]) {
            std::cerr << "divergence: tick=" << tick << " body=" << body << " field=" << field
                      << '\n';
            return 1;
          }

          for (unsigned shift = 0; shift < 32; shift += 8) {
            digest ^= (expected[field] >> shift) & 0xffu;
            digest *= 1099511628211ull;
          }
        }
      }
    }
    std::cout << "verified_ticks=" << steps << " bodies=" << serial.bodyCount()
              << " threads=" << threads << "\ntrajectory_fnv1a64=" << std::hex << std::setw(16)
              << std::setfill('0') << digest << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
