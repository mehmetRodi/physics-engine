#include "physics/World.hpp"
#include <iostream>
int main() {
  std::cout << "bodies,body_storage_bytes,with_collision_hints_bytes,candidate_hint,contact_hint\n";
  for (std::size_t n : {1024u, 2048u, 8192u}) {
    World world{Vec3()};
    world.reserveRigidBodies(n);
    const auto bodies = world.reservedStorageBytes();
    world.reserveCollisionCapacity(n * 8, n * 4);
    std::cout << n << ',' << bodies << ',' << world.reservedStorageBytes() << ',' << n * 8 << ','
              << n * 4 << '\n';
  }
}
