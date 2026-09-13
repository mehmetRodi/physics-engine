#include "bench/Benchmark.hpp"
#include "physics/RigidBodySystem.hpp"

class RigidBodySystemInstrumentationAccess {
public:
  static void integrate(RigidBodySystem& s, float dt, const Vec3& g) {
    s.integrate(dt, g);
  }
  static void proxies(RigidBodySystem& s) {
    s.buildAABBProxies();
  }
  static void broadphase(RigidBodySystem& s) {
    findAABBPairsSweepAndPrune(s.m_aabbProxies, s.m_aabbSweepScratch, s.m_aabbPairs);
  }
  static void contacts(RigidBodySystem& s) {
    s.buildContacts();
  }
  static void solve(RigidBodySystem& s) {
    s.m_contactSolver.solve(s.m_bodies, s.m_contacts);
  }
};
int main(int argc, char** argv) {
  try {
    const auto o = bench::options(argc, argv);
    const World initial = bench::scene(o);
    RigidBodySystem system;
    system.reserveRigidBodies(o.bodies);
    system.reserveCollisionCapacity(o.bodies * 8, o.bodies * 4);
    system.setWorkerThreadCount(o.threads);
    system.setContactSolverSettings(initial.contactSolverSettings());
    for (std::size_t i = 0; i < initial.bodyCount(); ++i) {
      const auto& b = initial.rigidBody(i);
      auto id = system.createRigidBody(b.mass(), b.shape().sphereRadius);
      system.rigidBody(id) = b;
    }
    const Vec3 gravity = o.scene == "columns" ? Vec3(0, -9.8f, 0) : Vec3();
    for (int i = 0; i < o.warmup; ++i)
      system.step(1.f / 60, gravity);
    struct Sample {
      std::uint64_t integrate, proxies, broadphase, contacts, solve;
      std::size_t pairs, count;
    };
    std::vector<Sample> samples;
    samples.reserve(o.samples);
    using A = RigidBodySystemInstrumentationAccess;
    for (int i = 0; i < o.samples; ++i) {
      const auto t0 = bench::Clock::now();
      A::integrate(system, 1.f / 60, gravity);
      const auto t1 = bench::Clock::now();
      A::proxies(system);
      const auto t2 = bench::Clock::now();
      A::broadphase(system);
      const auto t3 = bench::Clock::now();
      A::contacts(system);
      const auto t4 = bench::Clock::now();
      A::solve(system);
      const auto t5 = bench::Clock::now();
      const auto stats = system.collisionPipelineStats();
      samples.push_back({bench::elapsed(t0, t1), bench::elapsed(t1, t2), bench::elapsed(t2, t3),
                         bench::elapsed(t3, t4), bench::elapsed(t4, t5),
                         stats.aabbCandidatePairCount, stats.contactCount});
    }
    std::cout
        << "sample,integrate_ns,proxies_ns,broadphase_ns,contacts_ns,solve_ns,pairs,contacts\n";
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const auto& s = samples[i];
      std::cout << i << ',' << s.integrate << ',' << s.proxies << ',' << s.broadphase << ','
                << s.contacts << ',' << s.solve << ',' << s.pairs << ',' << s.count << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
