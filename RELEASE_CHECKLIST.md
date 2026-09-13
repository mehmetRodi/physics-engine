# Publication checklist — 2026-09-13

The audit's implementation and local packaging work is complete. The user explicitly
authorized autonomous implementation for this release, overriding the earlier mentoring
workflow. The original [audit](PORTFOLIO_AUDIT.md) remains a historical snapshot.

| Work | Resolution and evidence | Status |
|---|---|---|
| Coincident contacts | Ordered-ID fallback; finite/separation and serial/parallel per-tick regression tests | Complete |
| Body state contracts | Protected mass/radius/inertia, validated setters, static semantics, invalid-input tests; documented reference lifetimes | Complete |
| Collision memory | Independent body/candidate/contact hints; checked capacity arithmetic; grow without dropping contacts; memory.csv | Complete |
| Allocation coverage | Ordinary/aligned new tracking with atomic counters; first/steady ticks and changing contacts on 1/2/4/8 threads | Complete |
| Replay | Explicit 25-field bit snapshots each tick, signed-zero test, CLI first-divergence reporting and trajectory digest | Complete |
| Job-system lifecycle | Partial-construction join, callback exception propagation, rejection of recursive/concurrent submission, reuse tests | Complete |
| Tangent cache | World-space impulse reprojection, incompatible-normal invalidation, basis-switch test, Coulomb-disk cap test | Complete |
| Benchmark validity | Dynamic stages/full ticks, per-sample workload counts/quality, verified synthetic AoS/SoA outputs | Complete |
| Performance evidence | Five process repetitions, raw CSV, exact environment/source hashes, two figures, separate sampled profile | Complete |
| Build/tooling | Headless default, explicit demo CI build, CMake 3.24.3 verified, formatter 18.1.8 pinned | Complete |
| Presentation | README, API/design contracts, contributor commands, actual SFML preview GIF and capture tooling | Complete |
| Local release checks | macOS Debug/Release/sanitizers; Linux GCC Release, demo and benchmark compilation; formatting | Complete |
| GitHub workflow execution | Requires choosing/configuring the destination remote and pushing | Pending publication |

## Verification record

- macOS 27.0 arm64, Apple Clang 21.0.0: final Debug and Release suites pass 149 checks.
- AddressSanitizer/UndefinedBehaviorSanitizer: 147 checks (two allocator interception
  tests are excluded by design). ThreadSanitizer: 149 checks.
- Ubuntu 24.04 aarch64 container, GCC 13.3: Release with warnings as errors, SFML 2.6.1,
  all eleven benchmark targets, and 148 checks passed. This ran before the final extra
  coincident-policy parallel regression; that additional test passed on macOS.
- The layout executable completed its numerical equivalence checks on macOS and Linux.
- CMake 3.24.3 completed a fresh dependency download/configure earlier in this release;
  the final builds used CMake 4.4.3 locally and the distribution CMake in the Linux container.
- All project C++ files pass clang-format 18.1.8. Python tooling compiles and the report,
  preview encoder, and macOS profiler scripts ran successfully.
- The optional demo compiled and captured its own rendered frames. The preview was
  visually inspected. Linux execution was headless; Linux GUI interaction was not tested.

Raw local test logs live under `/tmp/physics-final-*-tests.log` and
`/tmp/physics-linux-validation.log`; these are temporary. Durable performance data and
methodology are in [docs/performance](docs/performance/README.md). Future hosted CI
results should be attached to the actual publication commit rather than inferred from
these local checks. Windows, 32-bit ARM, and cross-platform bitwise replay are not claimed.
