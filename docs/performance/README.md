# Performance study — 2026-09-13

This study tests three questions: whether warm starting offers a useful quality/cost
tradeoff, whether the parallel stages accelerate a contact-heavy tick, and how much
of a broadphase improvement comes from packing versus SIMD. It also quantifies the
new memory reservation contract. These results describe one machine and build.

## Method and reproducibility

Apple M4, 10 logical CPUs, arm64 macOS 27.0; Apple Clang 21.0.0; CMake 4.4.3.
Release flags: `-O3 -DNDEBUG -g`, C++20, `-ffp-contract=off`, no LTO. Detailed metadata,
commands, and SHA-256 hashes of measured source inputs are in [environment.json](environment.json).
The UTC capture date is 2026-09-12; local time was 2026-09-13 in Europe/Istanbul.

Each world case has five independent process runs, 300 warmup ticks, and 600 timed
ticks at dt=1/60. World case order is reversed on alternating repetitions. Timings
use `steady_clock`; printing and quality/statistics collection happen outside the
timed step. Builds, sanitizer tests, preview rendering, and the profiler were not
running during this collection. Ordinary host/OS activity was not controlled.

No affinity, frequency locking, or exclusive machine reservation was used. This is a
heterogeneous laptop CPU. Charts report the median of five process medians, with
min/max run-median whiskers; these are variability ranges, not confidence intervals.
Raw samples remain chronological. Reported p99 values in [summary.csv](summary.csv)
are medians of per-run empirical p99s; 600 samples per run do not establish rare-event
tail guarantees, and no p99.9 claim is made.

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DBUILD_BENCHMARKS=ON \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG -g' -DPHYSICS_WARNINGS_AS_ERRORS=ON
cmake --build build-release --parallel
python3 -m venv .cache/report
.cache/report/bin/python -m pip install -r requirements-report.txt
.cache/report/bin/python scripts/benchmark_report.py --build build-release --repeats 5
./build-release/memory_budget_bench > docs/performance/memory.csv
```

The default script output directory is `docs/performance`; use `--output PATH` to
retain this baseline while collecting another. Kernel output equivalence is checked
by the test suite and the layout benchmark before interpreting timing differences.

## Warm starting: quality and cost change together

The scene contains 32 independent columns, each with ten dynamic radius-0.5 spheres
and one static pedestal: 352 bodies. The timer surrounds `World::step`, not only the
solver. Compression is the largest drop of a column's top body from its built height.

| Settings | Median tick, µs | Run-median range, µs | Final contacts | Final compression, m |
|---|---:|---:|---:|---:|
| Warm, 1 iteration | 89.6 | 88.8–94.0 | 320 | 0.701 |
| Warm, 8 iterations | 373.8 | 354.2–374.8 | 320 | 0.699 |
| Warm, 16 iterations | 699.8 | 669.1–701.1 | 320 | 0.699 |
| Cold, 1 iteration | 130.5 | 129.5–130.8 | 512 | 4.955 |
| Cold, 16 iterations | 690.7 | 663.4–692.0 | 320 | 2.066 |

A warm single pass maintains a much taller stack in this scene than sixteen cold
passes. Extra warm iterations barely improve the height metric, suggesting that
positional correction and residual penetration now limit this particular metric.
This does not prove that one iteration suffices for arbitrary contacts or impacts.
The default remains eight iterations.

The cold single-pass stack collapses into extra contacts, so its runtime is not a
measurement of warm-start overhead. Contact counts and compression are retained for
every sample in [world_samples.csv](world_samples.csv). These are deliberately
end-to-end quality/cost comparisons of evolving simulations.

## Parallel stages do not make a serial solver scale

The larger scene has 372 columns, 4,092 bodies, and 3,720 final contacts, using eight
warm-started solver iterations. Bodies, settings, and initial state are identical
across worker counts.

| Total threads, including caller | Median tick, ms | Run-median range, ms | Speedup vs 1 |
|---:|---:|---:|---:|
| 1 | 4.359 | 4.324–4.362 | 1.00× |
| 2 | 4.222 | 4.087–4.319 | 1.03× |
| 4 | 4.283 | 4.144–4.298 | 1.02× |
| 8 | 4.321 | 4.227–4.325 | 1.01× |

This is weak full-tick scaling. The result supports keeping single-thread execution
as the default and measuring a workload before enabling workers. Integration-only
scaling is a separate experiment in `job_system_bench`, not a substitute for this result.

The instrumented 352-body case averaged 5.74 µs integration, 2.46 µs proxy building,
2.85 µs broadphase, 4.26 µs contact generation, and 356.69 µs solving. The solver was
about 96% of the sum of instrumented stages. These stage timings have multiple clock
reads and omit World's initial all-body input-validation pass; they are diagnostic
rather than directly interchangeable with a full-tick timing. Raw data are in
[stage_samples.csv](stage_samples.csv).

A separate three-second, one-millisecond sampling profile of the 4,092-body serial
case collected 2,561 main-thread samples. Summing disjoint `ContactSolver::solve`
branches gives 2,436 samples (about 95%), largely under friction/impulse arithmetic
and Vec3/Mat3 operations. This agrees with the stage measurements. The capture spans
warmup and measured ticks and is statistical evidence, not an instruction count.
See [profile.txt](profile.txt).

```sh
python3 scripts/profile_macos.py --build build-release --output docs/performance/profile.txt
```

The profile is collected separately from timing, then terminates its own benchmark
process. On this system Instruments/xctrace was unavailable, so macOS `sample` was
used. No hardware cache-miss or branch-counter measurements were collected. A useful
next experiment would compare inlining/LTO or a sphere-specific impulse kernel while
preserving the operation sequence and replay tests; neither speedup is claimed here.

## Packing versus explicit SIMD

Each process tests 1,024 valid AABBs in sparse, dense, and all-overlapping distributions.
Each mode gets 50 warmup sweeps and 300 samples; five process repetitions are retained.
The modes see identical input and must emit identical ordered pairs. The three modes
run in a fixed order within each process, which remains an order-effect limitation.

| Distribution | Indirect scalar, µs | Packed scalar, µs | SIMD, µs |
|---|---:|---:|---:|
| Sparse | 20.33 | 12.42 | 11.17 |
| Dense | 83.67 | 60.79 | 51.40 |
| All overlapping | 2,103.04 | 2,251.85 | 1,831.54 |

For dense input, packing accounts for about a 1.38× improvement over indirect access;
SIMD contributes about 1.18× over packed scalar. For all-overlapping input, packing
alone is slower than indirect access; output emission/sorting still processes 523,776
pairs. The result is distribution-dependent rather than a universal SIMD multiplier.

Sparse indirect run medians ranged from 17.63 to 51.96 µs, a substantial variation.
The table's central estimate should be read with the full [raw sweep samples](sweep_samples.csv),
not as a stable microsecond-level guarantee. The explicit path on this machine was
NEON; x86 SSE2 performance has not been measured here.

## Memory reservation

`reservedStorageBytes()` reports vector element storage, excluding allocator metadata,
thread stacks, and fixed object members. [memory.csv](memory.csv) records the following:

| Body hint | Bodies/proxies/sweep intervals only | With 8n candidate and 4n contact hints |
|---:|---:|---:|
| 1,024 | 213,064 B | 1,826,888 B |
| 2,048 | 426,056 B | 3,653,704 B |
| 8,192 | 1,704,008 B | 14,614,600 B |

Both chosen reservation policies scale linearly. The audit's former
`reserveRigidBodies(1024)` requested 151,621,192 bytes because it reserved every
possible collision pair in several buffers. This is a changed reservation contract,
plus reduced sphere inertia storage, not a claim that quadratic contact output became
linear. Fully overlapping scenes still need quadratic storage and can grow beyond the
hints. Allocation tests cover first/steady ticks and changing contact populations under
explicit sufficient capacities on 1/2/4/8 threads.
