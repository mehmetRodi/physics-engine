# Design and API contracts

The public entry point is `World`. The core is a sphere simulation library with no
rendering dependency. Units are chosen by the caller and must be consistent; the
headless examples use meters, kilograms, seconds, and radians. The visual demo uses
pixel-scaled coordinates and a separate arena boundary rule.

## State, ownership, and supported mutation

`World` is movable and noncopyable. Its body vector owns bodies; IDs are append-only
indices with no removal/recycling. Invalid IDs throw `std::out_of_range`. A body
reference lasts until a creation or reservation reallocates that vector. There is no
synchronization for simultaneous World access: mutate/query it from one owner thread
between ticks, and never while workers are advancing it.

Position, velocity, orientation, angular velocity, accumulated acceleration/torque,
and material are public state. `World::step` validates all body inputs before advancing
any body. Mass, shape radius, inverse mass, and inverse inertia are protected:

- Mass must be finite and nonnegative; radius finite and positive.
- `setMass` and `setSphereRadius` validate derived inverse values before committing.
  Unrepresentable/underflowing inverse mass or inertia is rejected.
- Both setters clear accumulated acceleration/torque; set properties before applying
  forces for the next tick. They preserve dynamic velocity, so changing mass does not
  preserve momentum or energy automatically.
- Zero mass means static. Setting mass to zero clears motion. A static body's tick
  keeps its pose and clears assigned velocity, angular velocity, acceleration, and
  torque. There is no kinematic-body mode.
- Sphere inertia is isotropic: `I = (2/5)mr²` times the identity. Local and world inverse
  inertia accessors return the same immutable tensor. Custom anisotropic tensors are
  not supported; the general similarity transform remains tested in the math layer.

Material restitution is in [0,1]; friction and damping are finite and nonnegative.
Quaternions must have finite nonzero squared length; dynamic integration normalizes
them. Use normalized quaternions when directly assigning orientation for rendering.

`dt` is finite and nonnegative. A zero tick still refreshes contacts, solves impulses,
corrects overlap, and consumes force accumulators; it is not a pure query. Use a fixed
positive timestep for reproducible trajectories. This engine does not select a stable
step for arbitrary scales, speeds, or mass ratios.

Invalid input/settings throw `std::invalid_argument`. Size requests may throw
`std::length_error` or `std::bad_alloc`. Finite inputs can still overflow during
simulation: choose representable scales and reset/recreate a World after a failed tick.
There is no rollback guarantee for arithmetic or allocation failures midway through
a tick. This keeps a second full copy of the simulation state out of the hot path.

## Tick dependencies and contact policy

After input validation, integration updates velocity before position. Proxies are
rebuilt from the resulting body positions. Sweep-and-prune sorts by minimum x with
input index tie-breaking; candidate pairs are restored to input-index order. A
parallel narrowphase writes one contact slot and validity byte per candidate. Serial
compaction preserves that pair sequence before the solver runs.

For ordinary sphere overlap, the normal points from body B toward body A. At squared
center distance below `1e-12`, a geometric direction is unreliable. The fallback is
+x when A's ID is lower than B's and -x for reversed order. This is an arbitrary,
repeatable separation policy, not a recovered physical impact direction. Penetration
still comes from actual distance. Deep overlap is corrected gradually. The per-contact
correction cap does not bound the sum of corrections received from many contacts.

Lower-level collision routines require valid finite AABBs with min <= max and sphere
proxies with nonnegative finite radii. The low-level solver requires valid distinct
body indices, finite contact points, unit normals, nonnegative penetration/friction,
and restitution in [0,1]. `World` builds those inputs; callers bypassing World own
these preconditions. Broadphase equality tests use valid inputs.

## Solver and cache

The solver iterates contacts serially. It clamps accumulated normal impulse, allowing
an iteration to retract an earlier increment while keeping total normal impulse
nonnegative. Restitution bias comes from closing speed before warm starting. Positional
correction happens after the velocity solve, preserving contact lever arms during
iteration.

Tangential constraints are solved along two orthogonal axes, then their accumulated
vector is projected onto a disk of radius `friction * normalImpulse`. This removes the
old diagonal overshoot from independent per-axis box clamping. It remains an iterative
approximation rather than an exact coupled contact solve.

The pair-keyed cache stores a world-space tangent impulse. On the next tick it is
projected into the new tangent basis and clamped to the current friction limit. A normal
dot product below 0.95 rejects the cache entry; pairs absent from the current solve drop
out. A regression crosses the basis-selection boundary to detect a sign reversal.
Changing material or mass can make a cached magnitude a poor initial guess; normal
solver iterations adjust it. Zero velocity iterations are supported for warm-start
experiments only and are not a useful production solve setting.

## Memory behavior

`reserveRigidBodies(n)` reserves only bodies, AABB proxies, and sorted interval arrays:
O(n) storage. `reserveCollisionCapacity(pairs, contacts)` independently reserves pair
output/scratch, per-candidate contact slots, compacted contacts, constraints, and cache.
No API silently reserves `n(n-1)/2` entries merely because n bodies may exist.

Reservations are hints, not hard limits. If a scene needs more capacity, vectors grow
normally and all candidates/contacts are retained. A tick is allocation-free for the
instrumented C++ allocation paths when all required buffers already have sufficient
capacity and the thread pool is configured. Changing thread counts creates/destroys
threads outside that guarantee. Reserve before timing; measure actual workloads;
inspect `reservedStorageBytes()` for vector element storage (excluding allocator
metadata, thread stacks, and the World object's fixed members).

Worst-case candidate/contact count is still quadratic when every body overlaps. A
linear reservation policy cannot remove that output-size bound. Dense scenes may grow
beyond the hints. No strict fixed-capacity or real-time deadline mode is claimed.

## Job system

The caller participates in a fixed worker pool. An atomic index distributes chunks;
mutex/condition-variable barriers publish each batch and wait for completion. A relaxed
chunk-counter operation assigns work, while the mutexes publish the callback/context
and make worker writes visible before submission returns.

Concurrent or recursive submission on the same pool throws `std::logic_error`, including
inline execution. Destruction must not race a submission and cannot happen inside the
pool's own callback. Captured data must outlive the blocking call.

If thread creation fails after some workers started, construction stops and joins those
workers before rethrowing. Callback exceptions are captured; unclaimed work is cancelled,
all participating workers finish, then an exception is rethrown on the caller. The pool
can be reused. Callback writes are not rolled back, and which exception is captured
first is scheduling-dependent if several callbacks throw.

## Reproducibility boundary

Scheduling-independent results require identical initial state, inputs, settings, fixed
dt, executable, and floating-point environment. Pair/constraint ordering is deliberate;
independent jobs perform no shared floating-point reductions. SIMD is used for comparisons
and ordered candidate emission. GCC/Clang builds disable FP contraction and reject
several unsafe math flags, but this is not a proof of cross-platform reproducibility.

`StateSnapshot.hpp` extracts 25 explicit float fields as uint32 bit patterns. Tests compare
those fields each tick; they do not compare padded object representations. The CLI
verifier reports first divergence and computes FNV-1a over the trajectory's explicit
little-endian words. The digest is a compact diagnostic, not a cryptographic guarantee.
Snapshots contain observable body state; they do not serialize the solver cache and
cannot restore a simulation checkpoint.
