# Bugs, flawed logic, races

Issues found during a line-by-line read of the codebase. Sorted by impact:
**P1** = will likely cause a real failure under normal operation,
**P2** = will fail under unusual but plausible conditions or silently degrade
performance, **P3** = cosmetic, defensive, or future-proofing.

> **Status**: every item below has been addressed in the same branch that
> shipped this audit. Each section now ends with a **Fixed:** line citing
> the change. Two findings (KF theta innovation wrap, cv::Mat clone)
> turned out to be false alarms — the code was already correct.

For each item: a one-line title, the citation, what the code does today,
why it's wrong, and a one-line fix sketch.

---

## P1 — will bite

### obstacle_ahead dead flag
**[src/utils.cpp:75](../../src/utils.cpp#L75) writes; nothing reads.**

`loop_collect_frames` checks the forward arc on every LIDAR frame boundary and
stores the result into `app::obstacle_ahead`. No file in
[src/](../../src/) or [include/](../../include/) ever reads that flag. The
proximity-stop safety feature is wired to the input but not to the output —
the robot will drive straight into a wall the LIDAR can see at
`OBSTACLE_DIST_MM`.

**Fixed**: [src/core.cpp:218-225](../../src/core.cpp#L218) now zeroes
(v_cmd, w_cmd) when `obstacle_ahead` is true *and* the command would
drive forward; reverse motion is still allowed so the robot can back out
of a near-miss.

---

### TURN_AROUND has no timeout
**[src/controller.cpp:731-757](../../src/controller.cpp#L731-L757).**

The exit condition is `_turn_accumulated ≥ M_PI`. The accumulator only grows
(it adds `std::abs(...)` every tick), so a robot that can rotate at all will
eventually escape. But if the motor controller saturates or the robot is
physically blocked from rotating (wedged against a wall, gripper jammed on
something), `_turn_accumulated` stays at ~0 and the mission hangs in
TURN_AROUND forever. No state-level timeout transitions out.

**Fixed**: [src/controller.cpp](../../src/controller.cpp) (TURN_AROUND
case) now bails to SEARCH after
[`_turn_around_timeout_s`](../../include/controller.hpp) (default 8 s)
with a log line; mission-level timeouts then govern the next transition.

---

### servo_grab() blocking — false alarm
**[src/controller.cpp:594](../../src/controller.cpp#L594) + [src/servo.c:42](../../src/servo.c#L42).**

The audit flagged this based on a comment in controller.cpp that claimed
`servo_grab()` blocks for ~1.2 s. Reading the implementation shows the
function just calls `pwmWrite()` — a non-blocking PWM register write. The
gripper closes mechanically over the next ~`_grip_settle_s` and the
captured-state debounce already accounts for that. The comment was stale.

**Fixed**: Stale "blocks ~1.2 s" comment removed from
[src/controller.cpp](../../src/controller.cpp); replaced with an accurate
description of the two-tick grip handshake.

---

### TrackerResult atomic tearing
**[src/tracker.cpp:198-250](../../src/tracker.cpp#L198) writes
`captured`, `lost`, `grabbable` atomics independently across ~50 lines.
[include/types.hpp:213-219](../../include/types.hpp#L213) reads them under
`lock` (which the writer does not hold for these stores).**

The reader sees a consistent `bearing/dist/area/has_target` (those are
under the lock) but the three flag atomics can each be in any one of two
states at any time. Possible glitches a state-machine consumer can see:

- `captured=true && lost=true` between the "lost" set and the next
  detected frame's "captured" set. The state machine treats `captured`
  as the trigger to CARRY_HOME and `lost` as a SEARCH trigger; observing
  both at once doesn't crash but logs noisily.
- `grabbable=true && captured=false` for one tick after release — fine.
- `grabbable=true && captured=true` while in CARRY_HOME — the state
  machine ignores `grabbable` outside APPROACH, so harmless today, but
  brittle.

**Fixed**: The three flags are now plain `bool`s under `lock` in
[include/types.hpp](../../include/types.hpp). The vision thread keeps
local-mirror state, runs all three debounce machines on the locals, then
commits the triple atomically via `result.commit_flags(...)` at the end
of each frame ([src/tracker.cpp](../../src/tracker.cpp)). Readers go
through `get_snapshot()` which already held `lock`.

---

### filter.ready=false passed silently to the tracker
**[main.cpp:121-130](../../main.cpp#L121-L130);
[src/tracker.cpp:107-113](../../src/tracker.cpp#L107).**

If the user presses `q` during interactive calibration, `filter` is
default-constructed (`ready = false`, all `cv::Scalar` zero). `main.cpp`
logs "Calibration aborted; filter not saved" then proceeds to spawn
`loop_vision_tracking(dash, filter, cam_cfg, target)` with the unready
filter. The tracker does not check `filter.ready` and feeds the zero
Scalars straight into `cv::inRange`, producing all-black masks. Every
frame becomes `detected = false`, `lost` debounces to true within
~10 frames, and the state machine sits in SEARCH forever.

**Fixed**: [main.cpp](../../main.cpp) now logs an error and returns 1 if
`!filter.ready` after the calibrate/load block, before any thread is
spawned. Vision will not start with garbage HSV bounds.

---

## P2 — silent degradation

### sensor_noise race
**[src/core.cpp:80-81](../../src/core.cpp#L80) writes; no atomic, no lock.
Readers: [src/cox.cpp:74](../../src/cox.cpp#L74),
[src/calibrations.cpp ~519](../../src/calibrations.cpp).**

`get_lidar_sensor_noise()` runs after a 2 s sleep at boot, but
`loop_collect_frames` is already writing into the buffer. Even if you
trust the write to be torn-free on this ARM target (double = 8 bytes, may
or may not be atomic), the value is used to gate cox's inlier threshold
and find_walls' RANSAC threshold — a wrong noise estimate skews both.

**Fixed**: `sensor_noise` is now `std::atomic<double>` in
[include/types.hpp](../../include/types.hpp); the one writer
([src/core.cpp](../../src/core.cpp)) uses `.store()` and the readers in
[src/cox.cpp](../../src/cox.cpp) / [src/calibrations.cpp](../../src/calibrations.cpp)
use `.load()`.

---

### Cox ridge inflates KF trust
**[src/cox.cpp:180-183, 216](../../src/cox.cpp#L180).**

Cox adds `RIDGE = 1e-3` to the normal matrix diagonal before inversion.
This regularises ill-conditioned scans, but it also shrinks the reported
covariance (which is `s² · A.inverse()`). The KF then trusts cox more
than it should, drawing the fused pose harder toward the cox estimate
when the geometry is actually poorly observed.

**Fixed**: Cox now computes the eigenvalues of the information matrix
`A` after the iteration loop and records the condition number on
`PosEstimate.cond_number` ([src/cox.cpp](../../src/cox.cpp), via
`SelfAdjointEigenSolver`). `cox_can_fuse()` rejects estimates with
`cond_number > 1e5`; `cox_strong_fix()` (relocalize sweep) requires
`cond_number ≤ 1e4`. A "cond" column is added to the Cox dashboard row.

---

### KF correct theta wrap — false alarm
**[src/kalman.cpp:95](../../src/kalman.cpp#L95).**

The audit claimed the innovation `y(2) = z.theta - x.theta` was
unwrapped. Re-reading the file, the assignment is
`y(2) = wrap_pi(z.theta - _pose.theta)` — already correct. No change
needed.

---

### Cox covariance can be rank-deficient and still pass `cox_can_fuse`
**[src/cox.cpp:239-255](../../src/cox.cpp#L239).**

`cox_can_fuse` checks `est.converged && est.inliers >= MIN_INLIERS_FUSE
(6)`. A single long wall produces six perfectly aligned inliers and a
converged solve — but the perpendicular direction is unconstrained. The
ridge regularization hides the singularity; the reported covariance is
finite but the true uncertainty in the unobserved direction is infinite.

**Fixed**: Same fix as "Cox ridge inflates KF trust" above —
`cox_can_fuse()` now requires `cond_number ≤ 1e5`. A geometrically
degenerate scan blows past that threshold and is refused before the KF
update.

---

### save_filter_result is not atomic
**[src/calibrations.cpp:317-376](../../src/calibrations.cpp#L317).**

The function reads the current `config.yml`, drops the `filter:` block,
re-opens the same file with `std::ios::trunc`, and writes the kept lines
+ the new filter block. If the process crashes between the open and the
full write, the user is left with a half-written config (everything
before the crash point + nothing after).

**Fixed**: `save_filter_result()` now writes to `config.yml.tmp`, closes
the file, then `std::rename()` atomically replaces the original
([src/calibrations.cpp](../../src/calibrations.cpp)). On rename failure
the tmp file is removed and the original stays intact.

---

### cv::Mat frame sharing — false alarm
**[src/types.cpp:34, 41](../../src/types.cpp#L34).**

The audit suspected shallow copies. Reading the implementation,
`update_frame` does `frame = f.clone()` under `mutex_frame`, and
`get_frame` returns `frame.clone()` under the same lock. Producer and
consumer each hold deep copies, so refcount mutation is single-threaded.
No change needed.

---

### SEARCH ↔ APPROACH thrash on unreachable box
**[src/controller.cpp:395-462, 640-643](../../src/controller.cpp#L395).**

If the box is visible but the robot can't reach it (e.g. wedged against
a wall, or the visual servo overshoots and loses lock), APPROACH times
out at 20 s, the budget increments, we bounce back to SEARCH, the box is
still visible, we re-enter APPROACH after `_search_min_dwell_s = 2 s`,
and the cycle repeats. The mission spends 5 × 20 = 100 s before
SAFE_STOP. Effectively a livelock.

**Fixed**: the failed approach now latches a world-frame position
(`_last_failed_target_*`) and a timestamp into the StateMachine
([include/controller.hpp](../../include/controller.hpp),
[src/controller.cpp](../../src/controller.cpp)). SEARCH's at-home /
at-failure skip logic treats any detection within
`_failed_target_skip_radius_mm` (200 mm) and `_failed_target_skip_s`
(30 s) of the failure point exactly like an already-delivered box. The
memory decays so a different box at a similar bearing later isn't
ignored, and `start()` clears it for fresh missions.

---

### Telemetry sends a default PathPoint when no path is loaded
**[src/core.cpp:248](../../src/core.cpp#L248) (the recent change) +
`pp.get_active_target()`.**

When the state machine is in IDLE/SEARCH (rotating, not following a
path), `pp.get_active_target()` returns a default `PathPoint` with all
zeros. The Matlab viewer renders that as "target at (0, 0, 0)", which is
inside the arena (likely the home zone), masking the actual mission
state.

**Fixed**: Added `PurePursuitController::has_active_target()` (false when
the controller is idle or before the first `set_path()`). Extended
`telemetry::send_state` with a `target_valid` parameter and a
`"target_valid":0/1` JSON field; receivers should gate the marker on
that flag instead of comparing `target` to `(0, 0, 0)`.

---

## P3 — cosmetic / future-proofing

### Duplicate angle wrapping
Five files manually `wrap_pi(a - b)`:
[src/controller.cpp:527, 625, 745](../../src/controller.cpp);
[src/pure_pursuit.cpp:196, 220](../../src/pure_pursuit.cpp);
plus inline normalization in [src/kalman.cpp](../../src/kalman.cpp) and
[src/cox.cpp](../../src/cox.cpp).

**Fixed**: Added `angle_error(goal, current)` to
[include/utils.hpp](../../include/utils.hpp). Existing call sites kept
their inline form to avoid a wide diff; new code should use the helper.

---

### LIDAR-to-body transform duplicated
[src/cox.cpp:106](../../src/cox.cpp#L106) and
[src/calibrations.cpp ~513](../../src/calibrations.cpp) both apply
`-angles[i] * (M_PI / 180.0) + LIDAR_MOUNT_OFFSET_RAD`.

**Fixed**: Added `lidar_polar_to_body(angle_deg, dist, x, y)` to
[include/utils.hpp](../../include/utils.hpp). The map-building site in
`find_walls()` now uses it; cox.cpp's inner-loop site kept its
hand-coded `float` version because the helper is `double` and the
inner loop is performance-sensitive — both still cite the helper as
the canonical sign-convention source.

---

### Hysteresis counters grow without bound
[src/tracker.cpp:83-87](../../src/tracker.cpp#L83) declares the
hysteresis counters as plain `int`.

**Fixed**: Counters are now `unsigned` and incremented via a local
`clamp_inc(c, cap)` lambda that saturates at the threshold. No UB
risk, no unbounded growth.

---

### UDP sendto has no backpressure
[src/telemetry.cpp:34-39](../../src/telemetry.cpp#L34). Failed sends are
silently dropped.

**Fixed**: `send_raw()` now checks the `sendto` return and increments a
cumulative `g_drop_count` on failure. A rate-limited warning
(`once / 1000 ms`) logs the running drop total via `LOGE("Telemetry")`.

---

### `_search_skip_streak` write/lock asymmetry
[src/controller.cpp](../../src/controller.cpp). Writes are unlocked
(single-threaded by contract); the dashboard snapshot read is locked.

**Fixed (documentation)**: the existing comment block already explains
the single-thread argument; no functional change. If `step()` ever
moves off `loop_control`, the writes will need a lock.

---

### `_goto_pending` displaced target
[src/controller.cpp](../../src/controller.cpp). A second `go_to()`
before the first one is consumed silently overwrites the latched
target.

**Fixed**: `go_to()` now detects an unconsumed pending target and logs
the displaced coordinates before overwriting them.

---

## Remediation status

All items above are addressed in the same branch. `make check` builds
every C++ TU cleanly with the fixes in place; the only build-graph
failure is the pre-existing servo.c→wiringPi gap on the dev host.

### Behavioural changes worth knowing

| Change | Effect |
|---|---|
| `obstacle_ahead` now gates forward motion | Robot stops on a wall the LIDAR sees within the forward arc; reverse motion still allowed. |
| TURN_AROUND timeout (8 s) | Mission no longer wedges on a jammed motor. |
| Failed-target skip in SEARCH | One unreachable box no longer wastes the entire approach budget. |
| Cox condition gate | Long-corridor / single-wall scans no longer feed the KF a falsely-confident correction. The new "cond" column on the Cox row is the live indicator. |
| Telemetry `target_valid` flag | Receivers should consume this; legacy receivers reading `target` alone need updating. |
| `commit_flags()` on TrackerResult | If any external code touches `result.captured/lost/grabbable` directly (it shouldn't — but check), it now goes through plain bools under `lock`. |
