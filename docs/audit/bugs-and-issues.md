# Bugs, flawed logic, races

Issues found during a line-by-line read of the codebase. Sorted by impact:
**P1** = will likely cause a real failure under normal operation,
**P2** = will fail under unusual but plausible conditions or silently degrade
performance, **P3** = cosmetic, defensive, or future-proofing.

> **Status**: every item below has been addressed in the same branch that
> shipped this audit. Each section now ends with a **Fixed:** line citing
> the change. Two findings (KF theta innovation wrap, cv::Mat clone)
> turned out to be false alarms — the code was already correct.
>
> **Re-verified 2026-05-25**: all fixes still present in the code. Two
> have evolved further (servo handoff, cox gating) and the descriptions
> below have been updated to match what's actually in `src/` now. Line
> numbers were refreshed against the current files. A new P2 finding
> (CARRY_HOME stuck-recovery exhaustion drops box mid-arena) and a list
> of behavioural changes since the original audit are appended in the
> ["Post-audit changes"](#post-audit-changes-2026-05-25) section at the
> bottom.

For each item: a one-line title, the citation, what the code does today,
why it's wrong, and a one-line fix sketch.

---

## P1 — will bite

### obstacle_ahead dead flag
**[src/core.cpp:99](../../src/core.cpp#L99) writes; nothing reads.**

`loop_collect_frames` checks the forward arc on every LIDAR frame boundary and
stores the result into `app::obstacle_ahead`. No file in
[src/](../../src/) or [include/](../../include/) ever reads that flag. The
proximity-stop safety feature is wired to the input but not to the output —
the robot will drive straight into a wall the LIDAR can see at
`OBSTACLE_DIST_MM`.

**Fixed**: [src/core.cpp:298-301](../../src/core.cpp#L298) zeroes
`v_cmd` when `obstacle_ahead` is true *and* the command would drive
forward, but keeps `w_cmd` (so the state machine can rotate away —
"sidestep") and allows reverse motion. The gate is suppressed during
APPROACH (visual-servo handles avoidance) and during RELEASE /
TURN_AROUND where motion is reverse / in-place anyway
([src/controller.cpp:238-244](../../src/controller.cpp#L238)).

A subsequent refinement added a narrow gripper-mask cone
(`GRIPPER_MASK_HALF_DEG = 20°`, `GRIPPER_MASK_DIST_MM = 200 mm`,
[src/core.cpp:32-33](../../src/core.cpp#L32)) so the robot's own closed
gripper / held box during CARRY_HOME doesn't pin `obstacle_ahead=true`.
Beams outside the mask in the wider ±60° arc still trigger normally.

---

### TURN_AROUND has no timeout
**Original location (pre-refactor) was the `TURN_AROUND` case in
[src/controller.cpp](../../src/controller.cpp).**

The exit condition was `_turn_accumulated ≥ M_PI`. The accumulator only grows
(it adds `std::abs(...)` every tick), so a robot that can rotate at all will
eventually escape. But if the motor controller saturates or the robot is
physically blocked from rotating (wedged against a wall, gripper jammed on
something), `_turn_accumulated` stays at ~0 and the mission hangs in
TURN_AROUND forever. No state-level timeout transitions out.

**Fixed**: The TURN_AROUND state has since been repurposed as
REALIGN-to-`_home_theta` (rotate in place to face the search direction
after RELEASE), with the same wedging risk. The case at
[src/controller.cpp:1279-1303](../../src/controller.cpp#L1279) bails to
SEARCH after
[`_turn_around_timeout_s`](../../include/controller.hpp#L374) (default
8 s) with a log line; mission-level timeouts then govern the next
transition. The header comment at
[include/controller.hpp:369-371](../../include/controller.hpp#L369)
documents the rename.

---

### servo_grab() PWM brownout (now non-blocking poll-driven)
**[src/servo.c](../../src/servo.c), [src/controller.cpp](../../src/controller.cpp).**

Earlier note: the original `servo_grab()` was a single `pwmWrite()` —
non-blocking, but it slammed the PWM from OPEN_DEG to CLOSED_DEG in one
step. Hobby servos draw 1 A+ during a fast 65° slew, and the resulting
droop on the Pi's 5 V rail was browning out the SoC and hanging the
process.

The initial fix walked the angle inline (blocking ~140 ms inside
`step()`). That has since been superseded.

**Current implementation** ([src/servo.c:43-78](../../src/servo.c#L43)):
`servo_grab()` / `servo_release()` are non-blocking — they latch
`target_deg[ch]` and return immediately. A separate
[`servo_tick()`](../../src/servo.c#L55) is called from `loop_control`
([src/core.cpp:231](../../src/core.cpp#L231)) and advances each channel
by at most `STEP_DEG = 10` toward its target if `STEP_MS = 20` has
elapsed since the last step. Worst-case traversal is still
7 × 20 ms ≈ 140 ms but it happens across control ticks, not inside one
of them.

`StateMachine::step()` no longer blocks. Instead, the CARRY_HOME entry
path commands `v=0` and polls
[`servo_done(0)`](../../src/servo.c#L71) with a 3 s watchdog before
driving
([src/controller.cpp:895-921](../../src/controller.cpp#L895)). First
call on a channel still snaps (no prior state to interpolate from);
repeated calls to the same target are no-ops via the
`cur_deg == target_deg` early-out.

---

### TrackerResult atomic tearing
**(Pre-fix) `src/tracker.cpp` wrote `captured`, `lost`, `grabbable`
atomics independently across ~50 lines; readers held `lock` for the
other fields but the writer did not hold the lock for these stores.**

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
[include/types.hpp:205-207](../../include/types.hpp#L205). The vision
thread keeps local-mirror state
(`flag_captured`/`flag_lost`/`flag_grabbable` at
[src/tracker.cpp:169-171](../../src/tracker.cpp#L169)), runs all three
debounce machines on the locals, then commits the triple atomically via
[`result.commit_flags(...)`](../../include/types.hpp#L250) at
[src/tracker.cpp:433](../../src/tracker.cpp#L433). Readers go through
`get_snapshot()` which already holds `lock`.

---

### filter.ready=false passed silently to the tracker
**(Pre-fix) `main.cpp` spawned `loop_vision_tracking()` even when
calibration was aborted; the tracker did not gate on `filter.ready`.**

If the user presses `q` during interactive calibration, `filter` is
default-constructed (`ready = false`, all `cv::Scalar` zero). `main.cpp`
logs "Calibration aborted; filter not saved" then proceeds to spawn
`loop_vision_tracking(dash, filter, cam_cfg, target)` with the unready
filter. The tracker does not check `filter.ready` and feeds the zero
Scalars straight into `cv::inRange`, producing all-black masks. Every
frame becomes `detected = false`, `lost` debounces to true within
~10 frames, and the state machine sits in SEARCH forever.

**Fixed**: [main.cpp:176-189](../../main.cpp#L176) now checks both
`filter.ready=true` in config AND that `load_filter_result()` returns a
populated `FilterResult`. If either is missing it logs an error and
returns before any thread is spawned. Vision will not start with
garbage HSV bounds.

---

## P2 — silent degradation

### sensor_noise race
**(Pre-fix) `loop_localization` wrote `lidar_data.sensor_noise` as a
plain `double` while readers in cox and calibrations consumed it from
other threads.**

`get_lidar_sensor_noise()` runs after a 2 s sleep at boot, but
`loop_collect_frames` is already writing into the buffer. Even if you
trust the write to be torn-free on this ARM target (double = 8 bytes, may
or may not be atomic), the value is used to gate cox's inlier threshold
and find_walls' RANSAC threshold — a wrong noise estimate skews both.

**Fixed**: `sensor_noise` is now `std::atomic<double>` at
[include/types.hpp:35](../../include/types.hpp#L35); the one writer
[src/core.cpp:108](../../src/core.cpp#L108) uses `.store()` and the
readers at [src/cox.cpp:73](../../src/cox.cpp#L73) and
[src/calibrations.cpp:596](../../src/calibrations.cpp#L596) use
`.load()`.

---

### Cox ridge inflates KF trust
**[src/cox.cpp:184-187](../../src/cox.cpp#L184).**

Cox adds `RIDGE = 1e-3` to the normal matrix diagonal before inversion.
This regularises ill-conditioned scans, but it also shrinks the reported
covariance (which is `s² · A.inverse()`). The KF then trusts cox more
than it should, drawing the fused pose harder toward the cox estimate
when the geometry is actually poorly observed.

**Fixed (then revised)**: The original fix computed the eigenvalues of
the information matrix `A` after the iteration loop and stored the
condition number on `PosEstimate.cond_number`
([src/cox.cpp:227-238](../../src/cox.cpp#L227), via
`SelfAdjointEigenSolver`), then gated `cox_can_fuse()` on
`cond_number > 1e5` and `cox_strong_fix()` on `cond_number ≤ 1e4`. In
the 2-wall L-corner geometry of the arena, `cond` naturally runs ~1e6
even when sigmas are healthy (~2 mm) — the cond gate killed every fuse
([src/cox.cpp:260-264](../../src/cox.cpp#L260) documents this).

The gate now lives on the sigmas directly, which is what `cond` was
trying to proxy:
[`MIN_INLIERS_FUSE = 20`, `MAX_SIGMA_XY_FUSE = 30 mm`,
`MAX_SIGMA_THETA_FUSE = 0.025 rad`](../../src/cox.cpp#L265).
`cox_strong_fix()` uses the same shape with tighter sigma caps
([src/cox.cpp:274-276](../../src/cox.cpp#L274)) and additionally
requires `converged`. `cond_number` is still computed and surfaced on
the Cox dashboard row ([src/cox.cpp:540](../../src/cox.cpp#L540)) but
no longer participates in the fuse decision.

---

### KF correct theta wrap — false alarm
**[src/kalman.cpp:95](../../src/kalman.cpp#L95).**

The audit claimed the innovation `y(2) = z.theta - x.theta` was
unwrapped. Re-reading the file, the assignment is
`y(2) = wrap_pi(z.theta - _pose.theta)` — already correct. No change
needed.

---

### Cox covariance can be rank-deficient and still pass `cox_can_fuse`
**[src/cox.cpp:278-294](../../src/cox.cpp#L278).**

`cox_can_fuse` originally checked `est.converged && est.inliers >= 6`.
A single long wall produces a converged solve with healthy inliers —
but the perpendicular direction is unconstrained. The ridge
regularization hides the singularity; the reported covariance is finite
but the true uncertainty in the unobserved direction is infinite.

**Fixed**: same change as "Cox ridge inflates KF trust" above.
`cox_can_fuse()` now requires `sigma_x`, `sigma_y`, `sigma_a` finite
and each below its `MAX_SIGMA_*_FUSE` cap. A geometrically degenerate
scan blows `sigma_y` (or whichever axis is unconstrained) well past
30 mm and is refused before the KF update.

---

### save_filter_result is not atomic
**(Pre-fix) `save_filter_result()` re-opened `config.yml` with
`std::ios::trunc` and rewrote it inline; a crash between truncate and
full write left the user with a half-written config.**

**Fixed**: `save_filter_result()` now writes to `config.yml.tmp`, closes
the file, then `std::rename()` atomically replaces the original
([src/calibrations.cpp:425-450](../../src/calibrations.cpp#L425)). On
rename failure the tmp file is removed and the original stays intact.

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
**Pre-fix: APPROACH timed out at 20 s, robot bounced back to SEARCH,
re-acquired the same box, and re-entered APPROACH after the search
dwell. Cycle repeated until SAFE_STOP.**

**Fixed**: the failed approach now latches a world-frame position
(`_last_failed_target_x/y` +
[`_last_failed_at`](../../include/controller.hpp#L149)) into the
StateMachine when APPROACH times out
([src/controller.cpp:1051-1053](../../src/controller.cpp#L1051)).
SEARCH's at-home / at-failure skip logic
([src/controller.cpp:616-628](../../src/controller.cpp#L616)) treats
any detection within
[`_failed_target_skip_radius_mm` (200 mm)](../../include/controller.hpp#L378)
and [`_failed_target_skip_s` (30 s)](../../include/controller.hpp#L379)
of the failure point exactly like an already-delivered box. The memory
decays so a different box at a similar bearing later isn't ignored, and
`start()` clears it for fresh missions
([src/controller.cpp:145](../../src/controller.cpp#L145)).

---

### Telemetry sends a default PathPoint when no path is loaded
**`telemetry::send_state` call at
[src/core.cpp:334-338](../../src/core.cpp#L334) +
`pp.get_active_target()`.**

When the state machine is in IDLE/SEARCH (rotating, not following a
path), `pp.get_active_target()` returns a default `PathPoint` with all
zeros. The Matlab viewer renders that as "target at (0, 0, 0)", which is
inside the arena (likely the home zone), masking the actual mission
state.

**Fixed**:
[`PurePursuitController::has_active_target()`](../../include/pure_pursuit.hpp#L38)
returns false when the controller is idle or before the first
`set_path()` ([src/pure_pursuit.cpp:69](../../src/pure_pursuit.cpp#L69)).
[`telemetry::send_state`](../../include/telemetry.hpp#L24) takes a
`target_valid` parameter and emits a
[`"target_valid":0/1` JSON field](../../src/telemetry.cpp#L181);
receivers should gate the marker on that flag instead of comparing
`target` to `(0, 0, 0)`.

---

## P3 — cosmetic / future-proofing

### Duplicate angle wrapping
Inline `wrap_pi(a - b)` is spread across the controller
([src/controller.cpp](../../src/controller.cpp) — ~10 sites including
`322`, `381`, `701`, `719`, `742`, `784`, `841`, `1029`, `1286`),
pure_pursuit ([src/pure_pursuit.cpp:207, 234](../../src/pure_pursuit.cpp#L207)),
and kalman ([src/kalman.cpp:64, 95, 106](../../src/kalman.cpp#L64)).

**Fixed**: Added
[`angle_error(goal, current)`](../../include/utils.hpp#L36) to
`include/utils.hpp`. Existing call sites kept their inline form to
avoid a wide diff; new code should use the helper.

---

### LIDAR-to-body transform duplicated
Both `src/cox.cpp` and `src/calibrations.cpp` historically applied
`-angles[i] * (M_PI / 180.0) + LIDAR_MOUNT_OFFSET_RAD` inline.

**Fixed**: Added
[`lidar_polar_to_body(angle_deg, dist, x, y)`](../../include/utils.hpp#L45)
to `include/utils.hpp`. The map-building site in `find_walls()` uses it
at [src/calibrations.cpp:590](../../src/calibrations.cpp#L590) with a
referencing comment at
[:587](../../src/calibrations.cpp#L587). The cox inner-loop site kept
its hand-coded `float` version because the helper is `double` and the
inner loop is performance-sensitive — both still cite the helper as
the canonical sign-convention source.

---

### Hysteresis counters grow without bound
(Pre-fix) the tracker's debounce counters were plain `int` and
incremented unconditionally each frame.

**Fixed**: Counters are now `unsigned` and incremented via a local
[`clamp_inc(c, cap)`](../../src/tracker.cpp#L162) lambda that saturates
at the threshold (used at tracker.cpp
[:363](../../src/tracker.cpp#L363),
[:377](../../src/tracker.cpp#L377),
[:391](../../src/tracker.cpp#L391),
[:408](../../src/tracker.cpp#L408),
[:418](../../src/tracker.cpp#L418)). No UB risk, no unbounded growth.

---

### UDP sendto has no backpressure
Pre-fix: `send_raw()` ignored the `sendto` return; failed sends were
silently dropped.

**Fixed**: `send_raw()` now checks the `sendto` return and increments a
cumulative [`g_drop_count`](../../src/telemetry.cpp#L38) on failure. A
rate-limited warning (`once / 1000 ms`) logs the running drop total via
`LOGE("Telemetry")` ([src/telemetry.cpp:54](../../src/telemetry.cpp#L54)).

---

### `_search_skip_streak` write/lock asymmetry
[src/controller.cpp](../../src/controller.cpp). Writes are unlocked
(single-threaded by contract); the dashboard snapshot read is locked.

**Fixed (documentation)**: the existing comment block already explains
the single-thread argument; no functional change. If `step()` ever
moves off `loop_control`, the writes will need a lock.

---

### `_goto_pending` displaced target
A second `go_to()` before the first one is consumed silently
overwrites the latched target.

**Fixed**: `go_to()` now detects an unconsumed pending target
([src/controller.cpp:189](../../src/controller.cpp#L189)) and logs
the displaced coordinates before overwriting them
([src/controller.cpp:202](../../src/controller.cpp#L202)).

---

## Remediation status

All items above are addressed in the same branch. `make check` builds
every C++ TU cleanly with the fixes in place; the only build-graph
failure is the pre-existing servo.c→wiringPi gap on the dev host.

### Behavioural changes worth knowing

| Change | Effect |
|---|---|
| `obstacle_ahead` now gates forward motion | Robot stops on a wall the LIDAR sees within the forward arc; reverse motion still allowed. Gripper-mask cone added so the held box doesn't trip it during CARRY_HOME. |
| TURN_AROUND timeout (8 s) | Mission no longer wedges on a jammed motor. State is now REALIGN-to-`_home_theta` semantically. |
| Failed-target skip in SEARCH | One unreachable box no longer wastes the entire approach budget. |
| Cox sigma gate | Long-corridor / single-wall scans no longer feed the KF a falsely-confident correction. The `cond` column on the Cox row is the live indicator (the original cond-number gate was replaced by sigma gates after it killed valid fuses in L-corner geometry). |
| Telemetry `target_valid` flag | Receivers should consume this; legacy receivers reading `target` alone need updating. |
| `commit_flags()` on TrackerResult | If any external code touches `result.captured/lost/grabbable` directly (it shouldn't — but check), it now goes through plain bools under `lock`. |

---

## Post-audit changes (2026-05-25)

This section captures behavioural shifts and one new finding from a
2026-05-25 re-read of the branch. The original audit's fixes are still
in place; these are additions or revisions that landed afterwards.

### Re-verification summary

All P1/P2/P3 items in the sections above are still present in the code.
The two fixes whose implementation has materially changed since the
original audit:

- **Servo grip handoff** — the inline "blocking ~140 ms walk" was
  replaced by a non-blocking poll-driven scheme
  (`servo_grab()`/`servo_release()` latch a target;
  [`servo_tick()`](../../src/servo.c#L55) called from `loop_control`
  steps once per `STEP_MS`). `StateMachine::step()` no longer blocks;
  CARRY_HOME entry polls
  [`servo_done(0)`](../../src/servo.c#L71) with a 3 s watchdog
  ([src/controller.cpp:895-921](../../src/controller.cpp#L895)). The
  "P1 — servo_grab" section above has been rewritten to reflect this.

- **Cox cond-number gate replaced by sigma gates** — see the rewritten
  "Cox ridge inflates KF trust" and "Cox covariance can be
  rank-deficient" sections above. `cond_number` is still computed and
  dashboarded but no longer participates in the fuse decision.

### New finding (2026-05-25)

#### P2 — CARRY_HOME stuck-recovery exhaustion releases mid-arena
**[src/controller.cpp:1223-1235](../../src/controller.cpp#L1223).**

The CARRY_HOME stuck-recovery sub-state machine
([added in d0c50f4](../../src/controller.cpp#L1150)) detects
no-progress via a sliding-window anchor and runs up to
`_carry_recovery_max_attempts` (default 2) reverse-then-curl recovery
cycles. When all attempts are exhausted, the current behaviour is:

```cpp
LOG("Mission") << "... releasing in place (" << dist_to_home
               << " mm from home) and returning to SEARCH.";
_transition(State::RELEASE, pose);
```

This opens the gripper **regardless of distance to the home zone**.
Effect: a robot stuck against an obstacle anywhere in the arena will
litter the held box on the floor mid-arena, then re-enter SEARCH and
likely re-acquire the same dropped box (subject to the
`_failed_target_skip_radius_mm` / `_failed_target_skip_s` gate from the
SEARCH-thrash fix above).

Commit `0b285fc` ("add position guard to prevent gripper release
outside home zone") explicitly added a guard in the previous CARRY_HOME
implementation that refused to release outside the home zone and
SAFE_STOPped instead. Commit `2789ecb`
("refactor CARRY_HOME logic: enhance path retry mechanism…") rewrote
the CARRY_HOME case and the new stuck-recovery exhaustion path does
not restore that guard. The has-arrived path
([:1114-1136](../../src/controller.cpp#L1114)) does re-plan to home
rather than releasing far from it, but it has no equivalent of the
"out-of-zone → SAFE_STOP" check.

**Severity**: P2. The drop usually lands close to home (recovery only
fires after the robot already drove most of the way), but a stuck
event triggered early in CARRY_HOME can drop a box well inside the
arena where it will be re-targeted as if it were a fresh delivery
candidate.

**Suggested fix**: gate the exhaustion-RELEASE on `at_home`
(`dist_to_home <= _home_margin`). When false, either SAFE_STOP with
the box still gripped (matches the spirit of `0b285fc`) or fall through
to one more straight-shot plan attempt before SAFE_STOP. Operator
intervention is cheaper than mis-categorised boxes.

#### Drag guard removed (informational, not a finding)

The same `2789ecb` refactor removed the
`_carry_target_lost_timeout_s` / `_carry_last_target_seen_at` drag
guard that watched for "captured stayed true but vision never saw the
box again" during CARRY_HOME. The stuck-recovery state machine
nominally subsumes the same failure mode (the box-swap scenario the
drag guard targeted would also manifest as lack-of-progress), but
there's no explicit drag-time check anymore. Noted here so future
auditors don't go looking for it.
