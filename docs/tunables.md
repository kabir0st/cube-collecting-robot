# Tunables reference

Single index of every knob that changes runtime behavior, with what each one
does and when to push it up or down. Calibration constants (encoder
ticks/mm, wheelbase) are intentionally excluded — those are measured, not
tuned.

Conventions:
- `mm` is millimeters, `rad` radians, `deg` degrees (unless stated).
- Body frame: `+X` forward, `+Y` left, `+θ` CCW.
- Every velocity / accel cap that scales with `ROBOT_SPEED` is noted; the
  number you see in the source is the per-unit value (multiply by the
  configured `ROBOT_SPEED` to get the runtime cap).

---

## 1. Global

### `robot.speed` — [config.yml](../config.yml)
Multiplier applied to every linear/angular velocity cap and accel limit.
1.0 is the baseline tune; arena run uses 3.6.
- Increase: faster cruise, faster turns, shorter mission time.
- Decrease: gentler motion (cox folds in more corrections per move),
  safer near walls or weak grip.
- Cap: above ~3.6 the motor silently clips omega on tight turns
  (chassis hardware ceiling ~504 mm/s / ~5.45 rad/s).

### `mission.home_margin_mm` — [config.yml](../config.yml)
Acceptance radius around home for "box delivered". PurePursuit declares
arrival when the chassis is within this radius and at the terminal
heading.
- Increase: easier arrival, accepts sloppier final positioning.
- Decrease: tighter delivery; mission may stall if cox/KF can't
  converge below the radius.

### `_home_skip_radius_mm = 500` — [include/controller.hpp](../include/controller.hpp)
Radius around home inside which SEARCH ignores a detected box (treats
it as already delivered). Decoupled from `home_margin_mm` because the
released box can settle well off the chassis-stop center and a tight
arrival margin would let SEARCH re-engage drops.
- Increase: ignore boxes further from home center.
- Decrease: only filter very-on-target drops; risk of re-engaging
  delivered boxes that drifted off.

In addition to the radius, SEARCH also skips any box whose projection
onto `(cos(home_theta), sin(home_theta))` from home is negative
("behind home"). The search sweep is forward-only and the wall sits
on the back side of home, so anything in that half-plane is either a
delivered drop or impossible. No tunable — the cut is the home line
itself.

### `mission.arm_length_mm` — [config.yml](../config.yml)
LIDAR center → gripped-box center, along robot heading. Currently
unused by CARRY_HOME (which drives straight to `home_x, home_y` with
no offset); kept for future calibration.
- Re-measure if the gripper hardware changes.

### `camera.show_debug_view` — [config.yml](../config.yml)
Opens an OpenCV window with the tracker's annotated frame.
- Enable: tuning the color filter / debugging vision drops.
- Disable on the live mission: window adds ~10-20 ms per frame.

---

## 2. PurePursuit (path follower)

All in [include/pure_pursuit.hpp](../include/pure_pursuit.hpp).

### `_L_d = 250 mm`
Lookahead distance along the path arc, measured from the closest path
point. PurePursuit steers toward the point this far ahead.
- Increase: smoother curves, less oscillation on straights, but the
  controller cuts corners on tight turns and may overshoot end-of-path.
- Decrease: sharper tracking on curves, but the bot oscillates around
  the centerline on long straights.

### `_v_max = 75.0 * ROBOT_SPEED`
Absolute forward-velocity cap in PURSUIT (mm/s). Each path point also
carries its own `v_max` from the trapezoidal profile; the smaller wins.
- Increase: faster path follow.
- Decrease: helps with tight curves (smaller `v` raises achievable
  turn radius for the same omega cap, `r_min = v / omega_max`).

### `_omega_max = 2.0 rad/s` (NOT scaled with ROBOT_SPEED)
Absolute yaw-rate cap. ~11° per cox tick at 10 Hz — the upper bound
where motion-compensated scan matching still tracks.
- Increase: lets the bot follow tighter curves. Above ~2.5 rad/s cox
  starts to lose lock during fast turns.
- Decrease: more cox stability, but the bot can no longer follow
  high-curvature splines and orbits at the cap (see `tunables` history
  in CARRY_HOME).

### `_heading_done_rad = 0.05` (~3°)
TERMINAL_ROTATE exit gate. PurePursuit holds in TERMINAL_ROTATE until
heading error to the final pose drops below this.
- Increase: arrival accepts slop, faster state exit.
- Decrease: tighter heading, but the bot may sit oscillating in place
  if odometry/cox have residual jitter larger than the threshold.

### `_pre_roll_thresh = 0.52 rad` (~30°)
Heading error above which set_path() phase enters PRE_ROLL (in-place
spin) instead of PURSUIT directly.
- Increase: bot starts driving sooner with imperfect alignment.
  Likely to throw a fishhook on U-turns.
- Decrease: more in-place pre-alignment. Costs time but cleaner curves.

### `_k_pre_roll = 2.0`, `_k_terminal = 2.0`
Proportional gains: `omega = k * angle_error` for PRE_ROLL and
TERMINAL_ROTATE.
- Increase: faster convergence, may overshoot the deadband and
  oscillate.
- Decrease: tame slewing, longer settling time.

### `_inplace_omega_cap = 0.8 rad/s`
Cap on `omega` during in-place spins (PRE_ROLL, TERMINAL_ROTATE). Lower
than `_omega_max` so wheel-slip drift stays bounded during pure rotation.
- Increase: shaves seconds off 180° flips.
- Decrease: cleaner heading estimate (less wheel slip per rotation).

---

## 3. State machine — timeouts

All in [include/controller.hpp](../include/controller.hpp).

### `_search_timeout_s = 60.0`
Max time in SEARCH before SAFE_STOP.
- Increase: more retries if the arena is sparse.
- Decrease: faster failure (useful when you know the box is reachable
  quickly).

### `_approach_timeout_s = 20.0`
Max time in APPROACH before falling back to SEARCH and marking the
target as failed (avoid re-engaging same box for `_failed_target_skip_s`).
- Increase: stubborn approaches in jittery vision.
- Decrease: faster bail on unreachable boxes.

### `_carry_timeout_s = 90.0`
Max time in CARRY_HOME (straight-line drive to home) before SAFE_STOP.
- Increase: long path-home distances or slow speeds.
- Decrease: catch a wedged carry sooner.

### `_turn_around_timeout_s = 8.0`
Max time in REALIGN (post-release rotation back to `_home_theta`).
- Increase: large `home_theta` deltas with slow `_inplace_omega_cap`.
- Decrease: bail to SEARCH faster if the arc plan stalls on an obstacle.

### `_release_seconds = 2.0`, `_release_hard_timeout_s = 5.0`
Minimum back-up time (covers servo open slew + clearance) and absolute
timeout if the tracker keeps reporting `captured`.
- Increase `_release_seconds`: clear more distance from the box (useful
  if the chassis rotation in TURN_AROUND keeps clipping it).
- Decrease: faster cycles when release is reliably clean.

---

## 4. State machine — per-state behavior

### SEARCH

#### `_search_omega = 0.35 * ROBOT_SPEED` rad/s
Rotation rate during the LOOK leg of SEARCH. ~20°/s at baseline so
the camera sees enough overlapped frames to lock on a passing target.
- Increase: faster sweep but missed-target risk if vision FPS is low.
- Decrease: more reliable target acquisition in low light / weak filter.

#### `_search_v_forward_mm_s = 40 * ROBOT_SPEED`
Cruise speed during the FORWARD leg.
- Increase: cover the arena faster.
- Decrease: better odom/cox fusion in a long traversal.

#### `_search_forward_target_mm = 800`
Distance per FORWARD leg between sweeps. The sweep itself locks to a
fixed axis (`forward_heading_ref`, snapshotted on SEARCH entry) — every
cycle's LOOK_LEFT / LOOK_RIGHT targets are absolute (ref ± π/2) and
FORWARD realigns to ref before driving. No per-cycle drift.
- Increase: fewer sweeps per unit area, more coverage per cycle.
- Decrease: more frequent visibility sweeps for targets behind/beside.

#### `_search_realign_enter_rad = 0.15`, `_search_realign_exit_rad = 0.05`, `_search_realign_stable_ticks = 3`
Hysteresis on the FORWARD-leg realign-to-ref sub-state. Realign engages
when heading error exceeds `enter_rad`; disengages only after error
sits inside `exit_rad` for `stable_ticks` consecutive ticks. The gap
between enter/exit + the stable counter kills the R/L/FWD flap that
raw cox heading jitter caused near a single threshold.
- Widen `enter` / tighten `exit`: more dead zone, less chatter at the
  cost of slightly drifted forward heading.
- Increase `stable_ticks`: tolerates more cox noise mid-realign, costs
  slightly more time before resuming forward.

#### `_search_min_dwell_s = 0.0`
Minimum time in SEARCH before APPROACH can engage. Currently 0 (engage
on sight; failed-target skip handles bounce protection).
- Increase: rate-limits SEARCH→APPROACH transitions. Mostly legacy.

### APPROACH

#### `_approach_v = 30 * ROBOT_SPEED`
Forward speed during visual servo. Conservative so a late vision drop
doesn't slam the gripper into the box.
- Increase: faster grabs.
- Decrease: safer near-box dynamics.

#### `_approach_k_omega = 1.8`
Proportional gain on `bearing_rad` → `omega`. Larger means snappier
re-centering when the box drifts off the camera centerline.
- Increase: tighter visual tracking, may oscillate at high bearings.
- Decrease: smoother heading but slower to re-center.

#### `_approach_omega_max = 1.2 * ROBOT_SPEED` rad/s
Clamp on the visual-servo yaw command.
- Increase: catches boxes at sharper bearings.
- Decrease: reduces wheel slip → cleaner heading into CARRY_HOME.

#### `_dead_reckon_arrival_mm = 100`
When vision drops mid-APPROACH, dead-reckon toward the last latched
world target. Give up and fall back to SEARCH inside this radius.
- Increase: more tolerance for transient vision drops.
- Decrease: faster bail on lost targets.

#### `_approach_repulsion_gain = 50` (rad/s × mm)
Reactive obstacle avoidance inside the visual servo. For every
non-target colored box detected in front of the target on the visual
axis, omega gets a bias `gain / max(distance_mm, 100)` signed away
from the obstacle. At 250 mm this is 0.2 rad/s of avoidance bias on
top of the bearing P-term.
- Increase: stronger swerve around obstacles; risk of overshooting
  the target's bearing.
- Decrease: gentler avoidance, risk of brushing past blockers.
- Set to 0 to disable reactive avoidance (target will be chased
  straight; chain-push case will only be caught at the SEARCH
  obstacle-block filter).

#### `_approach_push_v = 20 mm/s`, `_approach_push_distance_mm = 30`, `_approach_push_timeout_s = 2.0`
Push-then-grip phase. On the first `grabbable` tick, instead of
immediately decelerating + closing the jaws, drive forward slowly
for `push_distance_mm` (or until `push_timeout_s`) so the target is
fully seated in the gripper before the close-servo fires. Avoids
the "jaws close on the front face" half-grip when the visual gate
triggers slightly short.
- Increase distance: deeper seating; risk of pushing the target out
  of the gripper if too long.
- Decrease distance: less push; risk of half-grip failures.
- Bump `push_v` for faster cycles, but slow keeps the target from
  squirting out under contact.

### CARRY_HOME drag guard

#### `_carry_target_lost_timeout_s = 10.0`
If `snap.captured` stays false for this many continuous seconds during
CARRY_HOME **and** vision is otherwise healthy (target colour absent
but at least one obstacle blob seen), the gripper releases whatever
it has — guards against carrying the wrong colored box if a swap
occurred mid-drag. Pure occlusion (camera sees no blobs at all) gets
infinite grace, since the gripper arms genuinely block the camera in
some configurations.
- Increase: more tolerance for partial occlusion (heavy arm shadows,
  slow target re-acquisition).
- Decrease: catches swaps faster, risk of false-release if the
  tracker takes a while to re-acquire after gripper close.

### RELEASE / TURN_AROUND

#### `_release_v = -40 mm/s` (backward)
Reverse speed during box release.
- Increase magnitude: clears the box from the rotation envelope faster.
- Decrease: avoids overshooting if there's an obstacle behind.

### Bench / lab modes

#### `_goto_v_max_mm_s = 60 * ROBOT_SPEED`, `_goto_margin_mm = 40`
GOTO uses a tighter margin than SEARCH (lab accuracy).
- Lower margin: more precise lab arrival; mission stalls in halt-and-
  localize if cox covariance can't shrink under the margin.

#### `_test_v_max_mm_s = 70 * ROBOT_SPEED`, `_test_margin_mm = 40`
Same shape, for the hardcoded TEST sequence.

---

## 5. Path-planning constants

In [src/controller.cpp](../src/controller.cpp):

### REALIGN in-place rotation ([State::TURN_AROUND tick body](../src/controller.cpp))
- `DELTA_DEADBAND_RAD = 0.05` (~3°): skip realign if already aligned.
- P-controller gain `2.0` (rad/s per rad of delta), `omega_cap = 1.0` rad/s.
- No forward motion, no spline — just a slow chassis rotation to
  `_home_theta`. SEARCH's bounded look-left/look-right sweep does the
  actual re-orientation work; this just gets the chassis pointed roughly
  back at home_theta before the sweep begins.
- Tighten the gain or cap if cox `inliers` drops during the spin.

### Generic spline (`bcubic_spline`) defaults
- `samples_per_segment = 20-100` — denser samples = smoother PurePursuit
  curvature but more memory / iteration cost.
- `default_v_max`, `accel_mm_s2`, `min_v_mm_s` — trap profile parameters
  set per-call by each planner.

---

## 6. Halt-and-localize ([src/controller.cpp:57-60](../src/controller.cpp#L57))

Used by GOTO / TEST to settle the KF before declaring arrival.

- `HALT_MIN_MS = 300` — minimum stand-still after PurePursuit reports
  arrived. Lets cox fold in at least one or two 100 ms scans.
- `HALT_MAX_MS = 700` — absolute cap; advance even if σ_xy hasn't
  settled.
- `HALT_SIGMA_XY_TARGET_MM = 30` — accept the halt once σ_xy drops
  below this.
- `MAHAL_GATE_CHI2 = 5.99` — χ²(2) 95% gate for "target inside
  post-halt covariance". Failure logs but still advances.
- Increase `HALT_MIN_MS` / decrease σ target: more accurate position
  fix, more total mission time.
- Decrease `HALT_MIN_MS`: faster leg cycling, risk of compounding
  position error.

---

## 7. Obstacle avoidance ([src/utils.cpp:17-31](../src/utils.cpp#L17))

- `OBSTACLE_DIST_MM = 230` — any lidar return inside ±60° forward arc
  at less than this triggers `obstacle_ahead` (zeroes `v_cmd`, keeps
  `w_cmd`). The gate is **suppressed** during APPROACH (visual servo
  + colour-based reactive omega handles avoidance; the lidar gate
  would zero `v` whenever the target itself entered the arc, stalling
  the grab) and during CARRY_HOME / RELEASE / TURN_AROUND (the closed
  gripper / held box sits inside the gate's forward arc). SEARCH keeps
  the gate as its safety net against unseen walls — see
  `StateMachine::obstacle_gate_suppressed()` in
  [src/controller.cpp](../src/controller.cpp).
- `OBSTACLE_ARC_HALF_DEG = 60` — width of the trigger cone.
- `GRIPPER_MASK_HALF_DEG = 20`, `GRIPPER_MASK_DIST_MM = 200` —
  narrower cone the closed gripper / held box occupies. Filtered out
  at the lidar layer (belt-and-suspenders alongside the state-based
  gate above; matters mostly for SEARCH where the gate is active).
- Increase `OBSTACLE_DIST_MM`: earlier stop, safer in cluttered arenas
  but stalls more often on near-miss walls.
- Decrease: more confident driving up to walls, risk of bumper-tap.
- Widen `OBSTACLE_ARC_HALF_DEG`: catches side-on obstacles, raises
  false-positive rate from the gripper cone.
- Widen `GRIPPER_MASK_*`: hides the held box more reliably from the
  obstacle gate. If too wide, real obstacles in that arc are ignored.

### Colour-based obstacle memory ([include/controller.hpp](../include/controller.hpp))

The vision tracker reports per-frame observations of any non-target
colour configured under `filter.obstacles.*`. The state machine
projects each observation to world coordinates and keeps a decaying
memory used by SEARCH (skip targets adjacent to a known obstacle)
and APPROACH (reactive omega bias).

- `_obstacle_merge_radius_mm = 200` — observations within this
  distance of an existing entry of the same colour are merged via EMA
  blend (α=0.35) instead of appended. Tighter = more sensitive to
  jitter; looser = entries drift faster as the obstacle moves.
- `_obstacle_forget_s = 30` — entries older than this are dropped on
  every tick. Decrease for genuinely dynamic obstacles; increase if
  the operator wants long-lived memory of static obstacles.
- `_obstacle_block_radius_mm = 250` — SEARCH skips any detected
  target inside this radius of a remembered obstacle (logged as
  `reason=obs_block`). Tune to the typical chain-push distance.

### Calibration ([src/calibrations.cpp](../src/calibrations.cpp), [main.cpp](../main.cpp))

App startup checks `filter.ready` in `config.yml`. If false / missing,
the calibration routine runs once for the target colour, then loops
asking the operator (`y/n` on stdin) whether to calibrate another
obstacle colour. Each `y` opens the same trackbar UI and writes the
result under `filter.obstacles.<N>`. On finish, sets
`filter.ready=true`, writes the config, and exits — restart to run
the mission.

To recalibrate later, edit `config.yml` and set `filter.ready: false`
(optionally also clear the existing `filter.target` / `filter.obstacles`
keys), then restart.

---

## 8. Cox scan matcher ([src/cox.cpp](../src/cox.cpp))

### Iteration controls
- `max_itr = 50` — max Cox iterations per scan.
- `tol_d_xy = 5.0 mm`, `tol_d_a = 2e-3 rad` — convergence deadband.
- Increase `max_itr`: more chances to converge on bad initial poses.
  Costs CPU per scan.
- Decrease tolerances: tighter fix, more iterations per scan.

### Gating
- `gate_k = 4.0` — Mahalanobis-style gate width on point-to-line
  associations (`gate = gate_k * sigma_r`, clamped by `MAX_GATE_MM`).
- `sigma_r = max(5, lidar_sensor_noise)` — beam noise sigma in mm.
  Loaded from runtime, floored at 5 mm.
- Increase `gate_k`: accepts more outliers, robust on noisy walls.
- Decrease: cleaner inliers, may reject good points if init is off.

### Fusion gates
- `MAX_SIGMA_XY_FUSE = 30 mm`, `MAX_SIGMA_THETA_FUSE = 0.025 rad`
  (~1.4°) — sigma ceilings above which a cox estimate is ignored by
  the KF. Anything worse is too noisy to fuse safely.
- `MAX_SIGMA_XY_FIX = 25 mm`, `MAX_SIGMA_THETA_FIX = 0.025 rad` —
  stricter gate for the cox-first-fix latch.
- Loosen: more cox updates fed into the KF (helps if cox is producing
  wide but useful estimates). Watch for KF drift from bad fixes.
- Tighten: cleaner KF, more frequent fallback to odom-only.

### KF-lost thresholds ([src/cox.cpp:379](../src/cox.cpp#L379))
- `KF_LOST_SIGMA_XY_MM = 200`, `KF_LOST_SIGMA_TH = 10°` — KF
  covariance above this lights the dashboard `kf_lost = yes` flag and
  triggers a relocalize sweep.
- Lower thresholds: relocalize more eagerly (costs ~1-2 s halted).
- Higher: less interruption, longer to recover from a real lost-pose.

---

## 9. Vision tracker ([config.yml + src/tracker.cpp:75](../src/tracker.cpp#L75))

### Camera (config.yml)
- `width = 640`, `height = 480`, `fps_target = 12` — capture resolution
  and frame rate.
- `focal_length_px = 650` — used for bearing/distance triangulation.
  Re-measure if optics change.

### Grab geometry
- `grab_bottom_region_ratio = 0.25` — bottom 25% of the frame is the
  "grab window" (the box must be within reach).
- `grab_bottom_fill_ratio = 0.65` — fraction of the grab window
  filled with the box color to declare `grabbable`.
- `grab_angle_tol_deg = 5.0` — bearing tolerance for the grab cone.
- `capture_distance_mm = 100`, `capture_bearing_tol_rad = 0.12` —
  distance/bearing gates that trigger `captured`.
- `retention_distance_mm = 150` — distance beyond which a `captured`
  latch is dropped.
- Tighten `grab_*`: fewer false grabs, more missed real grabs.
- Loosen: catches awkwardly-presented boxes, risks closing on empty air.

### Frame hysteresis ([src/tracker.cpp](../src/tracker.cpp))
- `CAPTURE_FRAMES = 10`, `RETENTION_FRAMES = 10`, `LOST_FRAMES = 10` —
  consecutive-frame counts to flip `captured` / `lost` flags. At 12 fps,
  each is ~0.8 s.
- `GRAB_FRAMES = 8`, `RELEASE_FRAMES = 8` — same idea for `grabbable`.
- Increase: more robust to single-frame noise, slower response.
- Decrease: snappier transitions, prone to vision flicker.

### Color filter (`filter.*` in config.yml)
- `dominant_color`, `lower1/2`, `upper1/2`, `has_wrap_around`,
  `ready` — HSV-range box-color filter.
- Re-train via the calibration tool when the lighting / box color
  changes. Not a runtime tuning knob — wrong values cripple the
  tracker.

---

## 10. RANSAC wall extractor ([config.yml `map.ransac`](../config.yml))

Runs once at boot from ~50 averaged lidar frames.

- `inlier_thresh_mm = 30` — point-to-line distance to count as inlier.
  Increase for noisy walls or curved surfaces; decrease for flat MDF.
- `max_iters_per_line = 500` — RANSAC trials per wall.
- `inlier_points_req = 18` — minimum inliers to accept a line.
  Lower if walls are heavily occluded.
- `min_inlier_fraction = 0.04` — fraction of remaining points that
  must support the line. Lower lets RANSAC find faint distant walls
  after the dominant ones are extracted.
- `max_lines = 6` — total walls to extract. Arena is 4 walls; +2
  slack for split walls (robot blocks midspan).
- `endpoint_padding_mm = 50` — slack added to inlier endpoints when
  forming the final segment.
- `min_segment_len_mm = 800` — reject segments shorter than this
  (filters out other robots and chassis edges; arena shortest wall is
  2400 mm).

Boot-only; CPU is free here. Errors propagate to every cox tick, so
verify the dashboard map preview after each tune.

---

## 11. Motor / chassis ([src/motor.cpp:43-44](../src/motor.cpp#L43))

Base accelerations (multiplied by `ROBOT_SPEED` at runtime):
- `ACCEL_V_BASE_MM_S2 = 150` — forward accel at `ROBOT_SPEED = 1`.
- `ACCEL_W_BASE_RAD_S2 = 1.5` — yaw accel at `ROBOT_SPEED = 1`.
- Increase: snappier response. Risk: wheel slip, encoder noise.
- Decrease: gentler ramps, more consistent odometry.

Calibration constants (don't tune — measure):
- `LEFT_TICKS_PER_MM = 121.04`, `RIGHT_TICKS_PER_MM = 118.42`,
  `WHEELBASE_MM = 175.0`.

---

## Common tuning recipes

**Bot too slow / mission overruns budget.**
Raise `robot.speed`. Watch the dashboard `kf_lost` flag — if it
flickers on tight turns, drop `robot.speed` back down or raise
`_omega_max` slightly. Do NOT raise `ACCEL_*` past 2x; the chassis
slips.

**Cox keeps losing during turns.**
Drop `_inplace_omega_cap` (PRE_ROLL/TERMINAL_ROTATE). Defaults
intentionally low.

**Bot stops dead in front of nothing.**
The obstacle gate fired. Either widen `GRIPPER_MASK_*` (your held
box is leaking into the obstacle cone) or shrink `OBSTACLE_DIST_MM`
(walls are scaring it). Check `obstacle_ahead` in the dashboard.

**Bot keeps grabbing at empty air.**
Tighten `grab_bottom_fill_ratio` and `grab_angle_tol_deg`. Verify
the color filter; recalibrate if lighting changed.

**APPROACH timeout loops forever on the same unreachable box.**
The failed-target skip should already handle this. If it doesn't:
shorten `_failed_target_skip_radius_mm` or lengthen
`_failed_target_skip_s` so the bot remembers the failure longer.
