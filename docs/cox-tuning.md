# Cox scan-match tuning guide

Reference for every tunable constant in [src/cox.cpp](../src/cox.cpp). For
each knob: what it does, what happens when you raise or lower it, and when
you'd actually reach for it.

Cox is a point-to-line iterative scan matcher. It takes a seed pose (from
the Kalman filter), associates each lidar point with the nearest wall in
the reference map, builds a Gauss-Newton normal-equation system from the
perpendicular residuals, and solves for a pose correction. Repeat until the
step is small. The output (pose + covariance + inlier count + condition
number) is then either fused into the KF or used as a relocalize candidate.

The constants below control three things: **point-to-wall association**
(which lidar points count as inliers), **the solver itself** (convergence,
numerical stability), and **what we trust the result for** (fuse into KF,
or use it to reset the KF entirely).

---

## File-local gate constants

Defined in the anonymous namespace at the top of
[src/cox.cpp:21-32](../src/cox.cpp#L21-L32).

### `ENDPOINT_SLACK_MM = 50.0f` &nbsp; — &nbsp; [src/cox.cpp:23](../src/cox.cpp#L23)

Allows lidar points whose projection lands slightly past a wall segment's
endpoint to still be associated with that wall. Used inside
[`find_nearest_wall`](../src/cox.cpp#L36-L64) — the segment is effectively
extended by this much at each end.

- **Increase (e.g. 100 mm):** More corner / near-endpoint rays get included
  → more inliers near corners → better θ observability when the robot is
  mainly seeing corners. **Risk:** rays meant for a perpendicular wall can
  be wrongly associated with the wall they're projecting past, biasing the
  pose toward a phantom alignment.
- **Decrease (e.g. 10 mm):** Stricter "on-segment" test → cleaner per-wall
  associations. **Risk:** corner rays get dropped, inlier counts dip near
  corners, hurting convergence when the robot is *in* a corner.
- **Change when:** the map has many short segments or tight corners and
  you're losing inliers near them (raise); or you see corner-induced pose
  jitter from cross-association (lower).

### `MAX_GATE_MM = 150.0f` &nbsp; — &nbsp; [src/cox.cpp:24](../src/cox.cpp#L24)

Upper cap on the per-point residual gate. A scan point whose perpendicular
distance to its matched wall exceeds the gate is dropped as an outlier.

- **Increase (e.g. 250 mm):** Cold seeds (50-150 mm off true) keep
  converging because early iterations still capture correspondences.
  **Risk:** scan points from *real obstacles* (a chair, another robot) get
  matched to walls and drag the pose toward those false matches.
- **Decrease (e.g. 80 mm):** Tighter rejection of outliers → robust to
  clutter. **Risk:** a moderately bad seed can't find any inliers on
  iter 0; the solve aborts before it had a chance to pull itself in.
- **Change when:** the arena has dynamic obstacles and you see cox getting
  yanked by them (lower); or the KF frequently sends bad seeds and cox
  can't recover (raise — but the real fix is improving KF prediction).

### `MIN_GATE_MM = 8.0f` &nbsp; — &nbsp; [src/cox.cpp:25](../src/cox.cpp#L25)

Floor for the gate after geometric decay across iterations.

- **Increase (e.g. 20 mm):** Late iterations accept slightly noisier matches
  → better convergence in noisy scans. **Risk:** final-pose accuracy
  degrades because the solver never insists on tight residuals.
- **Decrease (e.g. 3 mm):** Tighter final fit. **Risk:** with lidar noise
  ~10-20 mm the gate excludes too many genuine inliers; `n_in < 6` breaks
  fire and the solve aborts.
- **Change when:** you've upgraded the lidar (lower noise → safe to lower
  this); or you see good early iterations followed by `n_in` collapsing
  late in the solve (raise).

### `LIDAR_MOUNT_OFFSET_F` &nbsp; — &nbsp; [src/cox.cpp:30-31](../src/cox.cpp#L30-L31)

`float` copy of the global `LIDAR_MOUNT_OFFSET_RAD` (from `global.hpp`).
Rotates lidar-frame points so robot-forward = +x. Shared with
`calibrations.cpp::find_walls` so the reference map and live scans use the
same body-frame transform.

- **Not a tuning knob** — it tracks physical assembly. If you remount the
  lidar at a different orientation, update `LIDAR_MOUNT_OFFSET_RAD` in
  `global.hpp` (this constant follows automatically). Getting it wrong
  rotates every scan by a fixed offset, so cox will either fail to
  converge anywhere or converge to a globally rotated pose.

---

## `CoxScanMatchAlgorithm` constructor members

Set in the initializer list at
[src/cox.cpp:68-84](../src/cox.cpp#L68-L84).

### `max_itr = 50` &nbsp; — &nbsp; [src/cox.cpp:74](../src/cox.cpp#L74)

Gauss-Newton iteration cap per `estimate()` call.

- **Increase (e.g. 100):** Cold seeds get more chances to converge.
  **Cost:** worst-case scan-match time roughly doubles → can blow the
  100 ms tick budget of `loop_scan_cox_estimation`.
- **Decrease (e.g. 20):** Faster worst case. **Risk:** marginally seeded
  scans return non-converged when they would have made it in 25-30 iters.
- **Change when:** profiling shows `loop_scan_cox_estimation` overrunning
  its tick (lower); or relocalize-sweep seeds consistently fail to
  converge from 100+ mm offsets (raise).

### `sigma_r` &nbsp; — &nbsp; [src/cox.cpp:76](../src/cox.cpp#L76)

Assumed 1σ range noise in mm. Pulled from
`lidar_data.sensor_noise.load()`, floored at 5 mm. Used to set the gate
(`gate_k * sigma_r`).

- **Increase the floor:** Wider gate → robust to noisier scans, but less
  aggressive outlier rejection.
- **Decrease the floor:** Tighter gate → demands a clean fit, can starve
  the solver of inliers.
- **Change when:** the 5 mm floor is wrong for your lidar. The floor
  exists so a sensor mis-reporting "0 noise" doesn't collapse the gate.

### `gate_k = 4.0f` &nbsp; — &nbsp; [src/cox.cpp:77](../src/cox.cpp#L77)

Multiplier on `sigma_r` for the nominal gate (then clamped to
`[MIN_GATE_MM, MAX_GATE_MM]`).

- **Increase (e.g. 6.0):** Effectively raises the gate proportional to the
  sensor's own reported noise.
- **Decrease (e.g. 2.5):** Stricter outlier rejection.
- **Change when:** you trust the sensor's `sensor_noise` value and want
  gating to scale with it. Practically the clamps to
  `[MIN_GATE_MM, MAX_GATE_MM]` dominate — usually leave `gate_k` alone and
  tune the clamps instead.

### `tol_d_xy = 3.0f` mm &nbsp; — &nbsp; [src/cox.cpp:80](../src/cox.cpp#L80)

Translation step size that counts as "converged."

- **Increase (e.g. 5 mm):** Earlier exit → fewer iters → cheaper. **Risk:**
  stops while the solve still has measurable bias.
- **Decrease (e.g. 1 mm):** Tighter final pose. **Risk:** with lidar noise
  ≥10 mm the residual surface is genuinely flat below ~2 mm step — you'll
  burn the iter budget chasing nothing and time out as non-converged.
- **Change when:** lidar noise drops significantly (lower); or convergence
  is rarely flagged true and downstream uses `converged` (raise modestly).

### `tol_d_a = 2.0e-3f` rad (~0.11°) &nbsp; — &nbsp; [src/cox.cpp:81](../src/cox.cpp#L81)

Rotation step size that counts as "converged." Same trade-off as
`tol_d_xy` but for θ.

- **Change when:** the robot's heading precision spec changes; otherwise
  leave.

### `pts_lidar`, `pts_world` &nbsp; — &nbsp; [src/cox.cpp:82-83](../src/cox.cpp#L82-L83)

Preallocated 2 × 4096 Eigen matrices for lidar-frame and world-frame point
clouds. Not a tuning knob — sized to cover any lidar frame we receive.
They auto-resize at [src/cox.cpp:103-106](../src/cox.cpp#L103-L106) if a
larger frame ever arrives, but the 4096 default avoids per-tick
allocation. Only revisit if you switch to a much higher-resolution lidar.

---

## `estimate()` internals

### `start_gate` and `final_gate` &nbsp; — &nbsp; [src/cox.cpp:122-128](../src/cox.cpp#L122-L128)

Per-call gate schedule. `final_gate = clamp(gate_k * sigma_r, MIN, MAX)`.
`start_gate = clamp(final_gate, 50, MAX_GATE_MM)`.

- **Increase `start_gate`:** Better at absorbing cold seeds.
- **Decrease `start_gate`:** Cleaner early iterations but unforgiving to
  bad seeds.
- **Change when:** changing `MAX_GATE_MM` — they interact, `start_gate` is
  bounded by it.

### Gate decay factor `0.75` &nbsp; — &nbsp; [src/cox.cpp:138](../src/cox.cpp#L138)

How fast the gate shrinks from `start_gate` toward `final_gate`:
`gate = max(final_gate, start_gate * 0.75^iter)`.

- **Increase (e.g. 0.85 — slower decay):** Gate stays wide longer → more
  iterations to find the right basin. **Risk:** late iterations are still
  pulled by outliers.
- **Decrease (e.g. 0.5 — faster decay):** Quickly drops to precision mode.
  **Risk:** cold seed never sees enough correspondences to pull in before
  the gate slams shut.
- **Change when:** cold seeds (post-recovery, after a shove) consistently
  fail (raise); or clear inlier contamination late in the solve (lower).

### `RIDGE = 1e-3f` &nbsp; — &nbsp; [src/cox.cpp:187](../src/cox.cpp#L187)

Tikhonov regularizer added to `A`'s diagonal before the LDLT solve. Stops
the LDLT/inverse from blowing up on rank-deficient geometry — i.e. a long
corridor or a single visible wall orientation.

- **Increase (e.g. 1e-1):** Stronger numerical stability on rank-deficient
  geometry. **Risk:** *biases* well-constrained solves; the `cond_number`
  floor goes up, so `cox_can_fuse` may start rejecting genuinely
  well-observed scans.
- **Decrease (e.g. 1e-5):** Less bias on good scans. **Risk:** in
  single-wall geometry the LDLT either fails or returns covariance with
  Inf/NaN entries → cox produces garbage that may slip past the fuse gate.
- **Change when:** condition numbers on known-good scans are creeping
  above `MAX_COND_FUSE` (lower the ridge); or you see NaN sigmas in the
  dashboard (raise).

### Output: `cov`, `sigma_x/y/a`, `cond_number` &nbsp; — &nbsp; [src/cox.cpp:216-242](../src/cox.cpp#L216-L242)

Not tuning knobs — derived outputs:
- `cov = s2 * A.inverse()` is the 3 × 3 pose covariance, with
  `s2 = ssq / max(1, n_in - 3)` being the residual variance estimate.
- `sigma_x/y/a` are the square roots of the covariance diagonals.
- `cond_number = λ_max / λ_min` of `A`. Long-corridor / single-wall scans
  blow this up, which `cox_can_fuse` uses to refuse the estimate.

---

## Fusion gate constants

Defined at [src/cox.cpp:262-273](../src/cox.cpp#L262-L273).

These come in two sets: **fuse gates** (loose, for blending cox into the
KF via `kf.correct()`) and **strong-fix gates** (strict, for accepting a
relocalize sweep winner which triggers `kf.reset()`). The asymmetry is
intentional — fusing a noisy estimate is cheap because its large `R` makes
the KF correction weak, but resetting the KF blows away prior state, so we
demand high confidence.

### `MIN_INLIERS_FUSE = 30` &nbsp; — &nbsp; [src/cox.cpp:262](../src/cox.cpp#L262)

Minimum inliers required to fuse cox into the KF.

- **Increase (e.g. 60):** Only confident estimates touch the KF → KF stays
  clean. **Risk:** in lidar-sparse moments (heavy occlusion, far walls)
  you go long stretches dead-reckoning → KF cov balloons →
  false-triggered relocalize.
- **Decrease (e.g. 15):** More fusion opportunities. **Risk:** weak
  estimates with large cov-derived `R` still get into the KF; if `R` is
  wrongly *small* the KF state moves on bad data.
- **Change when:** relocalize triggers too often (lower); or you see KF
  jumps after weak cox fixes (raise).

### `MAX_COND_FUSE = 1.0e5f` &nbsp; — &nbsp; [src/cox.cpp:268](../src/cox.cpp#L268)

Condition-number cap for fusing. Well-observed scans are typically
~1e2-1e3; long-corridor / single-wall scans blow past 1e6.

- **Increase (e.g. 1e6):** Long-corridor scans squeak through. **Risk:**
  their pose along the unobserved axis is essentially noise; even with a
  large `R`, KF state can drift along that axis.
- **Decrease (e.g. 1e4):** Stricter geometric observability requirement.
  **Risk:** marginal-but-useful scans are dropped → more dead-reckoning
  between fuses.
- **Change when:** the arena has long corridors and you want to be safer
  about along-axis drift (lower); or the lidar's seeing limited wall
  variety but the matches that come through are good (raise).

### `MIN_INLIERS_FIX = 30` &nbsp; — &nbsp; [src/cox.cpp:270](../src/cox.cpp#L270)

Minimum inliers for a relocalize-sweep candidate to be accepted as a
strong fix.

- Same trade-off as `MIN_INLIERS_FUSE` but stricter consequences: a strong
  fix triggers `kf.reset()`.
- **Change when:** strong fixes are accepted too readily and reset to bad
  poses (raise); or relocalize never succeeds (lower).

### `MAX_SIGMA_XY_FIX = 30.0f` mm &nbsp; — &nbsp; [src/cox.cpp:271](../src/cox.cpp#L271)

Maximum allowed translation 1σ for a strong fix (~3 cm).

- **Increase (e.g. 60 mm):** Relocalize succeeds in more sweeps. **Risk:**
  KF gets reset to a pose with several cm of uncertainty as if it were
  trusted.
- **Decrease (e.g. 15 mm):** Safer resets. **Risk:** sweep often finds no
  strong fix → robot stays halted with motors down for another cycle.
- **Change when:** the robot frequently gets stuck unable to recover after
  a shove (loosen); or relocalize occasionally resets to a clearly-wrong
  pose (tighten).

### `MAX_SIGMA_THETA_FIX = 0.035f` rad (~2°) &nbsp; — &nbsp; [src/cox.cpp:272](../src/cox.cpp#L272)

Maximum allowed heading 1σ for a strong fix. Same trade-off as
`MAX_SIGMA_XY_FIX` but for θ. Tighten if you see the robot resetting to a
visibly rotated pose; loosen if relocalize never finds a winner with
good xy but a noisy θ.

### `MAX_COND_FIX = 1.0e4f` &nbsp; — &nbsp; [src/cox.cpp:273](../src/cox.cpp#L273)

Condition-number cap for a strong fix. One order of magnitude tighter than
`MAX_COND_FUSE` because a reset commits us to this pose.

- **Change when:** strong fixes accept rank-deficient solutions and pin
  the KF along an unobservable axis (lower); or arena geometry is
  genuinely tight and good fixes have higher cond than usual (raise).

---

## Relocalize sweep window

Defined inside `relocalize_local_sweep` at
[src/cox.cpp:300-348](../src/cox.cpp#L300-L348). The sweep grid is
**N(xy)² × N(θ) cox runs per sweep**, so window/step changes scale cubically.

### `HALF_XY_MM = 300.0` &nbsp; — &nbsp; [src/cox.cpp:306](../src/cox.cpp#L306)

Half-width of the xy search box centered on the current KF pose.

- **Increase (e.g. 500 mm):** Recovers from bigger shoves. **Cost:** grid
  count grows as (HALF / step)² — at 500/100 that's 11 × 11 = 121 xy
  seeds vs the default 7 × 7 = 49, plus the θ axis on top. Sweep time
  grows accordingly.
- **Decrease:** Faster sweep. **Risk:** large shoves push the true pose
  outside the box → recovery impossible without a new mechanism.

### `GRID_MM = 100.0` &nbsp; — &nbsp; [src/cox.cpp:307](../src/cox.cpp#L307)

xy grid spacing. Picked to fit inside cox's basin of convergence
(~100 mm / ~15° on this arena).

- **Decrease (e.g. 50 mm):** Finer seeds → less chance of skipping a
  basin. **Cost:** N(xy)² blows up; halving step quadruples xy seed
  count.
- **Increase (e.g. 150 mm):** Faster sweep. **Risk:** can step *over* the
  basin of convergence and miss the true pose entirely.

### `HALF_THETA = π/4` (±45°) &nbsp; — &nbsp; [src/cox.cpp:308](../src/cox.cpp#L308)

Half-width of the θ search range.

- **Increase:** Recovers from larger rotational kicks. **Cost:** linear in
  seed count on the θ axis.
- **Decrease:** Faster sweep, narrower recovery window.

### `GRID_RAD = 15°` &nbsp; — &nbsp; [src/cox.cpp:309](../src/cox.cpp#L309)

θ grid spacing.

- **Decrease (e.g. 7.5°):** Finer θ seeds. **Cost:** doubles θ seed count.
- **Increase (e.g. 22°):** Faster sweep. **Risk:** steps over the basin.

**Change all four together when:** profiling shows sweeps take much more
than ~1-2 s (raise step or shrink window); or the robot recovers but
into the wrong basin (shrink step).

---

## `loop_scan_cox_estimation` constants

Defined in the function body at
[src/cox.cpp:361-371](../src/cox.cpp#L361-L371).

### `KF_LOST_SIGMA_XY_MM = 300.0` &nbsp; — &nbsp; [src/cox.cpp:369](../src/cox.cpp#L369)

KF self-covariance threshold (xy 1σ in mm) that declares the robot "lost"
and triggers a sweep.

- **Increase (e.g. 500 mm):** Robot stays in normal operation longer
  despite cov growth. **Risk:** drives blind for longer; once it does
  trigger, the pose may be too far for the sweep window to recover.
- **Decrease (e.g. 150 mm):** Faster trigger on degraded localization.
  **Risk:** brief cox dropouts (one obstructed scan) false-trigger a
  halt-and-sweep → robot stutters unnecessarily.

### `KF_LOST_SIGMA_TH = 15°` &nbsp; — &nbsp; [src/cox.cpp:370](../src/cox.cpp#L370)

Same trigger but on heading 1σ.

**Change when:** the robot halts too readily during normal driving (raise
both); or it keeps driving into walls after losing track (lower both).

---

## `moderate_recovery_cov()`

Defined at [src/cox.cpp:350-357](../src/cox.cpp#L350-L357). Returns the
initial covariance handed to `kf.reset()` after a successful relocalize:
100 mm xy 1σ, 5° heading 1σ.

- **Increase:** KF treats the recovered pose as uncertain → next few cox
  fuses will pull it strongly. Safer if relocalize might be a bit off.
- **Decrease:** KF treats the reset as confident → it'll resist subsequent
  cox corrections. Risky if the strong-fix gates were ever wrong.
- **Change when:** you tighten or loosen the strong-fix gates — these
  should track each other. Stricter strong-fix gates → safe to lower
  this; looser → raise it.

---

## Quick "what to tune first" cheat sheet

| Symptom | First knob to try |
|---|---|
| Cox jitter, pose noisy | Lower `MAX_GATE_MM`, raise `MIN_INLIERS_FUSE` |
| Relocalize triggers too often | Raise `KF_LOST_SIGMA_XY_MM`, lower `MIN_INLIERS_FUSE` |
| Recovery from shove fails | Raise `HALF_XY_MM` / `HALF_THETA`, loosen `MAX_SIGMA_*_FIX` |
| NaN in sigmas / cond_number | Raise `RIDGE` |
| Long-corridor pose drifts along corridor | Lower `MAX_COND_FUSE` |
| Convergence rarely reaches `true` | Raise `tol_d_xy` to 5 mm |
| Loop misses tick deadline | Lower `max_itr`, raise sweep grid step |
| Corner rays getting dropped | Raise `ENDPOINT_SLACK_MM` |
| KF refuses subsequent corrections after recovery | Raise covariance in `moderate_recovery_cov` |
