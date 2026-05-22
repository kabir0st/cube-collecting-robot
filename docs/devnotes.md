# Dev Notes — Localization Tuning

Reference for the arena map + Cox + EKF tuning that lives in the codebase
today. Update this when you change any of the constants below.

---

## Hard-coded arena map

The arena is modeled as four `LineSeg` structs ([include/types.hpp:35-38](../include/types.hpp#L35-L38)).
Each carries a unit normal `(nx, ny)`, a signed distance `r` from the origin
along that normal, and two endpoints. Cox computes the residual for a world
point P as `y = nx·Px + ny·Py − r` ([src/cox.cpp:54](../src/cox.cpp#L54)).
With normals pointing **into** the arena, that residual is positive
everywhere inside.

**Frame.** Origin at the midpoint of the home wall. +x into the arena (along
the 3.6 m axis). +y to the robot's right. Robot starts at (0, 0, 0).

| Wall | Line | (nx, ny) | r | Endpoints |
|---|---|---|---|---|
| Back / home | x = 0 | (+1, 0) | 0 | (0, −1200) → (0, +1200) |
| Front | x = 3600 | (−1, 0) | −3600 | (3600, −1200) → (3600, +1200) |
| Right | y = +1200 | (0, −1) | −1200 | (0, +1200) → (3600, +1200) |
| Left | y = −1200 | (0, +1) | −1200 | (0, −1200) → (3600, −1200) |

Sanity check from the arena center (1800, 0):
- back: `1·1800 + 0·0 − 0 = 1800` ✓ (1.8 m from back wall)
- front: `−1·1800 + 0·0 − (−3600) = 1800` ✓
- right: `0·1800 + (−1)·0 − (−1200) = 1200` ✓
- left: `0·1800 + 1·0 − (−1200) = 1200` ✓

All residuals positive, magnitudes equal the perpendicular distance.

**Validator** ([src/calibrations.cpp](../src/calibrations.cpp), `validate_arena_walls`).
Accepts an observed RANSAC wall as a match if:
1. Its normal is parallel to an expected wall within `tol_deg`.
2. `r_obs == r_exp` after accounting for `(n, r) ≡ (−n, −r)` (RANSAC flips
   normals so r > 0; two expected walls have negative r).
3. Both observed endpoints lie within `tol_mm` of the expected segment's
   infinite line.

Extra observed walls that match nothing → fail. Missing expected walls →
OK (corner occlusion is normal).

**Switching the map source.** `arena.in_arena` in `config.yml` selects
between the analytical arena map (`true`) and a RANSAC-built map of
whatever room the robot is in (`false`). The flag *also* selects which
RANSAC preset is active: `arena.ransac_arena` (tight gates, ≤5 lines,
≥500 mm segments) is used inside the arena, `arena.ransac_room` (loose
gates, ≤10 lines, ≥800 mm segments to reject furniture edges) is used
outside. The repo currently ships with `in_arena: false` so lab-bench
runs work without code edits — flip it back to `true` before the actual
arena run.

---

## Odometry noise: scale-error model

`src/odometry.cpp:57-105`. The old code used a random-walk form (`var = k·|d|`),
which gives σ ~ √d. Real wheel slip + radius miscalibration is a scale error
(σ ~ d), so squared variance must grow as d². Calibrated against observed
"report 600 mm, actually 610–620 mm" — that's ~2-3 % scale error on the
chassis distance.

| Source | Constant | Affects | At d = 600 mm |
|---|---|---|---|
| Per-wheel independent slip | `K_WHEEL_INDEP = 0.020` | both d and θ | σ_L = σ_R ≈ 12 mm |
| Common-mode (both wheels same slip) | `K_COMMON = 0.025` | only d | σ_d_common ≈ 15 mm |
| Rotation scale error | `K_ROT = 0.030` | only θ during a turn | σ_θ_rot at 90° turn ≈ 2.7° |

For a 600 mm straight run:
`σ_d = √(0.25·(144+144) + 15²) = √(72 + 225) ≈ 17.2 mm`
That matches the observed 10–20 mm range.

`σ_θ ≈ √((144+144)/185²) = 0.092 rad ≈ 5.3° / m` of forward travel —
typical drift for a diff-drive robot without an IMU. Cox will hammer it
down once running.

**Re-calibrating for a new floor.** Edit the three constants near the top
of [src/odometry.cpp:75-79](../src/odometry.cpp#L75-L79). The fastest
direct measurement is `K_COMMON`: drive 1 m straight, measure the actual
distance with a ruler.

```
K_COMMON = |encoder_reading - measured_truth| / encoder_reading
```

For `K_WHEEL_INDEP`, run several 1 m drives, compute the standard deviation
of the heading error at the end, divide by 1000 mm, multiply by the
wheelbase / √2. For `K_ROT`, command a 360° in-place spin a few times,
measure the angular error at the end, divide by 2π.

---

## Cox trust gate

There are **two** gates with different jobs, defined at
[src/cox.cpp:239-255](../src/cox.cpp#L239-L255):

- `cox_can_fuse()` — loose floor. Used on every Cox→KF correction.
  Just checks that Cox converged and returned at least
  `MIN_INLIERS_FUSE = 6` inliers. There is no σ gate here: the KF's
  R-weighted gain already attenuates a noisy Cox automatically, so
  gatekeeping at this stage would only throw away useful information.
- `cox_strong_fix()` — strict gate. Used only to accept a relocalize
  sweep winner, because we're about to `kf.reset()` to it and need high
  confidence. Requires `inliers ≥ 15`, `σ_xy < 30 mm`, `σ_θ < 0.035 rad`
  (~2°).

| Threshold | Value | Used by |
|---|---|---|
| `MIN_INLIERS_FUSE` | 6 | `cox_can_fuse` — normal Cox→KF fuse |
| `MIN_INLIERS_FIX` | 15 | `cox_strong_fix` — accept relocalize sweep |
| `MAX_SIGMA_XY_FIX` | 30 mm | `cox_strong_fix` |
| `MAX_SIGMA_THETA_FIX` | 0.035 rad (≈2°) | `cox_strong_fix` |

The fix-gate thresholds are tight (analytical map gives much tighter Cox
than the old RANSAC walls did); tightening them also makes relocalize
candidates harder to fake, which is what we want when we're about to
overwrite KF state.

---

## Kalman filter initial covariance

`src/kalman.cpp` `KalmanFilter::default_cov()`.

|   | Before | After | Why |
|---|---|---|---|
| σ_x = σ_y | 50 mm | **25 mm** | Boot seed assumes the robot is at home; init at roughly half the home-placement tolerance the boot check enforces. |
| σ_θ | 5° | **2°** | Same logic against the boot heading tolerance. |

Initial cov doesn't change steady-state behaviour — Cox correction
overwhelms it within seconds either way. Tighter init just stops the first
few prediction-only ticks (before Cox's first trusted lock) from making the
EKF over-confident in nonsense. If the boot seed is wrong (robot wasn't
actually at home), the first untrusted Cox estimate triggers a relocalize
sweep that re-pins the KF with `moderate_recovery_cov()` — see
[src/kalman.cpp:28-38](../src/kalman.cpp#L28-L38).

---

## Things deliberately left alone

- **`sigma_r` inside Cox** ([src/cox.cpp:74](../src/cox.cpp#L74)) —
  `std::max(5, lidar_data.sensor_noise)`. The Welford calibration measurement
  is already accurate; the 5 mm floor protects against weird boot conditions.
- **Cox gate-shrinking schedule** ([src/cox.cpp:118-132](../src/cox.cpp#L118-L132)).
  Already well-tuned for tens of mm of initial error (start at ≥50 mm,
  shrink by 0.6× per iteration toward the σ-scaled final gate).
- **Ridge regularizer** (`RIDGE = 1e-3` in Cox). Important for the corridor
  degeneracy case, not worth touching.
- **`find_walls` RANSAC parameters** ([include/types.hpp:54-73](../include/types.hpp#L54-L73)
  `RansacParams`, populated from `arena.ransac_arena` / `arena.ransac_room`
  in `config.yml`). Still used in `in_arena=false` mode and as the
  validator's source of observed walls — tune via the YAML, not the
  struct.

---

## What can't be verified without a Pi run

Per the standing rule, builds happen on the Pi via `push_on_pi.sh`. After
the first push, two things to watch:

1. **Steady-state Cox σ.** Tail `logs/<run>_cox.txt` or the dashboard "Cox"
   panel. If σ_xy lands below ~3 mm we're golden. If it lands above 25 mm
   because of close-range chassis returns or odd reflections, bump
   `MAX_SIGMA_XY` back up to 50 mm.
2. **Validator false negatives.** RANSAC sometimes returns walls slightly
   outside the 100 mm tolerance because of chassis-return rays at near-zero
   range. If validation aborts spuriously, raise `arena.validation_tol_mm`
   to 150 — the underlying geometry is still right.

---

## Future hooks

If we end up tweaking the odometry constants often, expose `K_WHEEL_INDEP`
/ `K_COMMON` / `K_ROT` (currently at [src/odometry.cpp:75-77](../src/odometry.cpp#L75-L77))
to `config.yml` under a new `odometry:` block. That's about a 10-line
change — wire them into `Odometry` via a `configure()` setter, plumb the
defaults from `main.cpp`.

Same applies to the Cox trust gate: candidates for `cox.*` keys in config
if we start tuning per-arena.




+x = forward (into the arena, away from the home/back wall). Robot starts at x=0 facing +x.
+y = right (right wall sits at y = +half_w, left wall at y = -half_w).
+θ follows the standard right-hand rule around z, so positive θ rotates +x toward +y, i.e. clockwise when viewed from above (turn right = positive heading change).
This is a left-handed screen-style frame (x right, y down if you draw +x rightward on paper) but a right-handed robot frame in the usual "screen down = robot's right" sense. The dev notes (docs/devnotes.md:18) match: "+x into the arena (along the 3.6 m axis). +y to the robot's right."

Quick sanity for the snapshot you posted earlier:

KF Y_cm = 0.15, robot heading θ = 3.06° (slightly turned to the right). Mission goal (300, 0). So the robot is essentially on the centerline, nose pointing slightly right — consistent with the path-follower correcting back toward y=0.