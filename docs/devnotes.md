# Dev Notes — Localization Tuning

Reference for the arena map + Cox + EKF tuning that lives in the codebase
today. Update this when you change any of the constants below.

---

## Hard-coded arena map

The arena is modeled as four `LineSeg` structs ([include/types.hpp:35-38](../include/types.hpp#L35-L38)).
Each carries a unit normal `(nx, ny)`, a signed distance `r` from the origin
along that normal, and two endpoints. Cox computes the residual for a world
point P as `y = nx·Px + ny·Py − r` ([src/cox.cpp:45](../src/cox.cpp#L45)).
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

**Switching the map source.** Set `arena.in_arena: false` in `config.yml`
to fall back to the legacy RANSAC-built map. Useful when developing outside
the actual arena.

---

## Odometry noise: scale-error model

`src/odometry.cpp:59-85`. The old code used a random-walk form (`var = k·|d|`),
which gives σ ~ √d. Real wheel slip + radius miscalibration is a scale error
(σ ~ d), so squared variance must grow as d². Calibrated against observed
"report 600 mm, actually 610–620 mm" — that's ~2-3 % scale error on the
chassis distance.

| Source | Constant | Affects | At d = 600 mm |
|---|---|---|---|
| Per-wheel independent slip | `K_WHEEL_INDEP = 0.010` | both d and θ | σ_L = σ_R ≈ 6 mm |
| Common-mode (both wheels same slip) | `K_COMMON = 0.025` | only d | σ_d_common ≈ 15 mm |
| Rotation scale error | `K_ROT = 0.030` | only θ during a turn | σ_θ_rot at 90° turn ≈ 2.7° |

For a 600 mm straight run:
`σ_d = √(0.25·(36+36) + 15²) = √(18 + 225) ≈ 15.6 mm`
That matches the observed 10–20 mm range.

`σ_θ ≈ √((36+36)/185²) = 0.046 rad ≈ 2.6° / m` of forward travel — typical
drift for a diff-drive robot without an IMU. Cox will hammer it down once
running.

**Re-calibrating for a new floor.** Edit the three constants at the top of
[src/odometry.cpp:60-65](../src/odometry.cpp#L60-L65). The fastest direct
measurement is `K_COMMON`: drive 1 m straight, measure the actual distance
with a ruler.

```
K_COMMON = |encoder_reading - measured_truth| / encoder_reading
```

For `K_WHEEL_INDEP`, run several 1 m drives, compute the standard deviation
of the heading error at the end, divide by 1000 mm, multiply by the
wheelbase / √2. For `K_ROT`, command a 360° in-place spin a few times,
measure the angular error at the end, divide by 2π.

---

## Cox trust gate

`src/cox.cpp` `namespace` constants near the top of the file.

| Threshold | Before | After | Why |
|---|---|---|---|
| `MAX_SIGMA_XY` | 100 mm | **30 mm** | Analytical map gives much tighter Cox; 100 mm was for noisy RANSAC walls. |
| `MAX_SIGMA_THETA` | 0.087 rad (5°) | **0.035 rad (2°)** | Same reason. |
| `MIN_INLIERS` | 10 | 10 (unchanged) | Good floor for ~360-point scans. |

Side effect: a tighter gate = faster kidnap detection. A genuine shove now
needs `arena.kidnap_threshold` cycles (default 5, i.e. 0.5 s at 10 Hz) of
σ > 30 mm before the relocalize sweep kicks in.

---

## Kalman filter initial covariance

`src/kalman.cpp` `KalmanFilter::default_cov()`.

|   | Before | After | Why |
|---|---|---|---|
| σ_x = σ_y | 50 mm | **25 mm** | After home sanity-check passes we know we're within `arena.home_sanity_tol_mm` (50 mm by default); init at half that. |
| σ_θ | 5° | **2°** | Same logic against the 3° sanity tolerance. |

Initial cov doesn't change steady-state behaviour — Cox correction
overwhelms it within seconds either way. Tighter init just stops the first
few prediction-only ticks (before Cox's first trusted lock) from making the
EKF over-confident in nonsense.

---

## Things deliberately left alone

- **`sigma_r` inside Cox** ([src/cox.cpp:65](../src/cox.cpp#L65)) —
  `std::max(5, lidar_data.sensor_noise)`. The Welford calibration measurement
  is already accurate; the 5 mm floor protects against weird boot conditions.
- **Cox gate-shrinking schedule** ([src/cox.cpp:121-127](../src/cox.cpp#L121-L127)).
  Already well-tuned for tens of mm of initial error.
- **Ridge regularizer** (`RIDGE = 1e-3` in Cox). Important for the corridor
  degeneracy case, not worth touching.
- **`find_walls` RANSAC parameters** (`include/types.hpp:54-69` `RansacParams`).
  Still used in `in_arena=false` mode and as the validator's source of
  observed walls — leave alone.

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
/ `K_COMMON` / `K_ROT` to `config.yml` (under a new `odometry:` block).
That's about a 10-line change — wire them into `Odometry` via a `configure()`
setter, plumb the defaults from `main.cpp`.

Same applies to the Cox trust gate: candidates for `cox.*` keys in config
if we start tuning per-arena.
