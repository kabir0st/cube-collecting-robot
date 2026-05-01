# Lab 2 — Lidar Localization Inside a Box (RPi 3B)

Single-sensor localization. An RPLIDAR A1 reports range scans over USB; the program estimates the lidar's pose `(x, y, θ)` inside a known rectangular box and prints it with uncertainty `(σ_x, σ_y, σ_θ)` once per second.

The pipeline has two halves:

1. **Calibration** (first ~3.5 s, lidar stationary) — measures the lidar noise model `σ_r` and extracts a line map of the box from the averaged scan. The line map is the reference geometry used by the matcher.
2. **Cox scan matching** (1 Hz, runtime) — aligns each new scan to the line map by least squares, returning pose and covariance.

---

## Hardware

- Raspberry Pi 3B (Linux, gcc/g++ with C++11).
- RPLIDAR A1 over USB. The vendor driver is launched separately and feeds packets to this program over a local TCP socket on port `9888` (data) and `9887` (control).
- A four-walled box. Walls should be reasonably flat and high enough that the beam plane stays inside the box.

---

## Build

```
make
```

Produces `./app`. The Makefile compiles `main.cpp` and everything under `src/*.cpp` with `-std=c++11 -O2 -Wall -Wextra -pthread` and links against OpenCV.

```
make clean       # remove ./app and build/
make info        # show resolved flags
```

---

## Run

```
./app                 # uses cached box_map.txt if present
./app --recalibrate   # forces calibration phase, overwrites box_map.txt
```

Place the lidar stationary in the box before pressing enter. On `--recalibrate`, the program will refuse to start the matcher until calibration produces a sane 4-segment map.

---

## Console output

```
[info ] Starting Data Listen Server . .
[info ] Waiting for lidar client...
[info ] Lidar client connected
[calib] frame 12/35  σ_r=8.4mm  beams=358
[calib] line-map: 4 segments, RMS fit=6.1mm  (saved to box_map.txt)
[pose ] X=  412.3 mm   Y=  287.9 mm   θ=   3.21°    σx=4.2  σy=5.1  σθ=0.18°
[pose ] X=  413.0 mm   Y=  288.1 mm   θ=   3.19°    σx=4.1  σy=5.0  σθ=0.17°
...
```

`[pose ]` lines are emitted at ~1 Hz. `X, Y, σ_x, σ_y` are in millimetres; `θ, σ_θ` in degrees.

---

## Calibration phase — detailed guide

**Goal.** Estimate the lidar's noise model `(σ_r, σ_θ)` and build the reference line map of the box. Both are required by the Cox matcher.

### Procedure

1. Place the lidar **stationary** roughly in the centre of the box, oriented so the 0° beam points along the box's long axis (this fixes the world frame; the centre and orientation define `(0, 0, 0)`).
2. Run `./app --recalibrate`. The program records `N_CAL = 35` frames at 10 Hz (~3.5 s). Each frame is a full 360° scan.
3. **Per-bin statistics.** For each integer-degree angular bin `θ ∈ [0, 360)`:
   - Collect every distance reading that falls in that bin across all frames.
   - Compute `μ_r(θ)` = mean and `σ²_r(θ)` = sample variance.
   - Reject bins with fewer than `N_CAL/2` samples — those are dropped or occluded beams and would skew the noise estimate.
4. **Aggregate.** Define a single scalar noise level
   ```
   σ_r = median over accepted bins of √σ²_r(θ)
   ```
   The median (rather than mean) makes σ_r robust to bins where the beam grazes a wall obliquely — those bins legitimately have higher variance from surface roughness, but they are not representative of the lidar's intrinsic ranging noise.
5. `σ_θ` is left at the RPLIDAR A1 nominal of `0.5°`. We don't try to estimate it from data because angular jitter is dominated by the encoder, not the range, and 35 frames is too few to separate the two.
6. **Build the line map.** Convert the mean scan `μ_r(θ)` into a Cartesian point cloud `{p_i = (μ_r(θ_i)·cos θ_i, μ_r(θ_i)·sin θ_i)}`. Run **split-and-merge**:

   - Start with all points as one segment.
   - Find the point of maximum perpendicular distance `d` to the line through the segment's endpoints.
   - If `d > LINE_TOL = 15 mm`, split there and recurse on both halves.
   - When recursion stops, fit a least-squares line to each surviving segment.
   - Merge adjacent segments whose normal angles differ by `< 2°` and whose endpoint gap is `< 50 mm`.

7. **Store each segment** as `(n_x, n_y, r, p_start, p_end)` where `n` is the unit normal pointing **away from the lidar position** and `r = n · midpoint`. Endpoints are kept so the matcher can reject points that project past the wall.
8. **Persist.** Write to `box_map.txt`. Print summary: number of segments, RMS perpendicular fit error.

### Sanity gate

A rectangular box must produce 4 segments roughly perpendicular in pairs. The calibrator aborts with a hint if:
- fewer than 4 segments are found (`LINE_TOL` too tight, or lidar saw clutter);
- segment normals are not orthogonal in pairs to within 5° (lidar is rotated badly, or box is not rectangular);
- RMS fit error exceeds 20 mm (walls are not flat / lidar is wobbling).

### Output file format (`box_map.txt`)

One segment per line:
```
nx  ny  r  start_x  start_y  end_x  end_y
```
All values in millimetres. Comments with `#` are allowed.

---

## Cox scan matching — detailed guide

**Goal.** Given a line map `L = {(n_j, r_j, seg_j)}` and a fresh scan `{p_i}` in the lidar frame, find the pose `(x, y, θ)` that best aligns the scan to the map, plus a covariance for that pose.

### Geometric setup

A scan point `p_i` in the lidar frame is transformed to the world frame by
```
P_i(x, y, θ) = R(θ)·p_i + t,    t = (x, y)ᵀ,
```
where `R(θ)` is the standard 2D rotation matrix. For a target line `j` with unit normal `n_j` and signed offset `r_j`, the **signed perpendicular residual** is
```
y_i  =  n_j · P_i  -  r_j.
```
A perfectly localized scan would have `y_i = 0` for every inlier point.

### Linearization

The unknown rotation makes the equation nonlinear in `θ`. Linearize around the current estimate `θ̂`:
```
R(θ̂ + Δθ)  ≈  R(θ̂)  +  Δθ · J · R(θ̂),     J = [[0, -1], [1, 0]].
```
Substituting into `y_i` and expanding,
```
y_i  ≈  [ n_j · R(θ̂)·p_i  -  r_j ]   +   n_j · Δt   +   Δθ · n_j · J · R(θ̂)·p_i
        └────────── c_i (constant) ──────┘   └──────────── A_i · Δx ────────────┘
```
Stacking over all inlier points gives the linear system
```
A · Δx  =  -c,        Δx = (Δt_x, Δt_y, Δθ)ᵀ,
A_i     =  [ n_jx,  n_jy,  n_j · J · R(θ̂)·p_i ]   (1×3 row).
```

### Solve

Normal equations, solved directly because the system is 3×3:
```
Δx  =  (Aᵀ A)⁻¹ Aᵀ (-c).
```
Update the pose `t ← t + Δt`, `θ ← θ + Δθ`. Re-associate every point to its (now possibly different) nearest line, recompute `A` and `c`, and iterate until `‖Δx‖ < ε` or 10 iterations elapse. In practice 4–6 iterations suffice.

### Outlier gating

Drop point `i` from the system if either holds:
- `|y_i| > 3·σ_r` — the point is statistically inconsistent with the associated line;
- the perpendicular projection of `P_i` onto line `j` falls outside the stored segment endpoints — the point is "past" the wall and should not be matched to it.

Gating is re-applied on every iteration so a point can be re-included once the pose updates have brought it back near a valid wall.

### Uncertainty (the σ outputs)

Let `N` be the number of inliers in the final iteration. The residual variance is
```
σ̂²  =  (Σ y_i²) / (N − 3).
```
The pose covariance is
```
Σ_pose  =  σ̂² · (Aᵀ A)⁻¹.
```
Reported uncertainties are the standard deviations along each axis,
```
σ_x  =  √Σ_pose[0, 0],
σ_y  =  √Σ_pose[1, 1],
σ_θ  =  √Σ_pose[2, 2]   (printed in degrees).
```
This couples nicely with the calibration: `σ_r` from calibration sets the scale of the residuals, and `σ̂²` recovered at runtime should be close to `σ_r²` when the map is accurate. A persistently inflated `σ̂²` indicates a stale or wrong map.

### Initialization each tick

Seed Cox with the **previous tick's pose**. On the very first tick after calibration, seed with `(0, 0, 0)` — the lidar is by definition at the origin in box-centred coordinates, having not moved since the calibration scan.

### Failure modes & responses

| Symptom | Cause | Response |
|---|---|---|
| `AᵀA` near-singular | Only parallel walls visible (open box, partial occlusion) | Hold previous pose, inflate `σ` proportionally to the smallest singular value |
| All points gated out | Robot moved out of box, or map is wrong | Print warning, hold pose, do not update |
| `‖Δx‖` not converging in 10 iterations | Bad point-to-line associations bouncing | Restart Cox with `(0, 0, 0)` seed and only one association update |

---

## Tuning knobs

| Constant | Default | Where | Effect |
|---|---|---|---|
| `N_CAL` | 35 | calibration | Length of calibration phase. Higher = better σ_r, longer wait. |
| `LINE_TOL` | 15 mm | line extraction | Split threshold. Lower = more segments, noisier; higher = misses small walls. |
| `SIGMA_GATE` | 3.0 | Cox | Outlier rejection in units of σ_r. Lower = more rejections. |
| `MAX_ITERS` | 10 | Cox | Hard cap on iterations per tick. |
| `EPS_CONVERGE` | 0.5 mm / 0.05° | Cox | Early-stop threshold on `‖Δx‖`. |

---

## Troubleshooting

- **"failed to bind lidar stream socket"** — port 9888 is in use; either kill the previous run (`pkill -f ./app`) or restart the lidar driver.
- **Lidar driver connected but no packets** — check `0xA5` sync bytes; the receiver resyncs on mismatch but a fully misaligned start will look like silence.
- **Calibration aborts with "fewer than 4 segments"** — `LINE_TOL` is too tight for the box surface roughness, or the lidar saw a person/clutter. Re-run with the box clear; loosen `LINE_TOL` if walls are textured.
- **σ_θ stays unusually high (> 1°)** — there were dynamic objects during calibration, or the box has a near-symmetric layout that makes orientation poorly observable.
- **Pose drifts when stationary** — `box_map.txt` is stale; force `--recalibrate`.
