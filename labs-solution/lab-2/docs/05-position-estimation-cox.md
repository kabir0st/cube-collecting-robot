# 05 — Position Estimation (Cox '91)

The runtime matcher takes a lidar scan and a known wall map and returns the
2D pose `(x, y, θ)` that best aligns the scan to the map. It is a direct
implementation of:

> Cox, I. J., *Blanche — An Experiment in Guidance and Navigation of an
> Autonomous Robot Vehicle.* IEEE Transactions on Robotics and Automation,
> 1991.

It lives in [src/estimate_position.cpp](../src/estimate_position.cpp).

## Setup

We have:

- `N` scan points in the lidar frame: `(x_s,i, y_s,i)`.
- `M` wall segments in the world frame, each with unit normal `n` and
  signed distance `r` (so `n·p = r` is the line equation).
- A current pose estimate `(t_x, t_y, θ)`, used as the linearisation point.

Goal: find the pose increment `δ = (δx, δy, δθ)` that minimises the
residuals of the scan after transforming each scan point to world frame:

```
   p_w,i = R(θ + δθ) · (x_s,i, y_s,i) + (t_x + δx, t_y + δy)
```

## Linearisation

We linearise about `δ = 0`. For a small `δθ`, `R(θ + δθ) ≈ R(θ) + R'(θ)·δθ`.
Carrying the algebra through, the world-frame residual against the
wall closest to `p_w,i` is:

```
   y_i = n · p_w,i − r
```

and its gradient with respect to the pose increment is:

```
   J_i = [ ∂y/∂δx,  ∂y/∂δy,  ∂y/∂δθ ]
       = [ n_x,    n_y,    -n_x·(p_w.y − t_y) + n_y·(p_w.x − t_x) ]
```

Stacked over all surviving scan points, we get the **normal equations**:

```
   (Jᵀ J) δ = − Jᵀ y
```

a 3×3 linear system. Solve, update the pose, repeat.

## The loop ([estimate_position.cpp](../src/estimate_position.cpp))

```c++
for (int iter = 0; iter < max_iters_; ++iter) {
    // 1. transform scan to world frame at current pose (vectorized)
    R << c, -s, s, c;
    P_world = R * P_lidar;
    P_world.row(0) += pose.x;
    P_world.row(1) += pose.y;

    // 2. accumulate JᵀJ and Jᵀy
    A.setZero(); b.setZero();
    for (int i = 0; i < N; ++i) {
        int w = nearest_wall(walls_, pwx, pwy, &y_i);
        if (w < 0) continue;
        if (|y_i| > gate) continue;            // residual gate (drop outliers)
        Eigen::Vector3f Ji(nx, ny, j3);
        A += Ji * Jiᵀ;                          // selfadjointView::rankUpdate
        b -= Ji * y_i;
        n_in++; ssq += y_i*y_i;
    }

    // 3. solve normal equations
    LDLT(A); delta = solve(b);
    pose += delta;
    if (||delta_xy|| < tol_dxy && |delta_theta| < tol_dtheta)
        break;                                  // converged
}
```

Three things make this fast on a Pi 3B:

1. **Pre-cartesianise once**: lidar-frame `(x_s, y_s)` is computed up
   front into a `2×N` Eigen matrix. Each iteration re-uses it; only the
   rotation+translation changes.
2. **Vectorized transform**: `Eigen` reduces `R · P_lidar` to a single
   SIMD-friendly multiply.
3. **No std::vector allocation in the hot path**: `A` and `b` are stack
   matrices; `P_lidar_` and `P_world_` are pre-allocated to 4096 columns.

## Wall picking — `nearest_wall`

For each transformed scan point we need the wall it's closest to. With
the rectangle map this was four walls; with RANSAC walls there can be
any reasonable number. We pick the wall whose perpendicular distance is
smallest **among the segments the projection falls within**:

```c++
const float L_len   = sqrt(L²);
const float t_param = ((pwx - L.ax)*dx + (pwy - L.ay)*dy) / L²;
const float slack   = ENDPOINT_SLACK_MM / L_len;
if (t_param < -slack || t_param > 1.0f + slack) continue;
```

Slack is **absolute** (50 mm in code), not a percentage. A 30 cm column
segment has the same overshoot tolerance as a 5 m wall. This is
important because RANSAC produces variable-length segments — a percentage
slack would make short segments practically untouchable.

`y_signed = n · p_w − r` is the signed perpendicular distance. We track
the wall with the smallest `|y_signed|` and return it.

## Residual gate

Before contributing to `JᵀJ` and `Jᵀy`, each residual is clipped:

```c++
gate = clamp(gate_k_ * sigma_r_, MIN_GATE_MM, MAX_GATE_MM);  // default k=3
if (|y_i| > gate) continue;
```

This is the matcher's outlier rejection: a scan point that lies more than
3σ from any wall is treated as an outlier (specular hit, off-map, dynamic
obstacle) and thrown out. The clamp range `[10, 150]` mm is a safety net
against degenerate `σ_r` (very tiny or very large).

## Covariance

Once converged, the parameter covariance is:

```
   Σ = (RSS / dof) · (Jᵀ J)⁻¹
   dof = max(1, n_in − 3)
```

This is the standard Gauss-Markov result for a least-squares estimator
under the Gaussian-residual assumption. `RSS = Σ y_i²`. Diagonal entries
are `σ_x²`, `σ_y²`, `σ_θ²`; the off-diagonals describe pose-coordinate
correlation but we don't print them.

## Pose update policy

We update `current_pose_` whenever `n_in ≥ 6`, regardless of whether the
Newton iteration formally converged. Reasoning:

- "Not converged" usually means the iteration cap (15) was hit while
  still making decreasing-but-not-yet-tight steps. The pose at that
  point is still much better than the previous estimate.
- Refusing to update on non-convergence creates a feedback loop: the
  next solve starts from the same stale pose, sees an even larger
  delta, and is even less likely to converge. Stale prior → bigger
  delta → worse linearisation → less likely to converge. We've all
  seen that movie.

A `n_in ≥ 6` threshold (twice the parameter count) is the minimum
needed for a meaningful covariance.

## Buffering — the rolling window

The lidar streams ~360 points every revolution at ~10 Hz. The matcher
runs at 1 Hz. Naively that gives us 10 revolutions of "fresh" data
each second, but the matcher only needs a few — `frames_window_ = 3` by
default. This:

- gives more inliers per estimate than a single frame,
- keeps us in the linearisation neighbourhood (3 frames ≈ 0.3 s of motion
  at most ~10 cm/s, which is well within the gate).

`process_point` splits the stream into `LidarDataFrame` chunks at each
revolution boundary (detected via `angle + 90 < last_angle_` — a wrap
from ~360 back to ~0). The deque holds the last 3 frames; `estimate()`
snapshots all three under the lock before solving.

## Threading

```
listener thread             main thread
─────────────────           ────────────────────────────
process_point()
  lock buf_mx_
    push to in_progress
    on wrap: deque rotate
  unlock buf_mx_
                            estimate()
                              lock buf_mx_
                                snapshot to local
                              unlock buf_mx_
                              ... 15 ms of math ...
                              update current_pose_
                              return
```

The mutex is held only long enough to copy the frame buffers. The math
runs on local copies so the listener thread is never blocked by the
solver.

`current_pose_` is read and written only from the main thread (inside
`estimate()`), so it's not under the mutex. If you ever need to read
`current()` from a different thread, wrap that read in a separate atomic
or mutex.

## Why not Kalman?

A Kalman / extended Kalman filter would let us blend Cox's per-iteration
estimate with a motion model (constant velocity, IMU prior, wheel
odometry). For this lab the robot moves slowly and there is no other
sensor — a 1 Hz Cox solve from a fresh pose is sufficient. If you add
wheel odometry, layering a small EKF on top of `PoseEstimate.cov` is the
natural next step; the matcher already returns the linearisation
covariance you'd plug into the update step.

## Performance notes

On the Pi 3B (Cortex-A53, four cores) one estimate of 1080 points
(3 frames × 360 each) and ~5 walls takes around 4–6 ms in `-O3` Release.
The dominant cost is the per-point `nearest_wall` loop, which is
unavoidable without spatial indexing. For `M = 5` walls and `N = 1000`
points we do `5 N = 5000` segment tests per iteration, ×15 iterations
worst case = 75 k tests — trivial for the platform. We have plenty of
budget left over.
