# 06 — Coordinate Frames & Sign Conventions

This page collects every "which way is which" question in one place so
you don't have to re-derive it from the code.

## Frames

There are exactly two:

| Frame  | Origin                              | x axis            | y axis            |
|--------|-------------------------------------|-------------------|-------------------|
| **Lidar** (`P_lidar`) | the lidar's optical centre, at every instant | lidar's "forward" (the device's mechanical 0°) | 90° anticlockwise from x |
| **World** (`P_world`) | wherever the lidar was at the start of calibration | same as lidar's x at calibration time | same as lidar's y at calibration time |

The lidar frame moves with the device. The world frame is fixed by
calibration.

## Pose

`Pose2D = (x, y, θ)` is the lidar pose **expressed in the world frame**:

- `(x, y)` — position of the lidar's optical centre, in mm.
- `θ` — angle (rad) by which the lidar's x axis is rotated relative to
  the world's x axis. Positive θ = anticlockwise.

At calibration time, the lidar is at the world origin facing world-x:
`(0, 0, 0)`. As it moves, the matcher tracks how that frame relates to
the still-fixed world.

The transform from lidar frame to world frame is the standard rigid
motion:

```
   ┌ x_w ┐   ┌ cos θ   -sin θ ┐ ┌ x_s ┐   ┌ t_x ┐
   │     │ = │                │ │     │ + │     │
   └ y_w ┘   └ sin θ    cos θ ┘ └ y_s ┘   └ t_y ┘
```

In code (vectorized for all `N` scan points at once):

```c++
Eigen::Matrix2f R; R << c, -s, s, c;
P_world.leftCols(N).noalias() = R * P_lidar.leftCols(N);
P_world.row(0).head(N).array() += pose.x;
P_world.row(1).head(N).array() += pose.y;
```

## Polar to cartesian (lidar frame)

The driver gives us `(angle_deg, distance_mm)`. Lidar-frame cartesian:

```
   x_s = d · cos(angle · π / 180)
   y_s = d · sin(angle · π / 180)
```

The lidar's mechanical 0° is along x_s; angles increase
anticlockwise. This matches the right-hand rule for `(x_s, y_s, z_s)`
with z out of the scan plane.

Used in:

- [src/calibration.cpp](../src/calibration.cpp) when projecting bin
  averages to points for RANSAC.
- [src/estimate_position.cpp](../src/estimate_position.cpp) when filling
  `P_lidar_` once per estimate.

## Wall normals

Each `LineSeg` carries `(nx, ny)` — a unit normal — and `r` — the line's
signed perpendicular distance from the world origin along that normal.
The line equation is:

```
   n · p = r
```

where `p = (px, py)` is any world-frame point on the line.

**Convention**: `n` always points **away** from the sensor's calibration
origin. That is, `r > 0` strictly. Why:

- Residual sign becomes meaningful: `y_i = n · p_w − r > 0` means the scan
  ray ended past the wall (wall closer than expected); `y_i < 0` means
  short of the wall (wall farther).
- The Cox Jacobian uses `n` directly without sign tracking.

The flip is performed once at map-build time
([src/mapping.cpp](../src/mapping.cpp)):

```c++
double r = nx*centroid.x + ny*centroid.y;
if (r < 0.0) { nx = -nx; ny = -ny; r = -r; }
```

A sensor sitting **inside** a closed room sees every wall's centroid on
the outside, so `r > 0` is always achievable.

## Segment endpoints

`LineSeg::(ax, ay)` and `(bx, by)` are the two segment endpoints in
**world frame**. Their order (a vs b) is arbitrary — the matcher only
uses them to compute a parameter `t ∈ [0, 1]` along the segment:

```
   t_param = ((p.x - ax)·dx + (p.y - ay)·dy) / |segment|²
   where (dx, dy) = (bx - ax, by - ay)
```

A scan ray's projection onto the segment's line is "on the segment" iff
`t ∈ [-slack, 1 + slack]` where `slack = 50mm / |segment|`. Outside that
range, the ray is from a different physical wall and we move on.

## Angle wrapping

`Pose2D::theta` is wrapped to `(-π, π]` after every estimator solve:

```c++
while (pose.theta >  PI_F) pose.theta -= 2.0f * PI_F;
while (pose.theta <= -PI_F) pose.theta += 2.0f * PI_F;
```

For a 2D matcher running at 1 Hz on a slowly-moving robot this never
loops more than a few times. Print is in **degrees**:

```c++
est.pose.theta * 180.0f / PI_F
```

Same for the orientation uncertainty `σ_θ`.

## Visualizer wire format ([src/loging.cpp](../src/loging.cpp))

The Python visualizer expects angle in **1/100 degrees** (so it can use
integer math without losing 0.36° resolution at full revolution). We
multiply at the wire boundary:

```c++
payload.push_back(p.first * 100);   // angle in 1/100 deg
payload.push_back(p.second);        // distance mm
```

Internally everything is integer degrees. The conversion is one place
only — don't sprinkle the ×100 elsewhere.

## Cheat sheet

Quick reference, in order of "where you'd encounter it":

```
process_point inputs:    quality, angle_deg [0..359], distance_mm > 0
LidarDataFrame:          parallel arrays of angle_deg + distance_mm
P_lidar (Eigen 2 x N):   x_s = d·cos(a),  y_s = d·sin(a)
P_world (Eigen 2 x N):   p_w = R(θ)·p_s + (t_x, t_y)
LineSeg residual:        y = n·p_w - r            (sign meaningful)
Newton update:           pose += δ                 with δ from (JᵀJ) δ = -Jᵀy
Pose units:              x, y mm; θ rad in code, deg on print
σ units:                 σ_x, σ_y mm; σ_θ rad in code, deg on print
```
