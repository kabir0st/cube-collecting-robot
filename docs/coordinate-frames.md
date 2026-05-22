# Coordinate frames & sign conventions

Reference doc for the math convention used end-to-end: lidar, find_walls, Cox,
Kalman filter, odometry, motor IK, camera bearing, pure pursuit, test
waypoints. Everything in the codebase is internally consistent with the
convention below; the only items that depend on physical assembly are
called out in [Hardware-dependent assumptions](#hardware-dependent-assumptions).

## The convention

```
                                  +y  (LEFT)
                                   ▲
                                   │
                                   │
                                   │
                  ┌────────────────┼────────────────┐
                  │                │                │
                  │             ┌──┴──┐             │
                  │             │ CAM │             │
                  │             └──┬──┘  ──────────────────►  +x  (FORWARD, θ = 0)
                  │             ┌──┴──┐             │
                  │             │LIDAR│             │
                  │             └─────┘             │
                  │                                 │
                  │      ●─────── BASE ───────●     │
                  │   L wheel               R wheel │
                  └────────────────┬────────────────┘
                                   │
                                   │
                                   ▼
                                  -y  (RIGHT)


                              +θ rotation
                                 ╭───╮
                                 │   ▼  CCW (counter-clockwise from above)
                                 ╰───╯
```

- **+x = forward** when the robot is at heading θ = 0
- **+y = left** (90° CCW from +x — standard math frame, NOT screen-y)
- **+θ = CCW rotation** (right-hand rule with z pointing up out of the page)
- Angles are in radians everywhere except where suffixed `_deg` for I/O
- `wrap_pi(a)` in [include/utils.hpp](../include/utils.hpp) wraps to `(-π, π]`

## Diff-drive sign conventions

Diff-drive IK ([src/motor.cpp:168-169](../src/motor.cpp#L168-L169)):

```
  v_left  = v − ω · (WHEELBASE / 2)
  v_right = v + ω · (WHEELBASE / 2)
```

- `v > 0` ⇒ both wheels forward ⇒ robot drives +x in body frame
- `ω > 0` ⇒ right wheel faster than left ⇒ robot rotates **CCW**

The right wheel is **wired backwards on the chassis**. Two matched sign-flips
in code cancel that out so the abstraction stays clean:

- Command: `set_r = − mm_s_to_pps100(v + ω·b/2)` ([motor.cpp:169](../src/motor.cpp#L169))
- Readback: `r_ticks = − _enc_right_acc` ([motor.cpp:138](../src/motor.cpp#L138))

After both flips, `get_encoders()` returns positive `right_mm` when the robot
physically moves forward, and `set_velocity(+v, 0)` drives it forward.

## LIDAR mount

The 2D lidar scans **clockwise** in its own polar frame (angle_deg increases
CW). The body-frame transform — used identically by both Cox per-tick
([src/cox.cpp:108-110](../src/cox.cpp#L108-L110)) and the boot-time map
builder ([include/utils.hpp:45-50](../include/utils.hpp#L45-L50)) — is:

```
  a   = − angle_deg · π/180 + LIDAR_MOUNT_OFFSET_RAD
  x   = dist · cos(a)
  y   = dist · sin(a)
```

The `− angle_deg` flips CW→CCW. `LIDAR_MOUNT_OFFSET_RAD`
([include/global.hpp:14](../include/global.hpp#L14)) is the CCW angle from
the robot's +x axis to the lidar's raw 0° beam:

| Offset value | Meaning |
|---|---|
| `0` | lidar 0° already points forward |
| `−π/2` *(current)* | lidar 0° points to the robot's **right** (mounted 90° CW) |
| `+π/2` | lidar 0° points to the robot's **left** (mounted 90° CCW) |
| `π` | lidar 0° points backward |

The single source of truth lives in one constant. Same transform is used by
both find_walls and runtime Cox, so the reference map and the per-tick scans
are guaranteed to live in the same body frame — no drift possible.

## Kalman filter / pose state

`Pos2D = (x_mm, y_mm, theta_rad)`, θ wrapped to `(−π, π]` by the EKF.

Predict ([src/kalman.cpp:62-64](../src/kalman.cpp#L62-L64)):

```
  x     += d · cos(θ_mid)
  y     += d · sin(θ_mid)
  theta  = wrap_pi(theta + dθ)
```

where `θ_mid = θ + dθ/2` is the midpoint heading (better than start/end for
linearization).

Correct ([src/kalman.cpp:95](../src/kalman.cpp#L95)): innovation
`y(2) = wrap_pi(z.θ − pose.θ)` — handles the ±π wrap correctly so a
2°-CCW measurement near θ = π doesn't produce a 358° innovation.

## Odometry

[src/odometry.cpp:54-55](../src/odometry.cpp#L54-L55):

```
  d_mm    = (dL + dR) / 2
  d_theta = (dR − dL) / WHEELBASE_MM
```

Sign check: pure CCW pivot in place ⇒ dL < 0, dR > 0 ⇒ d_theta > 0 ⇒ +θ.
Matches the EKF/IK convention.

## Camera bearing

[src/tracker.cpp:40-43](../src/tracker.cpp#L40-L43):

```
  bearing = atan2(cx_image − cx_target, focal_length_px)
```

- Target to the **left of image center** ⇒ `cx_target < cx_image`
  ⇒ `bearing > 0`
- Target to the right ⇒ `bearing < 0`

So `+bearing = left = +y direction in body frame`, matching the world
convention.

## Visual servo

[src/controller.cpp:489-491](../src/controller.cpp#L489-L491):

```
  ω = clamp(k · bearing, ±ω_max)
```

`+bearing` (target left) → `+ω` (CCW) → robot rotates left toward target.
Signs line up.

## Polar → world

[src/controller.cpp:27-33](../src/controller.cpp#L27-L33), used to project a
bearing+distance measurement (from vision or anywhere) into world frame:

```
  world_bearing = pose.theta + bearing
  out_x = pose.x + dist · cos(world_bearing)
  out_y = pose.y + dist · sin(world_bearing)
```

Assumes the camera's optical axis is parallel to the robot's +x. See
[Hardware-dependent assumptions](#hardware-dependent-assumptions).

## TEST sequence waypoints

[src/controller.cpp](../src/controller.cpp) (`TEST_WAYPOINTS`):

```
  (  0,   0,    0  )  ◄── start (home)
  (400,   0,    0  )  ◄── forward 400 mm, still facing +x
  (400, 300,  π/2  )  ◄── turn CCW 90°, advance 300 mm in +y (LEFT)
  (  0,   0,    0  )  ◄── return home
```

- `+y = 300` means the second leg ends 30 cm to the **robot's left** of the
  first leg's endpoint — matches the convention.
- `θ = +π/2` at waypoint 2 means heading rotated 90° CCW, i.e. now facing +y.
- The user-requested "+90° turn" maps to +θ = CCW = turn left.

## Per-component summary

| Component | Code | Conforms? |
|---|---|---|
| KF predict | [kalman.cpp:62-64](../src/kalman.cpp#L62-L64) | ✓ |
| KF correct (innovation wrap) | [kalman.cpp:95](../src/kalman.cpp#L95) | ✓ |
| Odometry sign | [odometry.cpp:54-55](../src/odometry.cpp#L54-L55) | ✓ |
| Motor IK | [motor.cpp:168-169](../src/motor.cpp#L168-L169) | ✓ |
| Motor wiring sign-flip (matched pair) | [motor.cpp:138](../src/motor.cpp#L138), [motor.cpp:169](../src/motor.cpp#L169) | ✓ |
| LIDAR polar→body | [utils.hpp:45-50](../include/utils.hpp#L45-L50) | ✓ |
| Cox per-scan transform | [cox.cpp:108-110](../src/cox.cpp#L108-L110) | ✓ (identical to utils.hpp) |
| Cox world rotation matrix | [cox.cpp:138](../src/cox.cpp#L138) | ✓ standard CCW R(θ) |
| find_walls (reference map) | [calibrations.cpp:527](../src/calibrations.cpp#L527) | ✓ uses utils.hpp helper |
| Camera bearing | [tracker.cpp:40-43](../src/tracker.cpp#L40-L43) | ✓ |
| Visual servo sign | [controller.cpp:489-491](../src/controller.cpp#L489-L491) | ✓ |
| polar_to_world | [controller.cpp:27-33](../src/controller.cpp#L27-L33) | ✓ |
| TEST waypoints | [controller.cpp](../src/controller.cpp) | ✓ |

## Sign-table sanity drills

### Drive forward 100 mm from (0, 0, 0)

```
  encoder:   dL = +100,  dR = +100
  odom:      d_mm = +100,  d_theta = 0
  KF:        x += 100·cos(0) = +100        ✓ X grows
             y += 100·sin(0) =    0        ✓ Y unchanged
             θ unchanged                   ✓
  motor cmd: v=+100, ω=0
             set_l = +100·s, set_r = −(+100·s)
             after wiring flip: both wheels physically forward  ✓
  lidar:     walls in front grow closer in body frame
             Cox correction pushes +X further (matches odom)    ✓
```

### Rotate CCW (turn LEFT) 90° in place from (0, 0, 0)

```
  motor cmd: v=0, ω=+ω
             set_l = −scale, set_r = −(+scale)
             after wiring flip: L back, R fwd  → CCW            ✓
  encoder:   dL = −x,  dR = +x
  odom:      d_mm = 0,  d_theta = +2x/b > 0                     ✓
  KF:        θ → +π/2                                            ✓
  camera:    a target sitting at world (1 m, 0) was at bearing 0;
             after the pivot, world_bearing − θ = 0 − π/2 = −π/2
             (target now 90° to the right of optical axis)      ✓
```

## Hardware-dependent assumptions

Two assumptions can't be verified from code alone — they live in the physical
mounting. Both have 30-second verification procedures.

### 1. `LIDAR_MOUNT_OFFSET_RAD = −π/2`

Claims the lidar is mounted **rotated 90° CW** (raw 0° beam points to the
robot's right). The transform math is correct *for that mounting*. If the
mounting is different the entire reference map will be rotated by a constant
angle and Cox still works (the rotation absorbs into pose θ) — **but pushing
the robot physically forward will not increase `KF.X_mm`**.

**Verify:** Run TEST mode. Watch the dashboard.

- Leg 1 (forward 400 mm): `KF.X_mm` should rise 0 → ~400, `KF.Y_mm` stays
  near 0.
- If `KF.X_mm` stays at 0 and `KF.Y_mm` grows instead, the offset is wrong
  by ±90°. Adjust `LIDAR_MOUNT_OFFSET_RAD` in
  [include/global.hpp](../include/global.hpp).
- If both move the wrong way (forward decreases X), the offset is wrong by
  π. Add π to the constant.

### 2. Camera optical axis aligned with robot +x

[src/tracker.cpp:40-43](../src/tracker.cpp#L40-L43) returns bearing relative
to the camera's optical axis. [src/controller.cpp:30](../src/controller.cpp#L30)
adds it directly to `pose.θ`. This is correct **only if the camera looks
straight ahead** (parallel to +x). There is no `CAMERA_MOUNT_OFFSET`
constant — if the camera is canted left or right, every visual-servo target
carries a constant bearing bias.

**Verify:** Place a box dead-center in front of the robot at ~1 m. Check the
`Vision.bearing_deg` dashboard reading — it should be within ±1°. If it
reads consistently 5-10° off with the box clearly centered, the camera is
canted. Fix by either physically realigning, or adding a
`CAMERA_MOUNT_OFFSET_RAD` constant and subtracting it at
[src/tracker.cpp:42](../src/tracker.cpp#L42).

## How to add a new sensor

If you ever add another body-mounted sensor (second lidar, IMU, etc):

1. **Convert into the body frame at the source.** Don't sprinkle sign-fix
   tweaks across consumers — bake the transform once at the producer
   (analogous to `lidar_polar_to_body` / `LIDAR_MOUNT_OFFSET_RAD`).
2. **Document the mount offset** as a constant in
   [include/global.hpp](../include/global.hpp). Comments should list the
   per-value semantics like `LIDAR_MOUNT_OFFSET_RAD` does.
3. **Run the sanity drill** above with the new sensor's output during the
   TEST sequence. If forward → X grows and CCW → θ grows, you're done.
