# Config and Tunables Reference

`config.yml` is read at startup by `load_config()` (see `src/calibrations.cpp`). Missing files or keys are not errors; defaults are used instead. Keys are dotted paths built from YAML sections (e.g. `camera.width`). Units are called out per key.

## `config.yml` keys

### `camera`

| Key | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `camera.width` | `640` | Camera frame width in pixels. | **Increase:** more pixels and detail but more CPU and bandwidth; changes distance/bearing scaling. **Decrease:** faster but less detail and noisier distance estimates. |
| `camera.height` | `480` | Camera frame height in pixels. | **Increase:** more vertical resolution; more CPU. **Decrease:** faster, less detail. |
| `camera.fps_target` | `12` | Desired capture FPS passed to OpenCV. | **Increase:** more responsive tracking, potentially darker frames and higher CPU. **Decrease:** lower CPU but higher latency and noisier hysteresis. |
| `camera.focal_length_px` | `650.0` | Focal length in pixels used for pinhole distance and bearing. | **Increase:** distance estimates grow and bearing shrinks (narrower FOV). **Decrease:** distance estimates shrink and bearing grows. |
| `camera.grab_bottom_region_ratio` | `0.35` | Height fraction of the image used as the bottom “grabbable” region. | **Increase:** region extends further upward → more chances to be “grabbable” (risk of false positives). **Decrease:** stricter, box must be lower in frame. |
| `camera.grab_bottom_fill_ratio` | `0.60` | Minimum overlap ratio between the detected box and the bottom region to mark grabbable. | **Increase:** requires more overlap → later/closer grabs. **Decrease:** earlier grabs, risk premature closing. |
| `camera.capture_distance_mm` | `120.0` | Max distance for “captured” state. | **Increase:** capture earlier/farther. **Decrease:** capture only when very close. |
| `camera.capture_bearing_tol_rad` | `0.12` | Max bearing offset from centerline to allow “captured”. | **Increase:** more tolerant of lateral offset. **Decrease:** stricter alignment requirement. |
| `camera.retention_distance_mm` | `150.0` | Max distance to keep “captured” true once captured. | **Increase:** retain capture longer with noisy distance. **Decrease:** release sooner on distance spikes. |
| `camera.show_debug_view` | `false` | If true, shows the OpenCV debug window. | **True:** renders frames and overlays (needs display). **False:** headless and faster. |

### `robot`

> **Note:** The robot extents are currently **not wired into code** (they are comments for geometry planning). They are intended for wall-clearance and home-placement math.

| Key | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `robot.back_extent_mm` | `150.0` | Rear overhang behind the LIDAR mount. | **Increase:** assumes a longer rear; would require more rear clearance. **Decrease:** assumes a shorter rear. |
| `robot.left_extent_mm` | `70.0` | Left side overhang from LIDAR center. | **Increase:** assumes wider left side; tighter clearance. **Decrease:** assumes narrower left side. |
| `robot.right_extent_mm` | `70.0` | Right side overhang from LIDAR center. | **Increase:** assumes wider right side; tighter clearance. **Decrease:** assumes narrower right side. |

### `mission`

> **Note:** Only `mission.home_theta_rad` is used today (seed pose for localization). The other mission keys are loaded only if `StateMachine::configure()` is wired in.

Home is centered on the robot's **starting pose** (world origin). The home zone is a 1 ft x 1 ft area, with `home_margin_mm` = 0.5 ft as the arrival radius.

| Key | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `mission.home_x_mm` | `0.0` | Target **box** landing X (mm) in the localization frame; defaults to the robot's starting pose. | **Increase / Decrease:** shifts the drop point relative to the starting pose. |
| `mission.home_y_mm` | `0.0` | Target **box** landing Y (mm) in the localization frame. | **Increase / Decrease:** shifts drop point left/right. |
| `mission.home_theta_rad` | `0.0` | Home pose heading (rad) used to seed the KF at boot. | **Increase:** rotate seed heading CCW. **Decrease:** rotate seed heading CW. |
| `mission.home_margin_mm` | `152.4` | Arrival radius (≈ 0.5 ft) used at home — half the side of the 1 ft home zone. | **Increase:** larger home zone, easier to "arrive". **Decrease:** tighter precision. |
| `mission.arm_length_mm` | `80.0` | Distance from robot center to the box when held. | **Increase:** biases robot stop point further back from the drop. **Decrease:** biases stop point forward. |

### `map`

The reference map for cox localization is built from ~50 lidar frames at boot using RANSAC. There is no hardcoded arena geometry — walls are whatever the lidar finds.

Defaults are tuned for a **rectangular arena with 1-2 other robots inside**: tight inlier gates keep walls straight, high inlier requirements reject robot-body clusters, and the segment-length filter discards anything shorter than a real wall.

| Key | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `map.ransac.inlier_thresh_mm` | `25.0` | Perpendicular distance gate for line inliers. | **Increase:** tolerates more noise; risks fitting around obstacles. **Decrease:** tighter fits, may drop noisy wall points. |
| `map.ransac.max_iters_per_line` | `400` | RANSAC trials per line. | **Increase:** more robust under robot occlusion, slower. **Decrease:** faster. |
| `map.ransac.inlier_points_req` | `30` | Minimum inliers (angle-averaged points) for a line. | **Increase:** stricter, rejects robots more aggressively but may miss partially-occluded walls. **Decrease:** accepts weaker lines (may mis-fit robots). |
| `map.ransac.min_inlier_fraction` | `0.06` | Minimum fraction of points that must be inliers. | **Increase:** stricter; stops RANSAC sooner. **Decrease:** more permissive. |
| `map.ransac.max_lines` | `6` | Cap on walls extracted. 4 walls + 2 slack for occlusion-split walls. | **Increase:** allows more candidates (risk clutter). **Decrease:** fewer lines. |
| `map.ransac.endpoint_padding_mm` | `50.0` | Extends wall endpoints for downstream matching. | **Increase:** more forgiving endpoints. **Decrease:** tighter endpoints. |
| `map.ransac.min_segment_len_mm` | `600.0` | Reject segments shorter than this. Robot bodies are ≤ 400 mm wide. | **Increase:** filters robots more aggressively, may drop short walls. **Decrease:** allows shorter segments (risk fitting robot edges). |

### `logger`

> **Note:** only `udp_*` and `scan_subsample` are currently used (via `telemetry::init()` in `main.cpp`). The others are present for older lab logging paths.

| Key | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `logger.files_enabled` | `true` | Intended to toggle file logging (not currently wired). | **True/False:** no effect in current code path. |
| `logger.udp_enabled` | `true` | Enables UDP telemetry streaming. | **True:** emits telemetry. **False:** disables UDP output. |
| `logger.out_dir` | `logs` | Intended output directory for file logging (not currently wired). | **Increase/Decrease:** not used today. |
| `logger.udp_host` | `169.254.6.52` | UDP destination host for telemetry. | **Change:** points telemetry to a different listener. |
| `logger.udp_port` | `5005` | UDP destination port for telemetry. | **Increase/Decrease:** use a different UDP port. |
| `logger.tick_ms` | `100` | Intended telemetry/log tick in ms (not currently wired). | **Increase/Decrease:** no effect in current code path. |
| `logger.scan_subsample` | `2` | Send every Nth lidar sample over UDP. | **Increase:** fewer points, smaller packets. **Decrease:** more points, higher bandwidth. |

### `battery`

| Key | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `battery.enabled` | `false` | Enables the INA219 battery monitor thread. | **True:** enables voltage/current monitoring and auto-shutdown on low battery. **False:** disables battery checks. |

## Other tunable values (code constants)

These are **not** in `config.yml`; they require editing code. File paths are provided for quick navigation.

### Global geometry & speed

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `include/global.hpp:LIDAR_MOUNT_OFFSET_RAD` | `-π/2` | Angle from robot +x to LiDAR 0° scan. | **Increase/Decrease:** rotates all scans; wrong value mirrors/rotates localization. |
| `include/global.hpp:ROBOT_SPEED` | `1.0` | Global speed scale for velocity/accel limits. | **Increase:** faster driving/turning/ramping. **Decrease:** slower and safer. |

### Motor & drivetrain

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `include/motor.hpp:TICKS_PER_MM` | `71.48` | Encoder ticks per mm (distance scale). | **Increase:** reports less distance per tick (slower odom). **Decrease:** reports more distance. |
| `include/motor.hpp:WHEELBASE_MM` | `185.0` | Wheelbase used for kinematics and odometry. | **Increase:** reduces computed yaw per wheel delta. **Decrease:** increases yaw per wheel delta. |
| `src/motor.cpp:ACCEL_V_MM_S2` | `150 * ROBOT_SPEED` | Linear acceleration limit. | **Increase:** snappier acceleration. **Decrease:** smoother but slower response. |
| `src/motor.cpp:ACCEL_W_RAD_S2` | `1.5 * ROBOT_SPEED` | Angular acceleration limit. | **Increase:** quicker turns. **Decrease:** smoother turning. |
| `src/motor.cpp:MAX_CMD_PPS100` | `286` | Hard cap on wheel command. | **Increase:** higher top speed; may exceed safe motor range. **Decrease:** lower max speed. |

### Odometry noise model

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `src/odometry.cpp:K_WHEEL_INDEP` | `0.020` | Per-wheel slip fraction (heading noise). | **Increase:** higher θ uncertainty. **Decrease:** tighter θ covariance. |
| `src/odometry.cpp:K_COMMON` | `0.025` | Common-mode distance scale error. | **Increase:** higher distance uncertainty. **Decrease:** tighter distance cov. |
| `src/odometry.cpp:K_ROT` | `0.030` | Extra rotation scale error. | **Increase:** more θ noise during turns. **Decrease:** less. |
| `src/odometry.cpp:VAR_L_FLOOR_MM2` | `0.25` | Floor on wheel variance. | **Increase:** more noise at tiny steps. **Decrease:** less. |
| `src/odometry.cpp:VAR_THETA_FLOOR` | `1e-6` | Floor on θ variance per step. | **Increase:** larger base θ uncertainty. **Decrease:** smaller. |

### Pure pursuit path follower

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `include/pure_pursuit.hpp:_L_d` | `250 mm` | Lookahead distance. | **Increase:** smoother but cuts corners more. **Decrease:** tighter tracking, can oscillate. |
| `include/pure_pursuit.hpp:_v_max` | `75 mm/s * ROBOT_SPEED` | Max linear speed cap. | **Increase:** faster tracking. **Decrease:** slower and safer. |
| `include/pure_pursuit.hpp:_omega_max` | `1.5 rad/s * ROBOT_SPEED` | Max angular speed cap. | **Increase:** sharper turns. **Decrease:** gentler turns. |
| `include/pure_pursuit.hpp:_heading_done_rad` | `0.05 rad` | Heading tolerance for terminal rotate completion. | **Increase:** earlier “done”. **Decrease:** stricter final heading. |
| `include/pure_pursuit.hpp:_pre_roll_thresh` | `0.52 rad` | Heading error threshold to leave PRE_ROLL. | **Increase:** more time rotating in place before moving. **Decrease:** engages pursuit sooner. |
| `include/pure_pursuit.hpp:_k_pre_roll` | `2.0` | P gain for PRE_ROLL angular command. | **Increase:** faster pre-roll. **Decrease:** slower, smoother. |
| `include/pure_pursuit.hpp:_k_terminal` | `2.0` | P gain for terminal rotate. | **Increase:** faster final alignment. **Decrease:** slower but smoother. |

### Mission state machine (search/approach/carry)

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `include/controller.hpp:_search_omega` | `0.35 * ROBOT_SPEED` | SEARCH spin rate. | **Increase:** faster scanning. **Decrease:** more stable vision. |
| `include/controller.hpp:_search_min_dwell_s` | `2.0 s` | Minimum time between failed approaches. | **Increase:** more patience; slower retry. **Decrease:** faster retry. |
| `include/controller.hpp:_search_v_max_mm_s` | `40 * ROBOT_SPEED` | SEARCH waypoint speed. | **Increase:** faster travel. **Decrease:** slower, safer. |
| `include/controller.hpp:_waypoint_margin_mm` | `80 mm` | SEARCH waypoint arrival radius + wall standoff. | **Increase:** looser arrival, more wall clearance. **Decrease:** tighter path. |
| `include/controller.hpp:_box_detection_range_mm` | `700 mm` | Sets subdivision spacing for search plan. | **Increase:** fewer waypoints (coarser). **Decrease:** denser search. |
| `include/controller.hpp:_grip_settle_s` | `1.0 s` | Wait after closing gripper before checking capture. | **Increase:** more time to settle. **Decrease:** quicker transitions. |
| `include/controller.hpp:_approach_v` | `30 * ROBOT_SPEED` | APPROACH cruise speed. | **Increase:** more aggressive approach. **Decrease:** gentler approach. |
| `include/controller.hpp:_approach_k_omega` | `1.8` | Bearing-to-omega gain in APPROACH. | **Increase:** stronger steering corrections. **Decrease:** softer corrections. |
| `include/controller.hpp:_approach_omega_max` | `1.2 * ROBOT_SPEED` | Max APPROACH turning rate. | **Increase:** sharper turns. **Decrease:** smoother turns. |
| `include/controller.hpp:_dead_reckon_arrival_mm` | `100 mm` | How close to last target before giving up without vision. | **Increase:** gives up later. **Decrease:** gives up sooner. |
| `include/controller.hpp:_release_v` | `-40 mm/s` | Back-up speed during RELEASE. | **More negative:** faster back-up. **Less negative:** slower back-up. |
| `include/controller.hpp:_release_seconds` | `1.25 s` | Back-up duration. | **Increase:** longer retreat. **Decrease:** shorter retreat. |
| `include/controller.hpp:_release_hard_timeout_s` | `5.0 s` | Max time waiting for box to clear. | **Increase:** more patience. **Decrease:** faster failover. |
| `include/controller.hpp:_search_timeout_s` | `60 s` | SEARCH timeout. | **Increase:** longer search before fail. **Decrease:** shorter. |
| `include/controller.hpp:_approach_timeout_s` | `20 s` | APPROACH timeout. | **Increase:** more time to approach. **Decrease:** quicker fallback. |
| `include/controller.hpp:_carry_timeout_s` | `90 s` | CARRY_HOME timeout. | **Increase:** longer to reach home. **Decrease:** quicker failover. |
| `include/controller.hpp:_carry_replan_s` | `5 s` | Path replan cadence during CARRY_HOME. | **Increase:** less replanning. **Decrease:** more frequent replanning. |

### Cox localization & relocalize

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `src/cox.cpp:ENDPOINT_SLACK_MM` | `50` | Endpoint slack for segment match. | **Increase:** more tolerant of endpoint noise. **Decrease:** stricter segment match. |
| `src/cox.cpp:gate_k` | `4.0` | Gate multiplier on range noise. | **Increase:** more permissive gating. **Decrease:** stricter gating. |
| `src/cox.cpp:tol_d_xy` | `1.5` | Convergence tolerance (translation). | **Increase:** earlier stop. **Decrease:** more iterations. |
| `src/cox.cpp:tol_d_a` | `1e-3` | Convergence tolerance (angle). | **Increase:** earlier stop. **Decrease:** more iterations. |
| `src/cox.cpp:MIN_INLIERS_FUSE` | `6` | Min inliers to fuse Cox into KF. | **Increase:** stricter fuse. **Decrease:** more frequent fuses. |
| `src/cox.cpp:MIN_INLIERS_FIX` | `15` | Min inliers for relocalize winner. | **Increase:** stricter relocalize acceptance. **Decrease:** more permissive. |
| `src/cox.cpp:MAX_SIGMA_XY_FIX` | `30 mm` | Max σ_xy to accept relocalize winner. | **Increase:** more permissive. **Decrease:** stricter. |
| `src/cox.cpp:MAX_SIGMA_THETA_FIX` | `0.035 rad` | Max σ_θ to accept relocalize winner. | **Increase:** more permissive. **Decrease:** stricter. |
| `src/cox.cpp:KF_LOST_SIGMA_XY_MM` | `200 mm` | KF “lost” trigger (position). | **Increase:** relocalize less often. **Decrease:** relocalize sooner. |
| `src/cox.cpp:KF_LOST_SIGMA_TH` | `15°` | KF “lost” trigger (heading). | **Increase:** relocalize less often. **Decrease:** relocalize sooner. |
| `src/cox.cpp:HALF_XY_MM / GRID_MM` | `300 / 100` | Relocalize sweep window/step. | **Increase:** wider/denser search (slower). **Decrease:** narrower/faster. |
| `src/cox.cpp:HALF_THETA / GRID_RAD` | `45° / 15°` | Relocalize heading sweep. | **Increase:** broader heading sweep (slower). **Decrease:** narrower/faster. |

### Vision tracking

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `include/types.hpp:BOX_HEIGHT_MM` | `40 mm` | Physical box height for pinhole distance. | **Increase:** estimated distance increases. **Decrease:** estimated distance decreases. |
| `src/tracker.cpp:area threshold` | `500 px²` | Minimum contour area to accept a blob. | **Increase:** rejects small detections. **Decrease:** more detections (more noise). |
| `src/tracker.cpp:close_kernel` | `5x5` | Morphology close kernel size. | **Increase:** more aggressive blob fill. **Decrease:** less smoothing. |
| `src/tracker.cpp:CAPTURE_FRAMES` | `10` | Debounce frames to mark “captured”. | **Increase:** more stable but slower capture. **Decrease:** faster but noisier. |
| `src/tracker.cpp:RETENTION_FRAMES` | `10` | Debounce frames to drop capture. | **Increase:** holds capture longer. **Decrease:** releases sooner. |
| `src/tracker.cpp:LOST_FRAMES` | `10` | Frames to declare “lost”. | **Increase:** less sensitive to dropouts. **Decrease:** quicker lost state. |
| `src/tracker.cpp:GRAB_FRAMES` | `8` | Frames to declare “grabbable”. | **Increase:** more stable, slower. **Decrease:** faster, noisier. |
| `src/tracker.cpp:RELEASE_FRAMES` | `8` | Frames to clear “grabbable”. | **Increase:** holds grabbable longer. **Decrease:** clears sooner. |

### Lidar I/O and obstacle flagging

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `include/Lidar.hpp:LIDAR_SERIAL_DEVICE` | `/dev/ttyUSB0` | Serial device for LiDAR. | **Change:** points to another device. |
| `include/Lidar.hpp:LIDAR_SERIAL_BAUDRATE` | `115200` | Serial baud rate. | **Increase/Decrease:** must match device; wrong value breaks comms. |
| `include/Lidar.hpp:LIDAR_DEFAULT_MOTOR_PWM` | `660` | LiDAR motor PWM for normal scan. | **Increase:** faster spin. **Decrease:** slower spin (may reduce sample quality). |
| `src/utils.cpp:OBSTACLE_DIST_MM` | `140 mm` | Distance threshold for obstacle flag. | **Increase:** more conservative obstacle flag. **Decrease:** less sensitive. |
| `src/utils.cpp:OBSTACLE_ARC_HALF_DEG` | `60°` | Half-angle of forward arc to check. | **Increase:** wider arc. **Decrease:** narrower arc. |

### Battery monitor

| Location | Default | What it does | Increase / Decrease effect |
|---|---|---|---|
| `include/types.hpp:LOW_BATTERY_LIMIT` | `11.25 V` | Shutdown threshold. | **Increase:** earlier shutdown. **Decrease:** deeper discharge risk. |
| `include/types.hpp:BATTERY_PRESENT_MIN_V` | `1.0 V` | Below this, treat as “no battery”. | **Increase:** more likely to treat rail as absent. **Decrease:** more tolerant of low rail readings. |
| `include/types.hpp:LOW_HITS_NEEDED` | `5` | Debounce count for low-battery shutdown. | **Increase:** slower to shut down. **Decrease:** quicker. |
| `include/types.hpp:CHECK_PERIOD` | `3 s` | Battery check interval. | **Increase:** fewer checks. **Decrease:** more frequent checks. |
| `src/core.cpp:ABSENT_HITS_CAP` | `10` | Debounce count for “battery absent” shutdown. | **Increase:** slower to shut down on sensor fault. **Decrease:** faster. |
