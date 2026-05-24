# Robotics Lab Bot — Operator Guide

A box-fetching mobile robot built on a Raspberry Pi with a 360° LIDAR and a
USB camera. The mission loop is **SEARCH → APPROACH → grab → CARRY_HOME →
RELEASE → re-search**, with colour-based vision tracking, lidar obstacle
gating, cox + KF localization, and a B-cubic spline + PurePursuit path
follower.

This guide covers everything needed to get the robot driving: first-time
deploy, colour calibration, running the mission, and reading the dashboard.

---

## 1. Prerequisites

Hardware:
- Raspberry Pi (Bullseye+, with a working WiringPi install)
- 360° LIDAR on `/dev/ttyUSB0`
- USB camera at `/dev/video0`
- Two wheeled motors driven via SPI (handled by the included `motor.so`)
- Optional: INA219 battery monitor on I²C

Pi-side build dependencies:
- `g++` with C++11 support
- `make`
- `libopencv-dev` (the Makefile auto-selects `opencv4` if available, else
  falls back to `opencv`)
- `libwiringpi-dev`
- Eigen headers at `/usr/include/eigen3`

Workstation-side (for the push script):
- `sshpass`
- `zip`

---

## 2. Deploy to the Pi

Two ways.

### A. Push from workstation (recommended)

`push_on_pi.sh` zips up `main.cpp`, `src/`, `include/`, `Makefile`,
`libraries/`, and `config.yml`, sends it to the Pi, unpacks it over the
existing `~/group-6/` tree, and runs `make`.

```bash
# Edit REMOTE_HOST in push_on_pi.sh to match your Pi's IP first.
./push_on_pi.sh
```

The Pi keeps its `build/` cache across pushes, so incremental builds are
fast (~10 s for an edit, ~60 s from clean).

### B. Build directly on the Pi

```bash
ssh pi@<pi-ip>
cd ~/group-6
make
```

`make -j3` is on by default in the Makefile (Pi 4 has 4 cores; one is left
for the OS). To start from scratch: `make clean && make`.

---

## 3. First-time colour calibration

The vision tracker needs HSV ranges for **the target colour** (the boxes
the mission should pick up) and optionally **N obstacle colours** (boxes
the mission must avoid pushing). Calibration is a one-time, interactive
flow; once saved in `config.yml`, mission runs load it and skip the UI.

### Trigger

Calibration runs **only when `filter.ready` is `false` or missing** in
`config.yml`. To recalibrate later, edit `config.yml`:

```yaml
filter:
  ready: false
```

…then run the app again.

### Procedure

```bash
ssh pi@<pi-ip>
cd ~/group-6
./start.sh    # runs `sudo WIRINGPI_CODES=1 ./app`
```

The app will detect missing calibration and prompt:

1. **Target colour window** opens (`Calibration Setup`). A green ROI
   rectangle marks the area sampled by the dominant-colour k-means.
   - Hold the **target box** centered in the ROI under representative
     mission lighting.
   - Press **`c`** to capture → terminal prints the selected dominant HSV
     and saves the derived filter into `target_filter`.
   - Press **`q`** to abort.
2. Terminal prompts: `Calibrate obstacle colour 0? (y/n):`
   - `y` → same trackbar UI for the first obstacle colour.
   - `n` → finish, write config, exit cleanly.
3. Repeat for obstacle 1, 2, … as many as you want.
4. On finish the config is saved as:
   ```yaml
   filter:
     ready: true
     target:
       has_wrap_around: false
       dominant_color: "..."
       lower1: "H,S,V"; upper1: "H,S,V"
       lower2: "0,0,0"; upper2: "0,0,0"
     obstacles:
       count: 2
       0: { ...same shape as target... }
       1: { ... }
   ```
5. Terminal prints `"Restart the app to run the mission."` and exits 0.

Run `./start.sh` again to actually run the mission with the saved filters.

### Tips

- Calibrate under the **same lighting** the mission will run in. The
  asymmetric saturation/value margins assume the calibration scene is
  representative of running conditions.
- Keep the target box mostly inside the ROI; small fragments give bad
  cluster centers.
- For red boxes (hue wraps around 180°/0°) the calibrator auto-detects
  the wrap and saves the two-range filter.
- To **add** an obstacle later without redoing everything, you can edit
  `config.yml` by hand following the existing schema (clone an obstacle
  entry, bump `filter.obstacles.count`). Or just set `filter.ready: false`
  and run the full flow again.

---

## 4. Run the mission

```bash
ssh pi@<pi-ip>
cd ~/group-6
./start.sh    # = sudo WIRINGPI_CODES=1 ./app
```

Boot sequence (~7–9 s total):

1. Load `config.yml`; resolve `ROBOT_SPEED` (default 1.875).
2. Load filter calibration (target + N obstacles).
3. Spawn `loop_localization` — lidar motor spins up (~1.5 s) **in parallel**
   with `loop_control`'s motor SPI handshake + 300 ms chassis settle.
4. Once `motors_ready` is set, lidar frame collector starts;
   `loop_localization` waits 4 s to accumulate ~20 frames.
5. RANSAC wall extraction (~1–2 s); cox seeds, KF accepts first fix.
6. Vision tracker thread spins up in parallel from `main`.
7. Mission gate opens → `sm.start()` → IDLE → **SEARCH**.

Wall-time is roughly `max(lidar spin, motor settle) + frame collect +
RANSAC + cox first fix` ≈ 1.5 + 4 + 1.5 + 0.5 ≈ **~7.5 s** under typical
conditions, down from the previous ~20 s.

The robot starts the look-left / look-right / forward sweep, attempts to
pick up any target-colour box it sees (subject to the at-home, behind-home,
failed-target, and obstacle-block skip filters), drags it back to (0, 0),
releases it, realigns to `home_theta`, and re-enters SEARCH.

### Stopping

- `Ctrl+C` once: clean shutdown — stops motors, joins threads, closes
  loggers, exits.
- `Ctrl+C` twice: hard exit (`_exit(130)`), use only if the clean shutdown
  hangs.

---

## 5. Dashboard quick reference

The terminal renders a live multi-panel dashboard while the app is
running. The key fields per panel:

| Panel       | Key                | Meaning                                                   |
|-------------|--------------------|-----------------------------------------------------------|
| **Battery** | `voltage`, `soc`   | INA219 reading; auto-shutdown below `LOW_BATTERY_LIMIT`   |
| **Cox**     | `inliers`, `cond`  | Scan-match health. `cond` >> 1e5 = degenerate geometry    |
| **KF**      | `sigma_xy`, `sigma_th` | Fused-pose uncertainty                                |
| **Motor**   | `v_cmd`, `w_cmd_deg_s` | Last commanded body velocity                          |
| **Odometry**| `D_mm`, `A_deg`    | Cumulative wheel-encoder distance / heading               |
| **Vision**  | `fps`              | Camera frame rate                                         |
|             | `target`           | `yes` / `no` — target colour blob present this frame      |
|             | `bearing_deg`, `dist_mm` | Target pose (only when `target=yes`)                |
|             | `captured`         | Target inside retention zone (debounced)                  |
|             | `grabbable`        | Target inside gripper-close zone (debounced)              |
|             | `lost`             | No detection for ≥ `LOST_FRAMES`                          |
|             | `obs_seen`         | Number of obstacle-colour blobs this frame                |
| **Mission** | `state`            | `IDLE` / `SEARCH` / `APPROACH` / `CARRY_HOME` / ...       |
|             | `t_state_s`        | Seconds since entering current state                      |
|             | `cmd_mode`         | `path` (PurePursuit) / `direct` (state machine) / `stop`  |
|             | `search`           | `L` / `R` / `FWD` for sweep phase; `/R` suffix = realigning |
|             | `fwd_mm`           | Distance traveled on current FORWARD leg                  |
|             | `obs_rot_deg`      | Cumulative obstacle-dodge rotation this leg               |
|             | `obstacles`        | Count of remembered obstacle world positions              |
|             | `attempts`         | Failed-approach budget consumed (out of 5)                |
|             | `skipping`         | `yes` while SEARCH is filtering a detected target         |
|             | `box_to_home_mm`   | Distance from currently-held box to home                  |

---

## 6. Common operations

### Re-run calibration

```yaml
# config.yml
filter:
  ready: false   # was true
```
Run `./start.sh`. The app drops back into the calibration UI and exits when
finished. Run again to start the mission.

### Tune mission knobs without recalibrating

`config.yml` is loaded fresh on every run — no rebuild needed for any
`camera.*`, `robot.*`, `mission.*`, `map.*`, `logger.*`, `battery.*` value.

In-code tunables (the `_search_*`, `_approach_*`, `_carry_*`, `_release_*`
families in [`include/controller.hpp`](include/controller.hpp), plus the
obstacle gate / cox / pure-pursuit constants) require `make` after editing.
See [`docs/tunables.md`](docs/tunables.md) for the full catalogue.

### Single-shot drive (no SEARCH mission)

Set in `config.yml`:
```yaml
mission:
  goto: "1200,0,0"     # "x_mm,y_mm,theta_deg"
```
The mission boots straight to a GOTO leg, halts with localize, and parks
at DONE. SIGINT to exit.

### Hardcoded test sequence

```cpp
// In main.cpp or a wrapper, call sm.start_test() instead of sm.start().
```
Drives forward 400 mm → 90° CCW + 300 mm → back to origin. Useful for
encoder / cox tuning without the mission state machine.

### Servo bench-test

```bash
make test          # builds servo_test (minimal: servo.c + wiringPi)
sudo ./servo_test
```

---

## 7. Troubleshooting

| Symptom                                          | Where to look                                                                              |
|--------------------------------------------------|---------------------------------------------------------------------------------------------|
| `filter.ready=true but filter.target is missing` | Set `filter.ready: false` and re-run to recalibrate. Existing legacy `filter.lower1`-style configs are auto-promoted to `filter.target.*` on load. |
| Vision sees nothing                              | `Vision.fps` panel; check camera permissions, lighting, and `camera.show_debug_view: true` to see the masked frame in an OpenCV window. |
| Cox `inliers` very low (< 10)                    | Bad map — try restarting the robot at home with a clear LIDAR view of all four walls. Tune `map.ransac.*` in `config.yml`. |
| Robot stalls during APPROACH                     | `Mission.cmd_mode` stays `direct` but `v_cmd ≈ 0`. The obstacle gate is suppressed in APPROACH now, so this usually means the visual servo's bearing is way off. Check `Vision.bearing_deg`. |
| CARRY_HOME releases before reaching home         | Log: `CARRY_HOME: target lost for X s with healthy vision`. Drag-guard fired. Bump `_carry_target_lost_timeout_s` if you're sure the grip is good. |
| Search keeps walking into the same wall          | `Mission.search` shows rapid `L` / `R` / `FWD` cycling. Cox heading jitter is exceeding the realign hysteresis — widen `_search_realign_enter_rad` or increase `_search_realign_stable_ticks`. |
| `APPROACH budget exhausted`                      | Repeated failed approaches on the same box. Either the target sits in an obstacle-block zone, the gripper geometry is misaligned, or the failed-target memory hasn't expired yet (`_failed_target_skip_s = 30 s`). |
| Robot bumps into walls during SEARCH             | Lidar obstacle gate is `OBSTACLE_DIST_MM = 230 mm`. Bump if you need more margin; the FORWARD-leg dodge handles up to a `π` rotation before abandoning the leg. |

Log lines worth grepping for:
- `[Cal]` — calibration UI events
- `[Mission]` — state transitions, skip reasons, drag-guard, push-then-grip, GOTO halts
- `[Vision]` — CAPTURED / GRABBABLE / LOST debounce edges
- `[Cox]` / `[MAP]` — localization and boot-time map build
- `[Motor]` / `[LIDAR]` — startup and shutdown

---

## 8. Further reading

- [`docs/tunables.md`](docs/tunables.md) — every tunable with rationale.
- [`docs/coordinate-frames.md`](docs/coordinate-frames.md) — body/world frame conventions, lidar offset.
- [`docs/cox-tuning.md`](docs/cox-tuning.md) — localization tuning notes.
- [`configs-readme.md`](configs-readme.md) — older `config.yml` reference (some sections superseded by `docs/tunables.md`).
- [`docs/audit/`](docs/audit/) — design audits (state machine, etc).
