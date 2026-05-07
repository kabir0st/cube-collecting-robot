# 01 — System Architecture

## Goal

Estimate the 2D pose `(x, y, θ)` of an RPLIDAR sitting inside a known-shaped
indoor environment, and print it once per second alongside an uncertainty
`(σ_x, σ_y, σ_θ)`. The robot is stationary during calibration, then may move
slowly during estimation. No odometry, no IMU — everything comes out of the
lidar.

The program runs on a Raspberry Pi 3B (Linux, gcc, C++11, single binary).

## Two phases

```
                 ┌───────────────────────────────┐
                 │ 1. Calibration  (one-shot,    │
                 │    lidar stationary, ~5 s)    │
                 │                               │
                 │   raw scans  ──► sigma_r      │
                 │              └──► RANSAC ──►  │
                 │                   wall list   │
                 └──────────────┬────────────────┘
                                ▼
                 ┌───────────────────────────────┐
                 │ 2. Estimation  (1 Hz, while   │
                 │    the robot may be moving)   │
                 │                               │
                 │   new scan + walls + prior    │
                 │   ──► Cox '91 line matching   │
                 │   ──► (pose, covariance)      │
                 └───────────────────────────────┘
```

Calibration produces two artefacts:

1. **`σ_r`** — pooled standard deviation of distance returns. Drives the
   residual gate in the matcher and the covariance scaling.
2. **Reference map** — a `std::vector<LineSeg>` of wall segments produced by
   iterative RANSAC + total-least-squares refit. Variable count, no
   "must be a rectangle" assumption.

Both are passed to the estimator. From there it's a 1 Hz print loop that
keeps running until SIGINT.

## Threads

There are three logical threads at any point in time:

| Thread       | Lifetime                        | Job                                                                |
|--------------|---------------------------------|--------------------------------------------------------------------|
| **main**     | whole program                   | drives phases, prints, owns shutdown                                |
| **listener** | one per phase, owned by `ListenerThread` | reads RPLIDAR packets, calls `process_point` on the active handler |
| **logger**   | whole program (if connected)    | drains the visualizer buffer every 42 ms                            |

`ListenerThread` is RAII: constructing it spawns the listener thread,
destruction joins it. There is no detached thread anywhere in the project.

```
main            listener            logger             driver
 │              (per phase)          │ (long-lived)    (external)
 │                                   │
 │ start logger                      │
 ├──────────────────────────────────►│
 │                                   │ connect to viz
 │                                   │
 │ ── do_calibration ─────────────┐  │
 │                                │  │
 │                                ▼  │
 │     ListenerThread cal_listener   │
 │           ↓                       │
 │       start_stream ──► accept ──►─┼─────► driver
 │                                   │
 │       start_lidar  ──── ctrl ────►│─────► driver
 │                                   │
 │       (5 s of process_point)      │
 │                                   │
 │       stop_stream  ──── ctrl ────►│─────► driver
 │              waits for recv_active=false
 │                                   │
 │     calibrate() / create_map()    │
 │     [scan_collection mutex]       │
 │                                   │
 │     ~ListenerThread (joins)       │
 │ ── done ──────────────────────────┘
 │
 │ ── do_estimate_position ──────────┐  (similar shape)
 │                                   │
```

## Files

```
include/
  global.hpp          — single shutdown atomic
  types.hpp           — Pose2D, LineSeg, LidarDataFrame, ports
  server.hpp          — LidarDataServer (TCP listener + control)
  utils.hpp           — send_control_signal, ListenerThread (RAII)
  loging.hpp          — Loging (visualizer feed)
  calibrate.hpp       — LidarCalibration (sigma + reference map)
  mapping.hpp         — Point, RansacParams, fit_lines_ransac
  estimate_position.hpp — PositionEstimator, do_estimate_position

src/
  server.cpp          — TCP server + RPLIDAR packet decode
  utils.cpp           — control signal sender
  loging.cpp          — visualizer worker thread
  calibration.cpp     — pooled sigma + RANSAC driver
  mapping.cpp         — RANSAC + total-least-squares refit
  estimate_position.cpp — Cox matcher + 1 Hz driver

main.cpp              — phase orchestration

docs/                 — this folder
visualizer.py         — Python TCP listener that draws scans + map
```

Each `.hpp` is roughly one concept; each `.cpp` is its implementation. There
are no cross-file utility piles — utilities live in the file that owns them.

## Key data shapes

### `LineSeg` ([include/types.hpp](../include/types.hpp))

```c++
struct LineSeg {
    float nx, ny, r;        // n . p = r is the line equation
    float ax, ay, bx, by;   // segment endpoints (mm)
};
```

The reference map is `std::vector<LineSeg>`. The unit normal `(nx, ny)`
points away from the sensor origin used during calibration. `r` is the
signed distance from origin to the line along the normal.

### `LidarDataFrame` ([include/types.hpp](../include/types.hpp))

```c++
struct LidarDataFrame {
    std::vector<int> angle;     // integer degrees, [0, 359]
    std::vector<int> distance;  // mm
};
```

One revolution. `PositionEstimator` keeps a rolling window of these.

### `Pose2D` ([include/types.hpp](../include/types.hpp))

```c++
struct Pose2D {
    float x, y;       // mm
    float theta;      // rad, wrapped to (-pi, pi]
};
```

## Synchronization summary

| Shared state                       | Owner                | Mutex                          | Notes                                                                                  |
|------------------------------------|----------------------|--------------------------------|----------------------------------------------------------------------------------------|
| `LidarCalibration::scan_collection_` | calibration phase    | `LidarCalibration::mx_`        | producer = listener, consumer = main after `stop_stream`                                |
| `PositionEstimator::frames_`         | estimation phase     | `PositionEstimator::buf_mx_`   | producer = listener, consumer = main at 1 Hz                                            |
| `Loging::buffer_`                    | whole program        | `Loging::buf_mutex_`           | producer = whoever called collect, consumer = logger worker thread                      |
| `app::shutdown`                      | whole program        | atomic                         | written by SIGINT handler, polled everywhere                                            |
| `LidarDataServer::*` flags + FDs     | whole program        | atomic                         | flags coordinate listener startup/shutdown                                              |

There is exactly **one** producer and **one** consumer for each of the buffers
in row 1 and row 2 — that simplifies reasoning. The mutex is the safety net
against the interval between "we asked the lidar to stop" and "the listener
thread has actually exited its recv loop". `stop_stream()` blocks until
that interval has elapsed, but the mutex makes the design robust to that
window growing for any reason.

## Shutdown

Pressing Ctrl+C triggers SIGINT, which sets `app::shutdown` to true. Every
blocking loop in the codebase polls this:

- `start_stream`'s recv loop exits.
- `start_lidar`'s wait-for-listener loop exits.
- `do_calibration` and `do_estimate_position` skip past their post-wait work.
- The 1 Hz print loop in `do_estimate_position` exits.

`logger.stop()` then joins the visualizer worker, and `main` returns. There
are no detached threads, no `std::quick_exit`, no leaked sockets.

## Cross-references

- For TCP packet layout and control protocol → [02-lidar-protocol.md](02-lidar-protocol.md)
- For sigma estimation math → [03-calibration.md](03-calibration.md)
- For RANSAC + total-least-squares fit → [04-mapping-ransac.md](04-mapping-ransac.md)
- For Cox '91 derivation → [05-position-estimation-cox.md](05-position-estimation-cox.md)
- For frame conventions → [06-coordinate-frames.md](06-coordinate-frames.md)
- For build / run → [07-build-and-run.md](07-build-and-run.md)
