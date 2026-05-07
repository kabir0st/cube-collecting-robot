# Lab 2 — Lidar Localization

A single-binary C++11 program that:

1. Sits the RPLIDAR A1 still for 5 s, learns its noise level (`σ_r`), and
   extracts a wall map from the averaged scan via iterative RANSAC.
2. Then matches each new revolution against that map at 1 Hz using
   Cox '91 line-based scan matching, printing pose `(x, y, θ)` and
   uncertainty `(σ_x, σ_y, σ_θ)`.

Hardware target: Raspberry Pi 3B running Linux. Single thread of compute,
two background threads for I/O.

## Reading order

If you're new to the codebase, read these in order:

1. **[01-architecture.md](01-architecture.md)** — phase split, threads,
   data shapes, file map. **Start here.**
2. **[02-lidar-protocol.md](02-lidar-protocol.md)** — RPLIDAR packet
   format, control protocol, server lifecycle.
3. **[03-calibration.md](03-calibration.md)** — pooled sigma estimation
   and how the reference map is built.
4. **[04-mapping-ransac.md](04-mapping-ransac.md)** — iterative RANSAC,
   total-least-squares refit, endpoint extraction.
5. **[05-position-estimation-cox.md](05-position-estimation-cox.md)** —
   Cox '91 derivation, Newton iteration, residual gate, covariance.
6. **[06-coordinate-frames.md](06-coordinate-frames.md)** — sign
   conventions, transforms, units.
7. **[07-build-and-run.md](07-build-and-run.md)** — practical: build
   commands, runtime expectations, common issues.

## Code map

```
include/
  global.hpp              one shutdown atomic
  types.hpp               Pose2D, LineSeg, LidarDataFrame, port consts
  server.hpp              LidarDataServer (TCP listener + control)
  utils.hpp               send_control_signal, ListenerThread (RAII)
  loging.hpp              visualizer feed
  calibrate.hpp           LidarCalibration (sigma + reference map)
  mapping.hpp             Point, RansacParams, fit_lines_ransac
  estimate_position.hpp   PositionEstimator, do_estimate_position

src/
  server.cpp              TCP I/O + RPLIDAR packet decode
  utils.cpp               control signal sender
  loging.cpp              visualizer worker thread
  calibration.cpp         pooled sigma, RANSAC driver, prints
  mapping.cpp             RANSAC + TLS line fit
  estimate_position.cpp   Cox matcher + 1 Hz driver

main.cpp                  phase orchestration
```

## At a glance

| Concept                   | File                                                       | Class / function              |
|---------------------------|------------------------------------------------------------|-------------------------------|
| Phase orchestration       | [main.cpp](../main.cpp)                                    | `main`, `do_calibration`      |
| TCP listener for lidar    | [src/server.cpp](../src/server.cpp)                        | `LidarDataServer::start_stream` |
| Control signals to driver | [src/utils.cpp](../src/utils.cpp)                          | `send_control_signal`         |
| Listener thread (RAII)    | [include/utils.hpp](../include/utils.hpp)                  | `ListenerThread`              |
| Visualizer feed           | [src/loging.cpp](../src/loging.cpp)                        | `Loging`                      |
| Sigma estimation          | [src/calibration.cpp](../src/calibration.cpp)              | `LidarCalibration::calibrate` |
| Reference map builder     | [src/calibration.cpp](../src/calibration.cpp)              | `LidarCalibration::create_reference_map` |
| RANSAC + TLS              | [src/mapping.cpp](../src/mapping.cpp)                      | `fit_lines_ransac`, `refit_tls` |
| Cox matcher (one solve)   | [src/estimate_position.cpp](../src/estimate_position.cpp)  | `PositionEstimator::estimate` |
| Wall picker               | [src/estimate_position.cpp](../src/estimate_position.cpp)  | `nearest_wall`                |
| 1 Hz print loop           | [src/estimate_position.cpp](../src/estimate_position.cpp)  | `do_estimate_position`        |
