# 07 — Build & Run

## Dependencies

- g++ with C++11 (the project uses none of the C++14+ niceties).
- Eigen 3 headers, expected at `/usr/include/eigen3` (Debian/Ubuntu/Raspbian
  package `libeigen3-dev`).
- OpenCV 4 (linked but not actually used in the hot path; transitively
  pulled in by the Makefile's link flags). Package `libopencv-dev`.
- A POSIX threading runtime (`-pthread`).
- The RPLIDAR vendor driver, listening on TCP `127.0.0.1:9888` for our
  connect and on `127.0.0.1:9887` for control bytes. The driver is in
  `rplidar/`; build it separately and run it before our binary.

## Build

```sh
make           # produces ./app
make clean     # removes build/ and ./app
make info      # prints resolved CXXFLAGS / LDFLAGS / source list
```

The Makefile compiles with `-O3 -ffast-math -Wall -Wextra` and detects
`aarch64` / `armv7l` to add `-mcpu=cortex-a53` (and NEON for armv7l).

There are no incremental-build pitfalls — every `.cpp` builds to
`build/*.o` with `-MMD -MP` for include-tracking, and the link step
gathers all of them.

A clean build emits one warning from inside Eigen's `SelfadjointMatrixVector`
header (`-Wmaybe-uninitialized`). It is not from this project's code and
does not affect runtime.

## Running

The two-phase program:

```sh
# 1. Start the lidar driver in another terminal:
./rplidar/rplidar_driver           # adjust path to your build
                                   # listens on :9887 and offers :9888

# 2. (optional) Start the visualizer:
python3 visualizer.py              # binds :9890, draws scans + map

# 3. Start us:
./app
```

You'll see something like:

```
[main] starting
[log] connected to 127.0.0.1:9890
[cal] starting . . .
[stream] waiting for lidar driver...
[stream] lidar client connected
[ctrl] START sent
[stream] lidar start signal sent
[ctrl] STOP sent
[stream] lidar stop signal sent
[cal] sigma = 27.4 mm
[map] data points: 358
[map] inlier threshold: 82.2 mm
[map] walls extracted: 4
  W0 n=(-1.0, 0.0) r=2510  ( 2510, -1430) -> ( 2510, 1620)
  W1 n=(0.0, -1.0) r=1430  ...
  ...
[est] starting position estimation . . .
[stream] waiting for lidar driver...
[stream] lidar client connected
[ctrl] START sent
[pose] x=12.3 y=-5.1 theta=0.4 deg | sigma=(8.2, 7.9, 0.6 deg) inliers=894 iters=3  OK
[pose] x=11.8 y=-5.4 theta=0.4 deg | sigma=(8.0, 7.8, 0.6 deg) inliers=901 iters=2  OK
...
^C
[main] shutting down
```

If the visualizer is not running, you'll see one warning at startup
(`[log] connect ... failed`) and the program continues fine — the logger
becomes a no-op.

## Common runtime issues

| Symptom                                              | Likely cause                                              | Action                                                                                  |
|------------------------------------------------------|-----------------------------------------------------------|------------------------------------------------------------------------------------------|
| `[ctrl] connect to 127.0.0.1:9887 failed`            | RPLIDAR driver isn't running                              | Start the driver first                                                                   |
| `[stream] waiting for lidar driver...` forever       | Driver is up but not connecting back to our :9888         | Check driver config; firewall (unlikely on localhost); port collision                   |
| `RANSAC found no walls` exception                     | Lidar saw no usable returns (lens covered? all near-zone?) | Check the room; check sigma — < 5 mm or > 200 mm both indicate a problem                 |
| Pose prints `NOT-CONVERGED` consistently              | Robot moving faster than the matcher's gate; few inliers  | Slow down, verify map, check `sigma_r_`                                                  |
| `[log] send failed, disabling stream`                 | Visualizer disconnected                                    | Reconnect — this disables the feed but doesn't crash us                                  |

## Testing without the lidar

There is no replay harness in tree, but the matcher is self-contained:
read `lidar_data.txt`, parse it into `(angle, distance)` pairs, and feed
them to a `PositionEstimator::process_point` mock.

A more robust setup would split the listener+driver layer into a
`stdio` adapter so a test binary can pipe canned scans in. That's a
worthwhile improvement, but out of scope for the lab.

## Memory footprint

In steady-state runtime (estimation phase) the program holds:

- `walls_` : ~5 × 28 bytes ≈ 140 bytes
- `frames_` : 3 × ~360 × 8 bytes ≈ 8 KB
- `P_lidar_` + `P_world_` : 2 × 4096 × 4 bytes × 2 = 64 KB
- `Loging::buffer_` : up to ~2048 × 8 bytes ≈ 16 KB before each flush

Total well under 100 KB. No allocations in the hot path.

## Code layout

```
lab-2/
├── Makefile
├── main.cpp
├── include/                  # public headers (one per concept)
├── src/                      # implementations
├── docs/                     # this folder
├── rplidar/                  # vendor driver (built separately)
├── lidar_data.txt            # captured scan log for inspection
├── visualizer.py             # optional Python scope
├── restart.sh                # convenience: kill + rebuild + run
└── install-this              # apt deps for fresh systems
```
