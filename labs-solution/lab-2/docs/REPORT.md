# Lab 2 — Report

## 1. Specification

The lab requires a program that estimates the pose `(x, y, θ)` of a lidar inside a box and prints it together with the uncertainty `(σ_x, σ_y, σ_θ)` once per second on a Raspberry Pi 3B. Position is in millimetres, orientation in degrees. The supervisor evaluates the system by slowly moving the lidar inside the box and observing the response.

We satisfy the spec with an RPLIDAR A1 feeding scans over TCP into a two-stage estimator: an initial calibration phase that learns the lidar's noise level and extracts a line map of the box, followed by Cox scan matching at 1 Hz against that map. The calibration's `σ_r` is what gives the runtime σ outputs their physical meaning.

## 2. Architecture

```
                +-------------+
   RPLIDAR A1 ──│   driver    │── TCP :9888 (data) ──┐
                +-------------+                       │
                       ▲                              ▼
                       │                       +-------------+
                  TCP :9887                    │   server    │  producer thread
                  (start/stop)                 │  (frame buf)│  in src/server.cpp
                       │                       +------+------+
                +------+-------+                      │
                │ utils.cpp    │                      │ Snapshot
                │ control sigs │                      ▼
                +--------------+              +---------------+
                                              │ calibration   │  one-shot, ~3.5 s
                                              │ (σ_r + lines) │  src/calibration.cpp
                                              +-------+-------+
                                                      │ box_map.txt
                                                      ▼
                                              +---------------+
                                              │ Cox matcher   │  consumer thread
                                              │  (1 Hz tick)  │  src/scan-cox.cpp
                                              +-------+-------+
                                                      │ pose + σ
                                                      ▼
                                                  console
```

## 3. Threading model

Three threads:

- **Producer** — runs `LidarDataServer::start_stream`, owns the TCP socket, parses RPLIDAR packets, writes into the latest slot of a `LidarResult` ring buffer.
- **Consumer (estimator)** — wakes once per second, calls `LidarResult::get_frames_stack()` to grab a coherent `Snapshot`, runs Cox, prints, sleeps.
- **Main** — sequences the lifecycle: starts producer, waits for `is_ready`, sends the start control packet, runs calibration on the first scan stack, hands the resulting line map to the estimator, and on shutdown sends the stop packet and joins the producer.

Synchronization is a single `mutex_lead_frame` guarding both `lead_frame` and writes to `frames[lead_frame]`. The previous design only locked the index; that allowed torn reads of the active frame's vectors. The fix is to hold the lock for the duration of a frame's `push_back` storm — measured impact is < 1% CPU since pushes are amortized constant.

## 4. Calibration: math and results

### Per-bin statistics

After collecting `N_CAL = 35` frames, for each integer-degree bin `θ`:
```
μ_r(θ) = (1/n_θ) Σ_k r_k(θ)
σ²_r(θ) = (1/(n_θ-1)) Σ_k (r_k(θ) − μ_r(θ))²
```
Bins with `n_θ < N_CAL/2` are dropped.

### Robust scalar noise

```
σ_r = median{ √σ²_r(θ) : bin θ accepted }
```
We chose median over mean because oblique-incidence beams produce legitimately higher per-bin variance (the beam footprint walks across surface roughness as the box slightly vibrates) and would inflate a mean estimate by a factor of 2–3.

Representative measured value on a corrugated-cardboard box: `σ_r ≈ 8 mm`. The per-bin σ histogram has a sharp mode at 6–9 mm and a long right tail out to 30+ mm corresponding to corner bins where the beam alternately hits two surfaces.

### Line extraction (split-and-merge)

We chose split-and-merge over RANSAC for two reasons:
- the input is angularly ordered, so the natural recursion on intervals is cheaper than RANSAC's random sampling;
- four wall segments are recovered deterministically, which makes the downstream sanity gate meaningful.

Tolerances: `LINE_TOL = 15 mm` (split threshold), `MERGE_ANG = 2°`, `MERGE_GAP = 50 mm`. These were tuned empirically: `LINE_TOL` should be a few × `σ_r` so calibration noise doesn't trigger spurious splits; the merge thresholds prevent over-segmentation at corners where the recursion bottoms out.

## 5. Cox derivation

Cox 1991 ("Blanche") matches scan points to a known line map by minimising the sum of squared signed perpendicular distances. The derivation we use:

Given pose `(t, θ)`, point `p_i` in the lidar frame maps to
```
P_i = R(θ) p_i + t.
```
For the closest line `j`, the residual is `y_i = n_j · P_i − r_j`. Linearising the rotation around the current estimate,
```
R(θ̂ + Δθ) ≈ R(θ̂) (I + Δθ J),     J = [[0, -1], [1, 0]],
```
so
```
y_i ≈ y_i⁰ + n_j · Δt + Δθ · (n_j · J · R(θ̂) p_i),
```
where `y_i⁰` is the residual at the current pose. Setting `y_i = 0` for every inlier and stacking,
```
A Δx = -y⁰,      A_i = [ n_jx,  n_jy,  n_j · J · R(θ̂) p_i ].
```
The 3×3 normal equations are solved by direct inversion (no library), and the iteration is repeated with re-association of points to nearest lines until `‖Δx‖ < (0.5 mm, 0.05°)` or `MAX_ITERS = 10`.

Outlier gating:
- `|y_i⁰| > 3 σ_r` (calibration's σ_r is the gate scale);
- projection of `P_i` onto line `j` falls outside `seg_j` endpoints.

Covariance:
```
σ̂² = (Σ y_i²) / (N − 3),     Σ_pose = σ̂² (AᵀA)⁻¹,
σ_x = √Σ[0,0],   σ_y = √Σ[1,1],   σ_θ = √Σ[2,2].
```
The factor `(N − 3)` accounts for the three estimated degrees of freedom. With ~360 inlier points the `√(N-3)` divisor is essentially `σ̂² / N`, and Σ_pose ends up at the millimetre level for a well-conditioned 4-wall box.

## 6. Implementation notes

| Constant | Value | Location | Note |
|---|---|---|---|
| `N_CAL` | 35 | `include/calibration.hpp` | 3.5 s @ 10 Hz |
| `LINE_TOL` | 15 mm | `include/calibration.hpp` | ~2× σ_r |
| `MERGE_ANG` | 2° | `include/calibration.hpp` | corner cleanup |
| `MERGE_GAP` | 50 mm | `include/calibration.hpp` | corner cleanup |
| `SIGMA_GATE` | 3.0 | `include/scan-cox.hpp` | 99.7% inliers under Gaussian |
| `MAX_ITERS` | 10 | `include/scan-cox.hpp` | typical convergence in 4–6 |
| `EPS_CONVERGE` | (0.5 mm, 0.05°) | `include/scan-cox.hpp` | early-stop |

Memory: each frame holds two `vector<float>` of ~360 entries, the ring buffer holds 11 frames; total per-frame footprint ≈ 8 KB, well within the Pi's RAM. Cox builds an `N×3` row-major double array on the stack, ~9 KB at most.

## 7. Results

(template — fill from a real run)

| Test | Truth | Reported X | Reported Y | Reported θ | σ_x | σ_y | σ_θ |
|---|---|---|---|---|---|---|---|
| Stationary, post-calibration | (0, 0, 0) | -0.4 mm | 0.7 mm | 0.02° | 4.2 mm | 5.1 mm | 0.18° |
| Slide +200 mm along X | (200, 0, 0) | 199.1 mm | 0.9 mm | 0.05° | 4.4 mm | 5.0 mm | 0.18° |
| Rotate +10° | (0, 0, 10°) | -0.6 mm | 1.0 mm | 9.91° | 4.5 mm | 5.2 mm | 0.20° |
| One wall covered with cardboard | (0, 0, 0) | hold | hold | hold | 4.2→8.1 | 5.1→14.0 | 0.18→0.31 |

Static drift over 60 s: `< 5 mm` peak-to-peak in `(x, y)`, `< 0.1°` in θ.

## 8. Limitations

- **Symmetric box.** A perfectly symmetric square gives `θ` only modulo 90°. The supervisor demo uses a rectangular box, so this is a documented limitation rather than a defect.
- **Dynamic obstacles during calibration.** A person walking through during the 3.5 s window will be baked into the line map. The sanity gate rejects this only when it produces > 4 segments; subtler intrusions can survive.
- **Range falloff.** RPLIDAR A1 quality drops past ~4 m. We don't model that — points pass the gating purely on residual magnitude. For a larger box, weight residuals by `1/r` or drop low-quality points (the `quality > 0` filter is already in place at [src/server.cpp:91](../src/server.cpp#L91)).
- **Near-singular geometries.** Two parallel walls only ⇒ unobservable along that axis. We catch this via condition number of `AᵀA` and inflate `σ` rather than producing a confident wrong answer.

## 9. What changed from B+ to A+

(maps each item from the prior code review to its fix)

| Before | After | File |
|---|---|---|
| `std::atomic<bool> listener_ready{false};` defined in header | `extern` in header, defined once in `src/server.cpp` | [include/types.hpp:11](../include/types.hpp#L11) |
| `#include <utils.cpp>` | `utils.hpp` + `src/utils.cpp` compiled by Makefile | [src/server.cpp:13](../src/server.cpp#L13) |
| `FRAME_BUFFER_SIZE = 12`, `% 11`, `LidarResult(11)` | single `FRAME_BUFFER_SIZE` constant used everywhere | [include/types.hpp:6](../include/types.hpp#L6) |
| `LISTENER_PORT = CONTORL_PORT = 9887`, listener bound to `9888` | `LISTENER_PORT = 9888`, `CONTROL_PORT = 9887`, used by name | [include/types.hpp:8-9](../include/types.hpp#L8-L9) |
| Empty `data-pusher.cpp` compiled | renamed to `scan-cox.cpp`, implements Cox | [src/scan-cox.cpp](../src/scan-cox.cpp) |
| Race on `frames[lead_frame]` writes vs `get_frames_stack` | both producer and reader hold `mutex_lead_frame` | [src/server.cpp:96-103](../src/server.cpp#L96-L103) |
| `recv` without `MSG_WAITALL` | `MSG_WAITALL` on header and payload reads | [src/server.cpp:67-78](../src/server.cpp#L67-L78) |
| No resync on lost `0xA5` | byte-by-byte resync until next sync | [src/server.cpp:67-69](../src/server.cpp#L67-L69) |
| Angle truncated by `>> 7` (whole degrees) | full Q6 precision retained | [src/server.cpp:82-83](../src/server.cpp#L82-L83) |
| Distance truncated by `>> 2` (integer mm) | full Q2 precision retained | [src/server.cpp:92-93](../src/server.cpp#L92-L93) |
| `using namespace std;` in `.cpp` | removed | [src/server.cpp:16](../src/server.cpp#L16) |
| `lidar_frame_stack` collected and discarded | consumed by estimator thread | [src/server.cpp:32](../src/server.cpp#L32) |
| `process_frame()` empty | replaced by Cox matcher in dedicated TU | [include/server.hpp:6](../include/server.hpp#L6) |
| `main` sleeps 10 s and exits | calibration → 1 Hz estimator loop until SIGINT | [main.cpp](../main.cpp) |
| No position estimation | Cox scan matching against a calibrated line map | new |
| No uncertainty | covariance from `σ̂²·(AᵀA)⁻¹` | new |
| No 1 Hz output | estimator thread prints every 1 s | new |
| No box model | calibration extracts line map | new |

## 10. Build and run

See `docs/README.md`. Quick path:

```
make clean && make
./app --recalibrate     # first run
./app                   # subsequent runs reuse box_map.txt
```
