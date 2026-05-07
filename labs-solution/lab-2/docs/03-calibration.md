# 03 — Calibration

Calibration runs once at program start, with the lidar sitting still, and
produces two outputs:

1. **`σ_r`** — the standard deviation of distance measurements (mm).
2. **Reference map** — the wall geometry (`std::vector<LineSeg>`).

It lives in [include/calibrate.hpp](../include/calibrate.hpp) and
[src/calibration.cpp](../src/calibration.cpp).

## Data collection

The lidar reports samples roughly uniformly in angle. While stationary,
every revolution observes the same physical surface at each bearing, so
distance returns at angle `a` are independent samples of one underlying
"true distance" — perfect for noise estimation.

`process_point` bins them by integer-degree angle:

```c++
void LidarCalibration::process_point(int quality, int angle, int distance) {
    if (quality <= 0 || distance <= 0) return;     // drop bad returns
    {
        std::lock_guard<std::mutex> g(mx_);
        scan_collection_[angle].push_back(distance);
    }
    if (logger_) logger_->collect(angle, distance);
}
```

`scan_collection_` ends up looking like:

```
{ 0:   [ 1842, 1838, 1841, 1839, ... ],
  1:   [ 1851, 1847, 1849, ...        ],
  ...
  359: [ 1832, 1834, ...               ] }
```

## Sigma estimation

Each angle bin observes a *different* (unknown) true distance — wall A is
at 1.84 m, wall B is at 2.51 m, and so on. We can't pool the raw samples
directly; we have to subtract each bin's mean first. The standard
construction is the **pooled sample standard deviation across groups**:

```
        ┌  Σᵢ Σⱼ (dᵢⱼ - μᵢ)²  ┐
σ = √   │  ─────────────────  │
        └    Σᵢ (nᵢ - 1)      ┘
```

where `i` indexes angle bins and `j` indexes samples within a bin. Bins
with fewer than 2 samples (no degrees of freedom) are skipped.

Implementation uses Welford's one-pass algorithm to compute each bin's
mean and `Σⱼ (dᵢⱼ - μᵢ)²` (`m2`) without storing the running mean²:

```c++
double mean = 0.0, m2 = 0.0;
size_t n = 0;
for (int d : samples) {
    ++n;
    const double delta = d - mean;
    mean += delta / static_cast<double>(n);   // updated mean
    m2   += delta * (d - mean);               // updated sum of squared dev
}
sum_sq_dev += m2;
total_dof  += n - 1;
```

Welford avoids the catastrophic cancellation of `Σx² − (Σx)²/n` when the
samples cluster tightly (which they do for a stationary lidar — many
returns within mm of each other).

The result is one floating-point value, in mm. For our RPLIDAR A1 it
typically lands around 25–35 mm. The matcher uses it as the `σ_r` in:

- the residual gate `gate = clamp(k * σ_r, 10, 150)` for inlier selection,
- the covariance scaling `cov = (RSS/dof) * (JᵀJ)⁻¹`.

## Reference map

After 5 s of capture, `create_reference_map()` averages each bin to a
single distance, projects to cartesian, drops the very-near-zone noise
(< 150 mm), and runs the cloud through RANSAC:

```c++
const double avg = sum(samples) / samples.size();
if (avg < 150.0) continue;
const double a = bin_angle_deg * M_PI / 180.0;
pts.emplace_back(avg * cos(a), avg * sin(a));
```

```c++
RansacParams ransac;
ransac.inlier_thresh_mm = std::max(15.0, 3.0 * (double)lidar_sigma_);
std::vector<LineSeg> walls = fit_lines_ransac(pts, ransac);

if (walls.empty())
    throw std::runtime_error("create_reference_map: RANSAC found no walls");
```

Threshold rationale: 3σ is the conventional Mahalanobis-equivalent
inlier gate for Gaussian residuals. The 15 mm floor protects against
unusually low sigma (very noiseless lidar or very few samples in
calibration), which would otherwise fragment a single physical wall
into multiple collinear RANSAC segments.

## Why throw on empty walls?

If RANSAC returns nothing, the matcher's `nearest_wall` will always
return -1, every estimate yields `n_in == 0`, and the program prints
"NOT-CONVERGED" forever with no clue why. A loud failure beats a silent
useless run. The user can then check the lidar mount, the room (is it
empty? are there only soft surfaces?), or `σ_r`.

## Threading

```
listener thread          main thread
─────────────────        ────────────────────────────
process_point()
  lock mx_
    insert into bin
  unlock mx_
                         server.stop_stream()  (waits for recv_active)
                         calibrate()
                           lock mx_
                             pool variance
                           unlock mx_
                         create_reference_map()
                           lock mx_
                             snapshot to local pts
                           unlock mx_
                           run RANSAC on pts
```

By the time `calibrate()` runs, the listener thread is no longer calling
`process_point` (`stop_stream()` blocked until that was true). The mutex
is still acquired in calibrate/create_reference_map for two reasons:

1. Defence in depth — atomic-flag synchronization can be subtle, and a
   future change might break that invariant.
2. `create_reference_map` builds `pts` while holding the lock, which keeps
   the snapshot consistent if anyone ever adds a "re-calibrate at runtime"
   path.

The lock is uncontended in steady state, so the cost is a single
uncontended atomic exchange — negligible.

## Configuration knobs

There are very few tunables, all in code:

| Where                                         | Knob                       | Default | Effect of raising                          |
|-----------------------------------------------|----------------------------|---------|--------------------------------------------|
| `do_calibration` ([main.cpp](../main.cpp))    | `sleep_for(seconds(5))`    | 5 s     | more samples per bin → more reliable σ      |
| `create_reference_map` ([calibration.cpp](../src/calibration.cpp)) | `if (avg < 150.0)`         | 150 mm  | filters more near-zone noise; loses close walls |
| `RansacParams::inlier_thresh_mm`              | `max(15, 3·σ)`             | runtime | wider threshold → fewer fragmented walls    |
| `RansacParams::min_inliers`                   | 20                         | const   | rejects more short walls                    |

If you change the inlier threshold formula here, also update the
floor reasoning in [04-mapping-ransac.md](04-mapping-ransac.md).
