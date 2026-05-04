# Calibration Logic — Fix Report

## Scope

Reviewed `LidarCalibration::calibrate()` in [src/calibration.cpp](src/calibration.cpp), used by [main.cpp](main.cpp) to compute a single sensor-noise figure from a stationary lidar capture (every angle bin holds repeated distance measurements of the same physical surface).

## Summary of changes

| # | Defect (before)                                                                 | Fix (after)                                                                                |
|---|---------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------|
| 1 | Returned **variance** (mm²) but was named "sigma" / printed as "Sensor Error"   | Returns **standard deviation** (mm) via `std::sqrt`; print label updated                   |
| 2 | Single-sample bins (n = 1) silently contributed 0, biasing σ downward           | Bins with `n < 2` are skipped                                                              |
| 3 | Numerator skipped empty bins, denominator was `scan_collection.size()` (all)    | Pooled estimator: degrees-of-freedom denominator only counts contributing samples          |
| 4 | Divide-by-zero when nothing was collected (returned NaN/Inf silently)           | Guard returns `0.0f` if no usable degrees of freedom                                       |
| 5 | Population variance (`1/n`) used for what is conceptually a sample estimate     | Bessel's correction baked into the pooled form (`Σ(nᵢ−1)` denominator)                     |
| 6 | Naïve `Σx²/n − (Σx/n)²` — catastrophic cancellation for stationary lidar        | Two-pass: compute mean first, then `Σ(d − μ)²`                                             |
| 7 | Unweighted mean across bins — a 3-sample bin counted as much as a 300-sample bin | Pooled across bins: `σ = √( Σᵢ Σⱼ (dᵢⱼ − μᵢ)² / Σᵢ (nᵢ − 1) )`                              |
| 8 | `int * int` for `d * d` (overflow risk above ~46 m if units ever changed)       | `static_cast<double>(d) − mean` keeps the multiply in double                               |

## Final formula

For each angle bin `i` with samples `dᵢⱼ` and bin mean `μᵢ`:

```
        ┌  Σᵢ Σⱼ (dᵢⱼ − μᵢ)²  ┐
σ = √  │  ─────────────────  │
        └    Σᵢ (nᵢ − 1)      ┘
```

Bins with fewer than 2 samples are excluded from both sums. This is the standard pooled sample-standard-deviation across groups, weighted by degrees of freedom — appropriate when each angle bin observes a different (unknown) true distance but the underlying sensor noise is assumed identical across bins.

## Files changed

- [src/calibration.cpp](src/calibration.cpp) — rewrote `LidarCalibration::calibrate()` (two-pass, n≥2 filter, pooled aggregation, sqrt at end, divide-by-zero guard).
- [main.cpp](main.cpp) — renamed local `lidar_sensor_error` → `lidar_sensor_sigma`; print label now reads `Lidar sensor noise σ = … mm` so the units match the returned value.

The `lidar_sigma` field in [include/calibrate.hpp](include/calibrate.hpp#L13) is currently unused; left untouched since the name is now accurate if it is wired up later.

## Verification

- `make clean && make` — builds cleanly, no warnings under `-Wall -Wextra`.
- Behavioural sanity checks (by inspection of the new code):
  - Empty `scan_collection` → returns `0.0f` (was NaN).
  - All bins with n = 1 → returns `0.0f` (was 0, but for the wrong reason — would have been `0/0` if even one bin had been empty).
  - Stationary capture with constant distance per bin plus i.i.d. Gaussian noise → recovers the true σ rather than σ² and without low-bias from singleton bins.
- End-to-end smoke run (`./app` against the lidar) was **not** executed here — requires the physical sensor on the configured TCP endpoint. Recommend running on hardware and sanity-checking that σ comes out in the expected 5–30 mm range for the unit in use.

## Not changed (intentionally)

- Aggregation strategy: still per-angle binning. This assumes the lidar is truly stationary during calibration; if the platform drifts, even pooled variance overestimates noise. Out of scope for a numerical-correctness fix.
- `LidarCalibration::process_point` filter (`quality > 0`) — unchanged. Worth revisiting if the sensor reports a richer quality scale, but that is a sensor-model question, not a math bug.
