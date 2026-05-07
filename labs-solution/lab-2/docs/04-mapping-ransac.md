# 04 — Mapping via Iterative RANSAC

The reference map is a `std::vector<LineSeg>` extracted from the averaged
calibration scan by iterative RANSAC line fitting in
[src/mapping.cpp](../src/mapping.cpp).

## Why RANSAC and not a rectangle fit?

The previous version assumed the arena was a rectangle and fit four sides.
That collapses for any environment that isn't a clean box — open doors,
columns, tables, irregular rooms, partial walls. RANSAC discovers
whatever lines actually have point support, with no prior on the count or
geometry.

It is also more robust: the input cloud has outliers (stray returns,
specular reflections, glass), and a least-squares fit to all points
biases toward those outliers. RANSAC's whole job is to throw them out.

## Algorithm

For one line:

1. **Hypothesis**: pick two distinct points `p_i`, `p_j` uniformly at
   random. They define a candidate line.
2. **Score**: count how many points in the cloud lie within
   `inlier_thresh_mm` perpendicular distance of that line.
3. **Repeat** for `max_iters_per_line` trials, keep the best inlier set.
4. **Refit**: total-least-squares on the inliers (eigendecomposition of
   the 2×2 covariance about the centroid). This gives a much better line
   than the original two-point sample.
5. **Re-collect inliers** around the refit. Some new points may now fall
   inside the gate; some old ones may have drifted out.
6. **Endpoints**: project each refined inlier onto the line direction,
   take the `min` and `max` projection; pad each end by
   `endpoint_padding_mm`.

For the whole cloud:

7. Remove the inliers from the working set.
8. Stop when the remaining set has fewer than `max(min_inliers,
   min_inlier_fraction × initial_size)` points or `max_lines` is reached.

The walls come out roughly in order of point support — the longest wall
has the most inliers and is found first.

## Why total-least-squares?

The classical `y = m·x + b` least-squares minimises **vertical** residuals.
For a near-vertical wall (`m → ∞`) the fit is ill-conditioned and explodes.
TLS minimises the **perpendicular** residual, which is symmetric in `x` and
`y` and works for any orientation:

```
   residual_i = | n · (p_i - centroid) |
   minimise   Σ residual_i²
```

The 2×2 sample covariance about the centroid is

```
   C = [ Σ(dxᵢ)²    Σ(dxᵢ dyᵢ) ]
       [ Σ(dxᵢ dyᵢ) Σ(dyᵢ)²    ]
```

`C` is symmetric positive semi-definite, so `Eigen::SelfAdjointEigenSolver`
returns real eigenvalues in ascending order. The eigenvector of the
**smaller** eigenvalue is the direction of minimum spread — that's the
line normal. The eigenvector of the **larger** eigenvalue is the line
direction. This is identical to running PCA on the inlier set and taking
the first principal component as the line.

Implementation:

```c++
Eigen::Matrix2d C;
C << sxx, sxy,
     sxy, syy;
Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
LineFit f;
f.cx = cx; f.cy = cy;
f.nx = es.eigenvectors()(0, 0);   // smaller eigenvalue, col 0  → normal
f.ny = es.eigenvectors()(1, 0);
f.dx = es.eigenvectors()(0, 1);   // larger eigenvalue, col 1   → direction
f.dy = es.eigenvectors()(1, 1);
```

## Endpoint extraction and padding

After refit, each inlier's projection onto the line direction is

```
   t_k = d · (p_k - centroid)
```

`min(t)` and `max(t)` give the two endpoints; padding by
`endpoint_padding_mm` (default 50 mm) gives a small safety margin so
points that hit just past the last inlier — e.g. near a corner — still
project onto the segment under the matcher's slack test.

```c++
double t_min =  ∞,
       t_max = -∞;
for (int k : refined) {
    const double t = fit.dx*(p[k].x - fit.cx) + fit.dy*(p[k].y - fit.cy);
    t_min = min(t_min, t);
    t_max = max(t_max, t);
}
t_min -= endpoint_padding_mm;
t_max += endpoint_padding_mm;
seg.ax = fit.cx + t_min * fit.dx;
seg.ay = fit.cy + t_min * fit.dy;
seg.bx = fit.cx + t_max * fit.dx;
seg.by = fit.cy + t_max * fit.dy;
```

## Normal orientation convention

We want every wall's normal to point **away from the sensor at origin**.
That makes the residual sign meaningful (positive ⇒ scan ray ended past
the wall, negative ⇒ short of the wall) and avoids per-iteration sign
flips inside the matcher.

`r = n · centroid` is the perpendicular distance from origin to the line
along `n`. If it's negative, flip `n` and `r`:

```c++
if (r < 0.0) { nx = -nx; ny = -ny; r = -r; }
```

For a sensor sitting inside a closed room, the centroid of any wall's
inlier set is always on the outside of the sensor — `r` is always
positive after this flip. (If the sensor were exactly on the line, the
flip is undefined, but that's not a valid configuration.)

## Stopping conditions

```c++
const size_t hard_floor = max(
    max(2, min_inliers),
    ceil(min_inlier_fraction * initial_size));

while (remaining.size() >= hard_floor &&
       walls.size() < max_lines) { ... }
```

We use the **initial** size (not the dwindling `remaining`) for the
fraction check. Otherwise the fraction goalpost moves every iteration
and the loop happily extracts noise lines once the real walls are
consumed. Tying the floor to the initial size makes the meaning
"any line worth extracting must explain at least 5% of the original
cloud".

## Determinism

By default `RansacParams::seed = 0xC0FFEE` so calibration runs are
reproducible across program invocations on the same lidar capture. Pass
`seed = 0` to opt into `std::random_device` for genuine randomness.

This matters for the lab grading run: same calibration data → same
reference map → same pose estimates. Easier to debug and reason about.

## Cost

For `N` points, one outer iteration is roughly:

| Step                    | Cost                                    |
|-------------------------|-----------------------------------------|
| Inner trials × N        | `max_iters_per_line · N`  perpendicular distance evaluations |
| TLS refit               | `O(|inliers|)` for covariance + 2×2 eigendecomp |
| Inlier rebuild          | `O(N)`                                  |
| Erase inliers           | `O(N)` (rebuild remaining)              |

For `N = 360` points and `max_iters_per_line = 200`, that's ~70 k
distance computations per line × ~5 walls = ~350 k double multiplies.
On the Pi 3B that's a few milliseconds; calibration is a one-shot
operation, so this is not a hot path. Simplicity beats raw speed here.

If you ever need to run RANSAC at scan-rate (10 Hz) on a much larger
cloud, you can:

- pre-allocate `trial_inliers` outside the inner loop,
- use a local-window probabilistic search (sample `j` near `i` in scan
  order — most walls are angularly contiguous),
- cap inlier checks at the first 80% of the cloud and accept early.

None of those tricks are worth the readability cost in the current code.
