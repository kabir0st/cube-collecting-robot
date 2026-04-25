What was wrong

- Low-saturation / low-value targets (especially black-like regions) were still getting into the mask because S/V bounds could become too permissive.
- Hue wrap-around was triggered by wide hue spread alone, which could accidentally enable wrap logic for non-red colors.

---

What the current code does

File changed:
- `labs-solution/lab-1/src/calibration.cpp`

1. Tightened default margins
- `h_margin = 8.f` (was 10)
- `s_margin = 24.f` (was 30)
- `v_margin = 28.f` (was 40)

2. Keeps dynamic S/V lower floors
- Uses `compute_lower_sv(...)` with:
  - `sv_floor_ratio = 0.35f`
  - `min_non_black_sv = 10.f`
- For non-black targets, this prevents lower S/V from collapsing too close to zero.

3. Wrap-around is now red-only
- Wrap is enabled only if all are true:
  - `(h_max - h_min) > 90.f`
  - hue confidence is high (`dominant_s >= 40` and `dominant_v >= 40`)
  - hue range spans red edges (`h_min <= 25` and `h_max >= 155`)
- This avoids wrap for black/gray and other non-red captures.

---

Why this matches expected behavior

- Black should not require hue wrap-around, because hue is unreliable at low S/V.
- Red is the main HSV case that legitimately crosses `0/180`, so wrap stays enabled only for that case.
- Result: tighter mask, fewer dark false positives, and cleaner tracking.
