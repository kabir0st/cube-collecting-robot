 What was wrong

  Your hue wrap logic was behaving as if it was wrong, but the main bug was actually in S/V thresholding:

   - The lower bounds were computed as:
    - s_lower = max(0, s_min - s_margin)
    - v_lower = max(0, v_min - v_margin)
   - With your example (Selected HSV: (102, 30, 39)), margins pushed both lower bounds to 0.
   - That produced ranges like:
    - Lowerbound HSV: [65, 0, 0]
    - Wraplower HSV: [0, 0, 0]
   - Result: black/dark pixels (S≈0, V≈0) matched the mask regardless of hue.

  So black inclusion was caused by permissive S/V bounds, not by hue wrap splitting itself.

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  Exactly what I changed

  File changed

   - labs-solution/lab-1/src/calibration.cpp

  1) Added dynamic non-black floor for S and V lower bounds

  I introduced:

   - sv_floor_ratio =
    0.35f
   - min_non_black_sv =
    10.f
   - A helper lambda compute_lower_sv(...):

  Logic:

   1. Start with old lower bound: raw_lower = max(0, min_channel - margin).
   2. If selected dominant channel is already near-black (<= 10), keep old behavior.
   3. Otherwise enforce floor:
    - dynamic_floor = max(10, dominant_channel
    
    * 0.35)
    - final_lower = max(raw_lower, dynamic_floor)

  Then:

   - s_lower = compute_lower_sv(s_min, s_margin, dominant_s)
   - v_lower = compute_lower_sv(v_min, v_margin, dominant_v)

  2) Reused those bounds in both normal and wrap ranges

  Previously, both wrap and non-wrap used direct max(0, ...) for S/V lower bounds.
  Now both paths use the computed s_lower and v_lower.

  So both:

   - lower1 (main range)
   - lower2 (wrap range)

  use the same protected S/V floor.

  3) Small compile-fix cleanup

  I removed a duplicate constructor definition from src/calibration.cpp:

   - Removed: CameraConfig::CameraConfig(int w, int h, int f) ...
   - Reason: it was already defined inline in include/types.hpp and caused redefinition on clean build.

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  Why this fixes your black box issue

  Before:

   - Lower S/V could become [0, 0], so black passed.

  After:

   - For non-black selected colors, S/V lower bounds stay above near-black values.
   - Hue still wraps correctly, but now only pixels with sufficient saturation/value are accepted.
   - This keeps dark background/black objects out of the mask.

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  Expected effect on your sample values

  Given dominant approx (H=102.9, S=30.3, V=39.4):

   - New floor targets:
    - S floor ≈ max(10,
    
    30.3 * 0.35) ≈ 10.6
    - V floor ≈ max(10,
    
    39.4 * 0.35) ≈ 13.8
   - So lower bounds no longer collapse to zero.
   - Black (S=0, V=0) no longer matches.

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  Behavior note

  This change preserves intentional behavior for genuinely dark targets: if the selected dominant color is near-black (<=10 in S or V), the logic falls back to old permissive behavior for that channel. So
  dark-object tracking remains possible when intentionally selected.
