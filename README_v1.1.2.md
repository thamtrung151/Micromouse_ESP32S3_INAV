# micromouse v1.1.2

## Wall correction changes
- Removed continuous wall-correction PID behavior.
- Added cell-based wall correction capture using side IR distances in mm.
- Correction is now decided once per cell center (~180 mm), while straight motion remains driven by encoder + IMU heading hold.
- Small error: apply immediate heading shift `a * error_mm` and keep the new heading.
- Large error: apply immediate heading shift `b * error_mm`, drive diagonally for one cell, then restore heading at the next cell center.
- Single-wall mode uses 80 mm target from robot center to wall.
- All correction angles are rounded to integer degrees before being applied.

## New tuning constants
See `src/config.h`:
- `WALL_CORR_GAIN_A_DEG_PER_MM`
- `WALL_CORR_GAIN_B_DEG_PER_MM`
- `WALL_CORR_BOTH_SMALL_ERR_MM`
- `WALL_CORR_ONE_SMALL_ERR_MM`
- `IR_SIDE_SETPOINT_MM`
- `WALL_CORR_MAX_DEG`

## Notes
- Current defaults are conservative starter values only.
- Because IMU resolution is effectively 1 degree, all wall-correction steps are intentionally quantized to integer degrees.
