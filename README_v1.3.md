# micromouse v1.3

## Safe upgrade A: slow wall correction, fast heading loop
This version is based on the uploaded `micromousev1.1.2-new.zip`.

What changed:
- Kept the existing AUTO straight / AUTO speed PI structure.
- Replaced the cell-kick wall correction in `AutoRunner` with a slow heading-bias layer.
- `Motion::tickAutoStraight()` stays as the fast loop.
- `AutoRunner` now updates a slow wall bias from IR and adds it on top of the straight base heading.
- Wall correction only refreshes every `WALL_CORR_UPDATE_MS` and only inside the mid-cell window.
- Outside the valid correction window, or when wall readings are not trusted, the bias decays back toward zero.
- Side IR observations are filtered by a trusted distance range before wall correction is applied.

Main files changed:
- `src/AutoRunner.h`
- `src/AutoRunner.cpp`
- `src/IrSensors.h`
- `src/IrSensors.cpp`
- `src/config.h`
- `config.h`

Main tuning constants:
- `WALL_CORR_UPDATE_MS`
- `WALL_CORR_PHASE_START_MM`
- `WALL_CORR_PHASE_END_MM`
- `WALL_CORR_DUAL_GAIN_DEG_PER_MM`
- `WALL_CORR_SINGLE_GAIN_DEG_PER_MM`
- `WALL_CORR_BIAS_MAX_DEG`
- `WALL_CORR_SLEW_DEG_PER_S`
- `WALL_CORR_DECAY_DEG_PER_S`
- `WALL_CORR_VALID_MIN_MM`
- `WALL_CORR_VALID_MAX_MM`
