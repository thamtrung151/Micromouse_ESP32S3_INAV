# Micromouse (ESP32-S3, Arduino/PlatformIO)

This project is an upgraded version of your **`micromouse-sript-run`** firmware:

* Keeps the existing **scripted motion** mode (IMU + encoder fusion + motion primitives).
* Adds a basic **AUTO micromouse** mode that uses **BFS flood-fill** (unknown edges are treated as open, NOT walls).
* Integrates **4 IR ADC sensors** for:
  * wall detection (left/right/front)
  * wall-centering correction while moving straight.
* Adds **IO11 Start/Stop toggle button** (active LOW, internal pull-up).

## Build mode selection

In `src/config.h`:

```c++
#define MODE_SCRIPT 1
#define MODE_AUTO   2

#ifndef RUN_MODE
  #define RUN_MODE MODE_AUTO
#endif
```

Set `RUN_MODE` to `MODE_SCRIPT` or `MODE_AUTO`.

## Hardware pins

* IR ADC pins:
  * IR1 (left side)  = GPIO4
  * IR2 (front-left) = GPIO5
  * IR3 (front-right)= GPIO6
  * IR4 (right side) = GPIO7
* Start/Stop button (active LOW) = GPIO11

## IR thresholds (default)

Your measurements (13-bit ADC):

* wall present: ~350–400
* no wall: ~4000–7000

Defaults in `src/config.h`:

* `IR_SIDE_WALL_TH  = 1200`
* `IR_FRONT_WALL_TH = 1200`

Tune on your maze if lighting/height changes.

## Wall correction (centering)

During straight motion, a small yaw bias is added:

* If **both** side walls exist: use difference `(IR4 - IR1)`.
* If only one wall exists: track the single-wall setpoint `IR_SIDE_SETPOINT`.

Key parameters in `src/config.h`:

* `WALL_CORR_ENABLE`
* `IR_SIDE_SETPOINT`
* `WALL_BOTH_K_YAW10_PER_COUNT`
* `WALL_ONE_K_YAW10_PER_COUNT`
* `WALL_CORR_MAX_YAW10`

## AUTO solver behavior

* Maze size: 16x16
* Goal region: (7,7), (7,8), (8,7), (8,8)
* At each cell center:
  1. sense `front/left/right` walls via IR
  2. update maze map
  3. run BFS flood-fill to the center
  4. choose the neighbor with smallest distance (tie-break: forward, left, right, back)

Unknown edges are treated as open (no "unknown treated as wall").

## IR standalone test

If you want to verify raw ADC readings quickly:

*Define* `IR_STANDALONE_TEST` (e.g., in PlatformIO build_flags) and the firmware will print CSV:

```text
ir1,ir2,ir3,ir4
```
