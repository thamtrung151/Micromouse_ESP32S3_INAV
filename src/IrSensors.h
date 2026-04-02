#pragma once

#include <Arduino.h>
#include "config.h"

class IrSensors {
public:
  void begin();
  void update(float dt);
  void enableIR(bool enable);
  bool irEnabled() const;

  // Raw filtered ADC
  uint16_t ir1() const { return _ir1; }
  uint16_t ir2() const { return _ir2; }
  uint16_t ir3() const { return _ir3; }
  uint16_t ir4() const { return _ir4; }

  // Distance in mm (already shifted to robot rotation center)
  float ir1_mm() const;
  float ir2_mm() const;
  float ir3_mm() const;
  float ir4_mm() const;
  float frontAvgMm() const { return 0.5f * (ir2_mm() + ir3_mm()); }

  bool leftWall() const;
  bool rightWall() const;
  bool frontWall() const;
  bool frontWallAt(float thMm) const;

  struct WallObservation {
    bool valid = false;
    bool dualWall = false;
    float errorMm = 0.0f; // Signed wall error in mm. Positive means turn +deg.
  };

  struct WallCorrectionSample {
    bool valid = false;
    bool dualWall = false;
    float errorMm = 0.0f;        // Signed error in mm. Positive means turn +deg.
    int16_t headingDeltaDeg = 0; // Legacy cell-kick output, kept for compatibility.
    bool restoreNextCell = false;
  };

  WallObservation observeWallError() const;
  WallCorrectionSample captureWallCorrection() const;
  int16_t wallYawBias10(uint16_t straightPwmCmd, float dt) const;
  void resetWallController() const;

private:
  static uint16_t clampU16(int v) {
    if (v < 0) return 0;
    if (v > 8191) return 8191;
    return (uint16_t)v;
  }

  uint16_t _ir1 = 8191;
  uint16_t _ir2 = 8191;
  uint16_t _ir3 = 8191;
  uint16_t _ir4 = 8191;

};
