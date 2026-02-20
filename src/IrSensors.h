#pragma once

#include <Arduino.h>
#include "config.h"

// Simple 4-channel IR ADC reader with IIR low-pass filtering.
// Sensor characteristic (inverted): wall -> small ADC, open -> large ADC.
class IrSensors {
public:
  void begin();
  void update(float dt);

  // Filtered readings (0..8191 for 13-bit)
  uint16_t ir1() const { return _ir1; } // left side
  uint16_t ir2() const { return _ir2; } // front-left
  uint16_t ir3() const { return _ir3; } // front-right
  uint16_t ir4() const { return _ir4; } // right side

  bool leftWall() const  { return _ir1 < IR_SIDE_WALL_TH; }
  bool rightWall() const { return _ir4 < IR_SIDE_WALL_TH; }
  bool frontWall() const { return (_ir2 < IR_FRONT_WALL_TH) && (_ir3 < IR_FRONT_WALL_TH); }

  // Compute a yaw bias (yaw10) for corridor centering.
  // Positive => steer right; Negative => steer left.
  int16_t wallYawBias10(uint16_t straightPwmCmd) const;

private:
  static uint16_t clampU16(int v) {
    if (v < 0) return 0;
    if (v > 8191) return 8191;
    return (uint16_t)v;
  }

private:
  // filtered values
  uint16_t _ir1 = 8191;
  uint16_t _ir2 = 8191;
  uint16_t _ir3 = 8191;
  uint16_t _ir4 = 8191;
};




