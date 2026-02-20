#include "IrSensors.h"

void IrSensors::begin() {
  analogReadResolution(IR_ADC_BITS);
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);

  // initialize with first read
  _ir1 = (uint16_t)analogRead(IR1_PIN);
  _ir2 = (uint16_t)analogRead(IR2_PIN);
  _ir3 = (uint16_t)analogRead(IR3_PIN);
  _ir4 = (uint16_t)analogRead(IR4_PIN);
}

void IrSensors::update(float dt) {
  if (dt <= 0.0f) return;
  const float alpha = dt / (IR_LP_TAU_S + dt);

  const uint16_t r1 = (uint16_t)analogRead(IR1_PIN);
  const uint16_t r2 = (uint16_t)analogRead(IR2_PIN);
  const uint16_t r3 = (uint16_t)analogRead(IR3_PIN);
  const uint16_t r4 = (uint16_t)analogRead(IR4_PIN);

  // IIR: y += a*(x-y)
  _ir1 = clampU16((int)lroundf((float)_ir1 + alpha * ((float)r1 - (float)_ir1)));
  _ir2 = clampU16((int)lroundf((float)_ir2 + alpha * ((float)r2 - (float)_ir2)));
  _ir3 = clampU16((int)lroundf((float)_ir3 + alpha * ((float)r3 - (float)_ir3)));
  _ir4 = clampU16((int)lroundf((float)_ir4 + alpha * ((float)r4 - (float)_ir4)));
}

int16_t IrSensors::wallYawBias10(uint16_t straightPwmCmd) const {
  if (!WALL_CORR_ENABLE) return 0;
  if (straightPwmCmd < WALL_CORR_MIN_PWM) return 0;

  const bool lw = leftWall();
  const bool rw = rightWall();

  float corr = 0.0f;

  if (lw && rw) {
    // centered when IR1 ~ IR4 => corr=0. If IR4 > IR1 => robot closer to left => steer right (+)
    const int diff = (int)_ir4 - (int)_ir1;
    corr = WALL_BOTH_K_YAW10_PER_COUNT * (float)diff;
  } else if (lw && !rw) {
    // keep left sensor near setpoint (smaller ADC => closer)
    const int e = (int)IR_SIDE_SETPOINT - (int)_ir1;
    corr = WALL_ONE_K_YAW10_PER_COUNT * (float)e; // + => steer right
  } else if (rw && !lw) {
    const int e = (int)IR_SIDE_SETPOINT - (int)_ir4;
    corr = -WALL_ONE_K_YAW10_PER_COUNT * (float)e; // + => too close right, steer left (negative)
  } else {
    corr = 0.0f;
  }

  if (corr > (float)WALL_CORR_MAX_YAW10) corr = (float)WALL_CORR_MAX_YAW10;
  if (corr < -(float)WALL_CORR_MAX_YAW10) corr = -(float)WALL_CORR_MAX_YAW10;
  return (int16_t)lroundf(corr);
}
