#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"

static inline int16_t wrapYaw10(int32_t yaw10) {
  yaw10 %= YAW_WRAP;
  if (yaw10 < 0) yaw10 += YAW_WRAP;
  return (int16_t)yaw10;
}

static inline int16_t yawDiff10(int16_t target, int16_t current) {
  int32_t d = (int32_t)target - (int32_t)current;
  while (d >  (YAW_WRAP / 2)) d -= YAW_WRAP;
  while (d < -(YAW_WRAP / 2)) d += YAW_WRAP;
  return (int16_t)d;
}

static inline int16_t fuseYaw10_Circular(int16_t yawA10, float wA, int16_t yawB10, float wB) {

  float a = (yawA10 / 10.0f) * (PI_F / 180.0f);
  float b = (yawB10 / 10.0f) * (PI_F / 180.0f);

  float x = wA * cosf(a) + wB * cosf(b);
  float y = wA * sinf(a) + wB * sinf(b);

  float ang = atan2f(y, x);
  float deg = ang * (180.0f / PI_F);
  int32_t yaw10 = (int32_t)lroundf(deg * 10.0f);
  return wrapYaw10(yaw10);
}
