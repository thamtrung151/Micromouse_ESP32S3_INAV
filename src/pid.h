#pragma once
#include <Arduino.h>

struct PID {
  float kp, ki, kd;
  float integ = 0.0f;
  float prev  = 0.0f;
  bool first  = true;

  PID() = default;
  PID(float p, float i, float d) : kp(p), ki(i), kd(d) {}

  void reset() { integ = 0; prev = 0; first = true; }

  float update(float err, float dt) {
    if (dt <= 0) return 0;
    if (first) { prev = err; first = false; }

    integ += err * dt;

    // anti-windup clamp (giữ giống style bạn dùng)
    const float I_LIM = 200000.0f;
    if (integ >  I_LIM) integ =  I_LIM;
    if (integ < -I_LIM) integ = -I_LIM;

    const float deriv = (err - prev) / dt;
    prev = err;

    return kp * err + ki * integ + kd * deriv;
  }
};
