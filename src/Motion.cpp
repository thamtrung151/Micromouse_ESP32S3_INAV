#include "Motion.h"

// ================= Stop model helpers =================
// We avoid a single fixed BRAKE_MARGIN_MM because the extra travelled distance
// depends strongly on speed (control period + motor response + PWM slew).
// Model: margin(v) = base + |v| * lag_s
static inline float motionBrakeMarginMm(float vAbs) {
  return BRAKE_MARGIN_BASE_MM + vAbs * BRAKE_LAG_S;
}

// Compute a braking-limited speed target with a creep floor near the goal
// so the robot does not "stall" when friction > commanded torque.
static inline float motionVTarget(float remainMm, float vAbsMm_s, float vMaxMm_s) {
  // Dynamic braking margin
  float remEff = remainMm - motionBrakeMarginMm(vAbsMm_s);
  if (remEff < 0.0f) remEff = 0.0f;

  float vBrake = sqrtf(2.0f * MOVE_A_DEC * remEff);
  float vT = fminf(vMaxMm_s, vBrake);

  // Creep: keep a small but non-zero target speed until we are truly at the goal.
  if (remainMm > POS_TOL_MM && remainMm < CREEP_ZONE_MM) {
    const float denom = (CREEP_ZONE_MM - POS_TOL_MM);
    float t = (denom > 1e-3f) ? ((remainMm - POS_TOL_MM) / denom) : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const float vCreep = V_CREEP_MM_S * t;
    if (vT < vCreep) vT = vCreep;
  }

  // Once inside POS_TOL, we want vTarget=0 and let the stop condition finish.
  if (remainMm <= POS_TOL_MM) vT = 0.0f;
  return vT;
}


// ================= Helper =================
int Motion::rampInt(int cur, int tgt, int step) {
  if (step < 1) step = 1;
  if (tgt > cur) return cur + min(step, tgt - cur);
  if (tgt < cur) return cur - min(step, cur - tgt);
  return cur;
}

// ================= Public =================
void Motion::begin() {
  _drv.begin();
  _odo.begin();
  _imu.begin();
  _ir.begin();

  _hasFused = false;
  _fusedYaw10 = 0;

  _done = true;
  _idx = 0;
  _nSteps = 0;
  _steps = nullptr;

  _lastCtlMs = millis();
}

void Motion::loadScript(const Step* steps, size_t n) {
  _steps = steps;
  _nSteps = n;
  _idx = 0;
  _done = (n == 0);
  _stepInit = false;
  _settleUntilMs = 0;
}

void Motion::updateFusion() {
  _imu.update();
  _odo.update();

  if (_imu.hasYaw() && !_odo.hasYaw()) {
    _odo.alignYawTo(_imu.yaw10());
  }

  if (_imu.hasYaw() && _odo.hasYaw()) {
    const int16_t imuY = _imu.yaw10();
    const int16_t encY = _odo.encYaw10();
    _fusedYaw10 = fuseYaw10_Circular(imuY, FUSION_ALPHA_IMU, encY, (1.0f - FUSION_ALPHA_IMU));
    _hasFused = true;
  } else if (_imu.hasYaw()) {
    _fusedYaw10 = _imu.yaw10();
    _hasFused = true;
  } else if (_odo.hasYaw()) {
    _fusedYaw10 = _odo.encYaw10();
    _hasFused = true;
  } else {
    _hasFused = false;
  }
}

void Motion::stopAndSettle(uint32_t nowMs) {
  _drv.stop();
  _settleUntilMs = nowMs + POST_STEP_SETTLE_MS;
}

void Motion::startStep() {
  _stepInit = true;
  _stepStartMs = millis();

  if (_idx >= _nSteps) {
    _done = true;
    _drv.stop();
    return;
  }

  Step s = _steps[_idx];

  // reset shared states
  _turnStable = 0;
  _turnPid.reset();

  // By default, a step consumes any carry-over flag (used to blend Drift->Move).
  const bool carry = _carryMoveState;
  _carryMoveState = false;

  if (s.type == StepType::MoveCells) {
    _moveStartDist = _odo.distMm();
    _moveTargetDist = _moveStartDist + (float)s.a * CELL_MM;
    _moveDir = +1;

    // Use forced heading when available (e.g., after BackAlign or Drift), otherwise hold current.
    _moveHeadingTarget10 = _forcedHeadingValid ? _forcedHeading10 : _fusedYaw10;
    // Do not auto-clear forced heading; let it persist until another command updates it.

    // Optionally carry the speed controller state (from Drift) to avoid a hard speed drop.
    _distPrev = _odo.distMm();
    _vRaw = 0.0f;
    _aCmd = 0.0f;
    if (!carry) {
      _vFilt = 0.0f;
      _vCmd = 0.0f;
      _speedI = 0.0f;
      _pwmCmd = 0;
      _kickUntilMs = millis() + KICK_MS;
    } else {
      _kickUntilMs = 0; // don't re-kick when blending
    }
    _yawErrPrev10 = 0;
  }
  else if (s.type == StepType::RunDistance) {
    _moveStartDist = _odo.distMm();
    _moveTargetDist = _moveStartDist + (float)s.a; // mm, signed
    _moveDir = (s.a >= 0) ? +1 : -1;

    _moveHeadingTarget10 = _forcedHeadingValid ? _forcedHeading10 : _fusedYaw10;

    _distPrev = _odo.distMm();
    _vRaw = 0.0f;
    _aCmd = 0.0f;
    if (!carry) {
      _vFilt = 0.0f;
      _vCmd = 0.0f;
      _speedI = 0.0f;
      _pwmCmd = 0;
      _kickUntilMs = millis() + KICK_MS;
    } else {
      _kickUntilMs = 0;
    }
    _yawErrPrev10 = 0;
  }
  else if (s.type == StepType::DriftDeg) {
    _driftStartDist = _odo.distMm();
    _driftStartYaw10 = _fusedYaw10;
    int dir = (s.b >= 0) ? +1 : -1;
    int32_t delta10 = (int32_t)dir * (int32_t)s.a * 10;
    _driftTargetYaw10 = wrapYaw10((int32_t)_driftStartYaw10 + delta10);
    // Arc length derived from a fixed calibration constant (tune DRIFT_ARC_MM_PER_90).
    float arc = (fabsf((float)s.a) / 90.0f) * DRIFT_ARC_MM_PER_90;
    _driftTargetDist = _driftStartDist + arc;

    // Reuse move controller state (fresh start)
    _distPrev = _odo.distMm();
    _vRaw = 0.0f;
    _vFilt = 0.0f;
    _aCmd = 0.0f;
    _vCmd = 0.0f;
    _speedI = 0.0f;
    _pwmCmd = 0;
    _yawErrPrev10 = 0;
    _kickUntilMs = millis() + KICK_MS;
  }
  else if (s.type == StepType::BackAlign) {
    _backStartDist = _odo.distMm();
    const float maxBack = (s.a > 0) ? (float)s.a : BACK_ALIGN_MAX_MM;
    _backTargetDist = _backStartDist - maxBack;
    _backStallSinceMs = 0;

    // Prepare speed estimate
    _distPrev = _odo.distMm();
    _vRaw = 0.0f;
    _vFilt = 0.0f;
    _aCmd = 0.0f;
    _yawErrPrev10 = 0;
  }
  else if (s.type == StepType::DelayMs) {
    _delayMs = (uint32_t)s.a;
    _drv.stop();
  }
  else if (s.type == StepType::TurnDeg) {
    _turnStartYaw10 = _fusedYaw10;
    int dir = (s.b >= 0) ? +1 : -1;
    int32_t delta10 = (int32_t)dir * (int32_t)s.a * 10;
    _turnTargetYaw10 = wrapYaw10((int32_t)_turnStartYaw10 + delta10);
  }
}

void Motion::tickStep(float dt) {
  Step s = _steps[_idx];
  if (s.type == StepType::MoveCells) tickMove(dt);
  else if (s.type == StepType::RunDistance) tickRunDistance(dt);
  else if (s.type == StepType::DriftDeg) tickDrift(dt);
  else if (s.type == StepType::BackAlign) tickBackAlign(dt);
  else if (s.type == StepType::DelayMs) tickDelay(dt);
  else if (s.type == StepType::TurnDeg) tickTurn(dt);
}

// ================= RUN DISTANCE (same controller as MoveCells, supports signed distance) =================
void Motion::tickRunDistance(float dt) {
  const uint32_t nowMs = millis();

  const float distNow = _odo.distMm();
  const float remain = (float)_moveDir * (_moveTargetDist - distNow); // always >=0 when progressing

  // speed estimate (projected to forward-positive)
  const float dd = (float)_moveDir * (distNow - _distPrev);
  _distPrev = distNow;
  _vRaw = dd / dt;

  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  if (remain <= POS_TOL_MM && fabsf(_vFilt) <= V_STOP_MM_S) {
    _speedI = 0.0f;
    _vCmd = 0.0f;
    _pwmCmd = 0;

    stopAndSettle(nowMs);
    _idx++;
    _stepInit = false;
    return;
  }
  float vTarget = motionVTarget(remain, fabsf(_vFilt), MOVE_V_MAX);

  float dv = vTarget - _vCmd;
  float dvMax;
  if (dv >= 0.0f) {
    dvMax = MOVE_A_ACC * dt;
    if (dv > dvMax) dv = dvMax;
  } else {
    dvMax = MOVE_A_DEC * dt;
    if (dv < -dvMax) dv = -dvMax;
  }
  _vCmd += dv;
  if (_vCmd < 0.0f) _vCmd = 0.0f;
  _aCmd = dv / dt;

  int pwmTarget = 0;
  if (_vCmd <= 0.0f) {
    _speedI = 0.0f;
    pwmTarget = 0;
  } else {
    int pwmMin = PWM_RUN_MIN;
    const bool kicking = (nowMs < _kickUntilMs) || (fabsf(_vFilt) < V_KICK_THRESH);
    if (kicking) pwmMin = PWM_KICK_MIN;

    float slope = (float)(FWD_CRUISE_PWM - pwmMin) / MOVE_V_MAX;
    if (slope < 0.0f) slope = 0.0f;
    float pwmFF = (float)pwmMin + slope * _vCmd;

    const float e = _vCmd - _vFilt;
    const float uP = SPEED_KP * e;

    if (kicking) {
      _speedI = 0.0f;
    } else {
      _speedI += e * dt;
    }
    if (_speedI >  SPEED_I_LIM) _speedI =  SPEED_I_LIM;
    if (_speedI < -SPEED_I_LIM) _speedI = -SPEED_I_LIM;
    const float uI = SPEED_KI * _speedI;

    float u = uP + uI;
    if (u >  (float)SPEED_U_LIM) u =  (float)SPEED_U_LIM;
    if (u < -(float)SPEED_U_LIM) u = -(float)SPEED_U_LIM;

    const float pwmRaw = pwmFF + u;
    float pwmClamped = pwmRaw;
    if (pwmClamped < 0.0f) pwmClamped = 0.0f;

    float top = (float)PWM_MAX;
    if (MOVE_PWM_MAX < PWM_MAX) top = (float)MOVE_PWM_MAX;
    if (pwmClamped > top) pwmClamped = top;

    if (pwmClamped != pwmRaw) {
      bool satHigh = (pwmRaw > top);
      bool satLow  = (pwmRaw < 0.0f);
      if ((satHigh && e > 0.0f) || (satLow && e < 0.0f)) {
        _speedI -= e * dt;
      }
    }
    pwmTarget = (int)lroundf(pwmClamped);
  }

  pwmTarget = constrain(pwmTarget, 0, PWM_MAX);
  int step = (int)lroundf((float)PWM_SLEW_PER_SEC * dt);
  if (step < 1) step = 1;
  _pwmCmd = rampInt(_pwmCmd, pwmTarget, step);

  // heading hold (same as forward). Note: when driving backward, motor commands are negative but corr math stays valid.
  int16_t tgt = _moveHeadingTarget10;
  if (_moveDir > 0) {
    tgt = wrapYaw10((int32_t)tgt + (int32_t)_ir.wallYawBias10((uint16_t)_pwmCmd));
  }
  const int16_t err10 = yawDiff10(tgt, _fusedYaw10);
  const int16_t derr10 = (int16_t)((int32_t)err10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = err10;
  float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f;
  int corr = (int)lroundf(HEADING_KP * (float)err10 + HEADING_KD * derr10_s);
  corr = constrain(corr, -HEADING_CORR_LIM, +HEADING_CORR_LIM);

  // Signed motor commands: forward uses +, backward uses -
  int l = (int)_moveDir * _pwmCmd + corr;
  int r = (int)_moveDir * _pwmCmd - corr;

  // If your HwDrive expects signed PWM, this will enable reverse. If it expects 0..PWM_MAX only,
  // we will add a dedicated driveSigned() in HwDrive later.
  _drv.setLeft(l);
  _drv.setRight(r);
}

// ================= DRIFT (forward while yawing to a target) =================
void Motion::tickDrift(float dt) {
  const uint32_t nowMs = millis();

  const float distNow = _odo.distMm();
  const float remain = _driftTargetDist - distNow;

  // speed estimate
  const float dd = distNow - _distPrev;
  _distPrev = distNow;
  _vRaw = dd / dt;
  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  const int16_t yawErr10 = yawDiff10(_driftTargetYaw10, _fusedYaw10);

  // Finish drift when both distance and yaw are close enough (no full stop; we blend into next Move)
  if (remain <= DRIFT_POS_TOL_MM && (int16_t)abs(yawErr10) <= DRIFT_YAW_TOL_YAW10) {
    // Carry speed controller into next Move/RunDistance
    _carryMoveState = true;
    _forcedHeadingValid = true;
    _forcedHeading10 = _driftTargetYaw10;

    _idx++;
    _stepInit = false;
    return;
  }

  // Use a conservative speed cap during drift to keep the curve stable.
  const float vMax = min(MOVE_V_MAX, DRIFT_V_MAX);
  float vTarget = motionVTarget(remain, fabsf(_vFilt), vMax);

  float dv = vTarget - _vCmd;
  float dvMax;
  if (dv >= 0.0f) {
    dvMax = MOVE_A_ACC * dt;
    if (dv > dvMax) dv = dvMax;
  } else {
    dvMax = MOVE_A_DEC * dt;
    if (dv < -dvMax) dv = -dvMax;
  }
  _vCmd += dv;
  if (_vCmd < 0.0f) _vCmd = 0.0f;

  int pwmTarget = 0;
  if (_vCmd <= 0.0f) {
    _speedI = 0.0f;
    pwmTarget = 0;
  } else {
    int pwmMin = PWM_RUN_MIN;
    const bool kicking = (nowMs < _kickUntilMs) || (fabsf(_vFilt) < V_KICK_THRESH);
    if (kicking) pwmMin = PWM_KICK_MIN;

    float slope = (float)(FWD_CRUISE_PWM - pwmMin) / vMax;
    if (slope < 0.0f) slope = 0.0f;
    float pwmFF = (float)pwmMin + slope * _vCmd;

    const float e = _vCmd - _vFilt;
    const float uP = SPEED_KP * e;
    if (kicking) {
      _speedI = 0.0f;
    } else {
      _speedI += e * dt;
    }
    if (_speedI >  SPEED_I_LIM) _speedI =  SPEED_I_LIM;
    if (_speedI < -SPEED_I_LIM) _speedI = -SPEED_I_LIM;
    const float uI = SPEED_KI * _speedI;

    float u = uP + uI;
    if (u >  (float)SPEED_U_LIM) u =  (float)SPEED_U_LIM;
    if (u < -(float)SPEED_U_LIM) u = -(float)SPEED_U_LIM;

    float pwmClamped = pwmFF + u;
    if (pwmClamped < 0.0f) pwmClamped = 0.0f;
    float top = (float)PWM_MAX;
    if (MOVE_PWM_MAX < PWM_MAX) top = (float)MOVE_PWM_MAX;
    if (pwmClamped > top) pwmClamped = top;
    pwmTarget = (int)lroundf(pwmClamped);
  }

  pwmTarget = constrain(pwmTarget, 0, PWM_MAX);
  int step = (int)lroundf((float)PWM_SLEW_PER_SEC * dt);
  if (step < 1) step = 1;
  _pwmCmd = rampInt(_pwmCmd, pwmTarget, step);

  // Drift heading controller: PD on yaw error, but clamp smaller than straight to avoid scrub.
  const int16_t derr10 = (int16_t)((int32_t)yawErr10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = yawErr10;
  float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f;
  int corr = (int)lroundf(DRIFT_HEADING_KP * (float)yawErr10 + DRIFT_HEADING_KD * derr10_s);
  corr = constrain(corr, -DRIFT_CORR_LIM, +DRIFT_CORR_LIM);

  int l = constrain(_pwmCmd + corr, 0, PWM_MAX);
  int r = constrain(_pwmCmd - corr, 0, PWM_MAX);
  _drv.setLeft(l);
  _drv.setRight(r);
}

// ================= BACK ALIGN (reverse until tail hits wall, then snap heading) =================
void Motion::tickBackAlign(float dt) {
  const uint32_t nowMs = millis();

  const float distNow = _odo.distMm();
  const float remain = distNow - _backTargetDist; // how much distance left to go backward (>=0)

  // speed estimate (backward-positive)
  const float dd = (_distPrev - distNow); // reverse movement gives positive dd
  _distPrev = distNow;
  _vRaw = dd / dt;
  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  // Stall detection: if speed is low continuously, we assume tail is pressed against wall.
  if (fabsf(_vFilt) < BACK_STALL_V_MM_S) {
    if (_backStallSinceMs == 0) _backStallSinceMs = nowMs;
  } else {
    _backStallSinceMs = 0;
  }

  const bool stalled = (_backStallSinceMs != 0) && (nowMs - _backStallSinceMs >= BACK_STALL_MS);
  const bool reached = (remain <= BACK_MIN_EXTRA_MM); // safety: reached max back distance

  if (stalled || reached) {
    _drv.stop();

    // Snap the forced heading to the nearest 90deg based on the current fused yaw.
    // This does NOT reset IMU; it only sets the heading reference for subsequent steps.
    int32_t y = _fusedYaw10;
    // round to nearest 900 (90deg)
    int32_t snapped = ((y + 450) / 900) * 900;
    _forcedHeading10 = wrapYaw10(snapped);
    _forcedHeadingValid = true;

    stopAndSettle(nowMs);
    _idx++;
    _stepInit = false;
    return;
  }

  // Drive backward at a fixed low PWM with small heading correction toward current forced heading (if any)
  const int16_t tgt = _forcedHeadingValid ? _forcedHeading10 : _fusedYaw10;
  const int16_t err10 = yawDiff10(tgt, _fusedYaw10);
  const int16_t derr10 = (int16_t)((int32_t)err10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = err10;
  float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f;
  int corr = (int)lroundf(BACK_HEADING_KP * (float)err10 + BACK_HEADING_KD * derr10_s);
  corr = constrain(corr, -BACK_CORR_LIM, +BACK_CORR_LIM);

  int base = BACK_ALIGN_PWM;
  int l = -base + corr;
  int r = -base - corr;
  _drv.setLeft(l);
  _drv.setRight(r);
}

void Motion::update() {
  uint32_t now = millis();
  if (now - _lastCtlMs < CONTROL_PERIOD_MS) return;
  float dt = (float)(now - _lastCtlMs) * 0.001f;
  // Clamp dt to keep control behavior consistent even if the loop hiccups
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.020f) dt = 0.020f;
  _lastCtlMs = now;

  updateFusion();
  _ir.update(dt);

  if (!_hasFused) {
    _drv.stop();
    return;
  }

  if (!_enabled) {
    _drv.stop();
    return;
  }

  if (now < _settleUntilMs) {
    _drv.stop();
    return;
  }

  if (_done) {
    _drv.stop();
    return;
  }

  if (!_stepInit) startStep();
  if (_done) return;

  tickStep(dt);
}

// ================= MOVE (Profile + FF+PI speed + PD heading) =================
void Motion::tickMove(float dt) {
  const uint32_t nowMs = millis();

  const float distNow = _odo.distMm();
  const float remain = _moveTargetDist - distNow;

  // speed estimate (encoder distance delta)
  const float dd = distNow - _distPrev;
  _distPrev = distNow;
  _vRaw = dd / dt;

  // low-pass filter on speed (reduces braking jitter at high speed)
  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  // stop condition: must be close AND nearly stopped
  if (remain <= POS_TOL_MM && fabsf(_vFilt) <= V_STOP_MM_S) {
    _speedI = 0.0f;
    _vCmd = 0.0f;
    _pwmCmd = 0;

    stopAndSettle(nowMs);
    _idx++;
    _stepInit = false;
    return;
  }

  // compute braking-limited velocity target (works for 1 cell and N cells)
  float vTarget = motionVTarget(remain, fabsf(_vFilt), MOVE_V_MAX);

  // accel/decel limit (separate accel vs brake)
  float dv = vTarget - _vCmd;
  float dvMax;
  if (dv >= 0.0f) {
    dvMax = MOVE_A_ACC * dt;
    if (dv > dvMax) dv = dvMax;
  } else {
    dvMax = MOVE_A_DEC * dt;
    if (dv < -dvMax) dv = -dvMax;
  }
  _vCmd += dv;
  if (_vCmd < 0.0f) _vCmd = 0.0f;

  // actual accel command (for debugging / future kA feedforward)
  _aCmd = dv / dt;

  // inner speed PI => PWM
  int pwmTarget = 0;

  if (_vCmd <= 0.0f) {
    _speedI = 0.0f;
    pwmTarget = 0;
  } else {
    int pwmMin = PWM_RUN_MIN;
    // 'kicking' is true until the robot is clearly moving; this avoids stiction-induced variability
    const bool kicking = (nowMs < _kickUntilMs) || (fabsf(_vFilt) < V_KICK_THRESH);
    if (kicking) pwmMin = PWM_KICK_MIN;

    // linear feedforward: pwmFF = pwmMin + kV * vCmd
    float slope = (float)(FWD_CRUISE_PWM - pwmMin) / MOVE_V_MAX;
    if (slope < 0.0f) slope = 0.0f;
    float pwmFF = (float)pwmMin + slope * _vCmd;

    // PI on speed error
    const float e = _vCmd - _vFilt;
    const float uP = SPEED_KP * e;

    // Gate integral during kick/stiction to prevent wind-up (common cause of random 'kick then overshoot')
    if (kicking) {
      _speedI = 0.0f;
    } else {
      _speedI += e * dt; // mm
    }
    if (_speedI >  SPEED_I_LIM) _speedI =  SPEED_I_LIM;
    if (_speedI < -SPEED_I_LIM) _speedI = -SPEED_I_LIM;
    const float uI = SPEED_KI * _speedI;

    float u = uP + uI;
    if (u >  (float)SPEED_U_LIM) u =  (float)SPEED_U_LIM;
    if (u < -(float)SPEED_U_LIM) u = -(float)SPEED_U_LIM;

    const float pwmRaw = pwmFF + u;

    float pwmClamped = pwmRaw;
    if (pwmClamped < 0.0f) pwmClamped = 0.0f;

    float top = (float)PWM_MAX;
    if (MOVE_PWM_MAX < PWM_MAX) top = (float)MOVE_PWM_MAX;
    if (pwmClamped > top) pwmClamped = top;

    // anti-windup: if saturated and integral pushes further into saturation, undo
    if (pwmClamped != pwmRaw) {
      bool satHigh = (pwmRaw > top);
      bool satLow  = (pwmRaw < 0.0f);
      if ((satHigh && e > 0.0f) || (satLow && e < 0.0f)) {
        _speedI -= e * dt;
      }
    }

    pwmTarget = (int)lroundf(pwmClamped);
  }

  pwmTarget = constrain(pwmTarget, 0, PWM_MAX);

  // slew limiter (prevents step PWM that causes slip)
  int step = (int)lroundf((float)PWM_SLEW_PER_SEC * dt);
  if (step < 1) step = 1;
  _pwmCmd = rampInt(_pwmCmd, pwmTarget, step);

  // heading hold: PD on yaw error (reduces scrub at high speed)
  int16_t tgt = _moveHeadingTarget10;
  // Add a small yaw bias based on side IR to keep the robot centered in a corridor.
  tgt = wrapYaw10((int32_t)tgt + (int32_t)_ir.wallYawBias10((uint16_t)_pwmCmd));
  const int16_t err10 = yawDiff10(tgt, _fusedYaw10);
  const int16_t derr10 = (int16_t)((int32_t)err10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = err10;

  float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f; // (0.1deg)/s
  int corr = (int)lroundf(HEADING_KP * (float)err10 + HEADING_KD * derr10_s);

  // Optional: reduce heading authority when PWM is high to limit slip.
  int corrLim = HEADING_CORR_LIM;
  corr = constrain(corr, -corrLim, +corrLim);

  int l = constrain(_pwmCmd + corr, 0, PWM_MAX);
  int r = constrain(_pwmCmd - corr, 0, PWM_MAX);

  _drv.setLeft(l);
  _drv.setRight(r);
}

// ================= DELAY =================
void Motion::tickDelay(float /*dt*/) {
  uint32_t now = millis();
  if (now - _stepStartMs >= _delayMs) {
    stopAndSettle(now);
    _idx++;
    _stepInit = false;
  } else {
    _drv.stop();
  }
}

// ================= TURN =================
void Motion::tickTurn(float dt) {
  int16_t err10 = yawDiff10(_turnTargetYaw10, _fusedYaw10);

  float out = _turnPid.update((float)err10, dt);
  int cmd = (int)lroundf(out);

  int16_t aerr = (int16_t)abs(err10);
  float scale = 1.0f;
  if (aerr < TURN_SLOW_ZONE_YAW10) {
    scale = (float)aerr / (float)TURN_SLOW_ZONE_YAW10;
    if (scale < 0.30f) scale = 0.30f;
  }
  int dynMax = (int)lroundf(TURN_MIN_PWM + (TURN_MAX_PWM - TURN_MIN_PWM) * scale);
  cmd = constrain(cmd, -dynMax, dynMax);

  if (abs(cmd) > 0 && abs(cmd) < TURN_MIN_PWM) cmd = (cmd > 0) ? TURN_MIN_PWM : -TURN_MIN_PWM;

  if (abs(err10) <= TURN_TOL_YAW10) {
    _turnStable++;
    _drv.rotateInPlace(0);
  } else {
    _turnStable = 0;
    _drv.rotateInPlace(cmd);
  }

  if (_turnStable >= TURN_STABLE_CYCLES) {
    stopAndSettle(millis());
    _idx++;
    _stepInit = false;
  }
}