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

  _directMode = DirectMode::None;
  _autoRunDoneFlag = false;
  _autoRearAlignDoneFlag = false;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;
  _autoStraightHeading10 = 0;
  _autoStraightWallCorr = true;
  _autoStraightStartDist = 0.0f;
  _autoStraightSpeedI = 0.0f;
  _autoBrakeUntilMs = 0;
  _autoBrakeHold = false;
  _autoRearAlignStartDist = 0.0f;
  _autoRearAlignTargetDist = 0.0f;
  _autoRearAlignHeading10 = 0;
  _autoRearAlignStallSinceMs = 0;
  _autoRearAlignHitWall = false;
  _autoRearAlignBackedMm = 0.0f;
  _autoSpeedTurnDir = +1;
  _autoSpeedTurnStartDist = 0.0f;
  _autoSpeedTurnRadiusMm = 0.0f;
  _autoSpeedTurnArcLenMm = 0.0f;
  _autoSpeedTurnTargetSpeedMmS = 0.0f;
  _autoSpeedTurnSpeedI = 0.0f;
  _autoSpeedTurnStartYaw10 = 0;
  _autoSpeedTurnFinalYaw10 = 0;

  _lastCtlUs = micros();
}

void Motion::autoClear() {
  _directMode = DirectMode::None;
  _autoRunDoneFlag = false;
  _autoRearAlignDoneFlag = false;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;
  _autoBrakeUntilMs = 0;
  _autoBrakeHold = false;
  _autoRearAlignStartDist = 0.0f;
  _autoRearAlignTargetDist = 0.0f;
  _autoRearAlignHeading10 = 0;
  _autoRearAlignStallSinceMs = 0;
  _autoRearAlignHitWall = false;
  _autoRearAlignBackedMm = 0.0f;
  _autoSpeedTurnDir = +1;
  _autoSpeedTurnStartDist = 0.0f;
  _autoSpeedTurnRadiusMm = 0.0f;
  _autoSpeedTurnArcLenMm = 0.0f;
  _autoSpeedTurnTargetSpeedMmS = 0.0f;
  _autoSpeedTurnSpeedI = 0.0f;
  _autoSpeedTurnStartYaw10 = 0;
  _autoSpeedTurnFinalYaw10 = 0;
  _turnStable = 0;
  _turnPid.reset();
  _yawErrPrev10 = 0;
  _autoStraightStartDist = _odo.distMm();
  _autoStraightSpeedI = 0.0f;
  _ir.resetWallController();
  _drv.stop();
}

void Motion::autoStartStraight(int16_t headingTarget10, bool useWallCorrection) {
  _directMode = DirectMode::Straight;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;
  _autoStraightHeading10 = headingTarget10;
  _autoStraightWallCorr = useWallCorrection;
  _autoStraightStartDist = _odo.distMm();
  _autoStraightSpeedI = 0.0f;
  _distPrev = _odo.distMm();
  _vRaw = 0.0f;
  _vFilt = 0.0f;
  _yawErrPrev10 = 0;
}

void Motion::autoSetStraightHeading(int16_t headingTarget10) {
  _autoStraightHeading10 = headingTarget10;
}

void Motion::autoSetStraightWallCorrection(bool en) {
  _autoStraightWallCorr = en;
  if (!en) _ir.resetWallController();
}

void Motion::autoStartRunDistance(float mm, int16_t headingTarget10) {
  _directMode = DirectMode::RunDistance;
  _autoRunDoneFlag = false;
  _autoRearAlignDoneFlag = false;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;

  _moveTargetDist = _odo.distMm() + mm;
  _moveDir = (mm >= 0.0f) ? +1 : -1;
  _moveHeadingTarget10 = headingTarget10;

  _distPrev = _odo.distMm();
  _vRaw = 0.0f;
  _vFilt = 0.0f;
  _vCmd = 0.0f;
  _speedI = 0.0f;
  _pwmCmd = 0;
  _kickUntilMs = millis() + KICK_MS;
  _yawErrPrev10 = 0;
  _ir.resetWallController();
}

void Motion::autoStartRearAlign(float maxBackMm, int16_t headingTarget10) {
  _directMode = DirectMode::RearAlign;
  _autoRunDoneFlag = false;
  _autoRearAlignDoneFlag = false;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;

  _autoRearAlignStartDist = _odo.distMm();
  _autoRearAlignTargetDist = _autoRearAlignStartDist - maxBackMm;
  _autoRearAlignHeading10 = headingTarget10;
  _autoRearAlignStallSinceMs = 0;
  _autoRearAlignHitWall = false;
  _autoRearAlignBackedMm = 0.0f;

  _distPrev = _odo.distMm();
  _vRaw = 0.0f;
  _vFilt = 0.0f;
  _yawErrPrev10 = 0;
  _ir.resetWallController();
}

void Motion::autoStartTurn(int deg, int dir) {
  _directMode = DirectMode::Turn;
  _autoRunDoneFlag = false;
  _autoRearAlignDoneFlag = false;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;
  _turnStable = 0;
  _turnPid.reset();
  _ir.resetWallController();

  _turnStartYaw10 = _fusedYaw10;
  const int sign = (dir >= 0) ? +1 : -1;
  const int32_t delta10 = (int32_t)sign * (int32_t)deg * 10;
  _turnTargetYaw10 = wrapYaw10((int32_t)_turnStartYaw10 + delta10);
}

void Motion::autoStartSpeedTurn(int dir, float radiusMm, float centerSpeedMmS) {
  _directMode = DirectMode::SpeedTurn;
  _autoRunDoneFlag = false;
  _autoRearAlignDoneFlag = false;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;
  _ir.resetWallController();

  _autoSpeedTurnDir = (dir >= 0) ? +1 : -1;
  _autoSpeedTurnRadiusMm = fmaxf(radiusMm, 0.5f * WHEELBASE_MM + 1.0f);
  _autoSpeedTurnArcLenMm = 0.5f * PI_F * _autoSpeedTurnRadiusMm;
  _autoSpeedTurnTargetSpeedMmS = fmaxf(centerSpeedMmS, V_CREEP_MM_S);
  _autoSpeedTurnSpeedI = 0.0f;
  _autoSpeedTurnStartDist = _odo.distMm();
  _autoSpeedTurnStartYaw10 = _fusedYaw10;
  _autoSpeedTurnFinalYaw10 =
      wrapYaw10((int32_t)_autoSpeedTurnStartYaw10 + (int32_t)_autoSpeedTurnDir * (int32_t)SPEEDRUN_PRETURN_YAW_DEG * 10);

  _distPrev = _autoSpeedTurnStartDist;
  _vRaw = 0.0f;
  _vFilt = 0.0f;
  _yawErrPrev10 = 0;
  _autoStraightWallCorr = false;
  if (_pwmCmd <= 0) {
    _pwmCmd = constrain(SPEEDRUN_PRETURN_CENTER_PWM, 0, PWM_MAX);
  }
}

void Motion::autoBrake(uint32_t holdMs, bool holdAfter) {
  _directMode = DirectMode::Brake;
  _autoRunDoneFlag = false;
  _autoRearAlignDoneFlag = false;
  _autoTurnDoneFlag = false;
  _autoBrakeDoneFlag = false;
  _autoBrakeUntilMs = millis() + holdMs;
  _autoBrakeHold = holdAfter;
  _ir.resetWallController();
}

bool Motion::consumeAutoRunDone() {
  const bool v = _autoRunDoneFlag;
  _autoRunDoneFlag = false;
  return v;
}

bool Motion::consumeAutoRearAlignDone() {
  const bool v = _autoRearAlignDoneFlag;
  _autoRearAlignDoneFlag = false;
  return v;
}

bool Motion::consumeAutoTurnDone() {
  const bool v = _autoTurnDoneFlag;
  _autoTurnDoneFlag = false;
  return v;
}

bool Motion::consumeAutoBrakeDone() {
  const bool v = _autoBrakeDoneFlag;
  _autoBrakeDoneFlag = false;
  return v;
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

void Motion::update() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - _lastCtlUs) < CONTROL_PERIOD_US) return;

  float dt = (float)(nowUs - _lastCtlUs) * 1e-6f;
  // Clamp dt to keep control behavior consistent even if the loop hiccups
  if (dt < 0.0005f) dt = 0.0005f;
  if (dt > 0.020f) dt = 0.020f;
  _lastCtlUs = nowUs;

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

  if (_directMode == DirectMode::Straight) {
    tickAutoStraight(dt);
    return;
  }
  if (_directMode == DirectMode::RunDistance) {
    tickAutoRunDistance(dt);
    return;
  }
  if (_directMode == DirectMode::RearAlign) {
    tickAutoRearAlign(dt);
    return;
  }
  if (_directMode == DirectMode::Turn) {
    tickAutoTurn(dt);
    return;
  }
  if (_directMode == DirectMode::SpeedTurn) {
    tickAutoSpeedTurn(dt);
    return;
  }
  if (_directMode == DirectMode::Brake) {
    tickAutoBrake(dt);
    return;
  }

  _drv.stop();
}

void Motion::tickAutoStraight(float dt) {
  const int baseOpenLoop = constrain(AUTO_STRAIGHT_PWM, 0, PWM_MAX);

  const float distNow = _odo.distMm();
  const float dd = distNow - _distPrev;
  _distPrev = distNow;
  _vRaw = (dt > 0.0f) ? (dd / dt) : 0.0f;

  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  const float straightMm = distNow - _autoStraightStartDist;
  int base = baseOpenLoop;
  if (AUTO_SPEED_PI_ENABLE && straightMm >= AUTO_SPEED_PI_ARM_MM) {
    const float e = AUTO_SPEED_TARGET_MM_S - _vFilt;
    _autoStraightSpeedI += e * dt;
    if (_autoStraightSpeedI >  AUTO_SPEED_I_LIM) _autoStraightSpeedI =  AUTO_SPEED_I_LIM;
    if (_autoStraightSpeedI < -AUTO_SPEED_I_LIM) _autoStraightSpeedI = -AUTO_SPEED_I_LIM;

    float trim = AUTO_SPEED_KP * e + AUTO_SPEED_KI * _autoStraightSpeedI;
    if (trim >  (float)AUTO_SPEED_TRIM_LIM) trim =  (float)AUTO_SPEED_TRIM_LIM;
    if (trim < -(float)AUTO_SPEED_TRIM_LIM) trim = -(float)AUTO_SPEED_TRIM_LIM;
    base = constrain(baseOpenLoop + (int)lroundf(trim), 0, PWM_MAX);
  } else {
    _autoStraightSpeedI = 0.0f;
  }

  _pwmCmd = base;

  int16_t tgt = _autoStraightHeading10;
  if (_autoStraightWallCorr) {
    tgt = wrapYaw10((int32_t)tgt + (int32_t)_ir.wallYawBias10((uint16_t)_pwmCmd, dt));
  } else {
    _ir.resetWallController();
  }

  const int16_t err10 = yawDiff10(tgt, _fusedYaw10);
  const int16_t derr10 = (int16_t)((int32_t)err10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = err10;

  const float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f;
  int corr = (int)lroundf(AUTO_HEADING_KP * (float)err10 + AUTO_HEADING_KD * derr10_s);
  corr = constrain(corr, -AUTO_HEADING_CORR_LIM, +AUTO_HEADING_CORR_LIM);

  const int l = constrain(_pwmCmd + corr, 0, PWM_MAX);
  const int r = constrain(_pwmCmd - corr, 0, PWM_MAX);
  _drv.setLeft(l);
  _drv.setRight(r);
}



void Motion::tickAutoRunDistance(float dt) {
  const uint32_t nowMs = millis();

  const float distNow = _odo.distMm();
  const float remain = (float)_moveDir * (_moveTargetDist - distNow);

  const float dd = (float)_moveDir * (distNow - _distPrev);
  _distPrev = distNow;
  _vRaw = dd / dt;

  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  if (remain <= POS_TOL_MM && fabsf(_vFilt) <= V_STOP_MM_S) {
    _speedI = 0.0f;
    _vCmd = 0.0f;
    _pwmCmd = 0;
    _drv.stop();
    _directMode = DirectMode::None;
    _autoRunDoneFlag = true;
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

  const int16_t err10 = yawDiff10(_moveHeadingTarget10, _fusedYaw10);
  const int16_t derr10 = (int16_t)((int32_t)err10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = err10;

  float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f;
  int corr = (int)lroundf(HEADING_KP * (float)err10 + HEADING_KD * derr10_s);
  corr = constrain(corr, -HEADING_CORR_LIM, +HEADING_CORR_LIM);

  int base = _pwmCmd;
  int l = constrain(_moveDir * base + corr, -PWM_MAX, PWM_MAX);
  int r = constrain(_moveDir * base - corr, -PWM_MAX, PWM_MAX);

  _drv.setLeft(l);
  _drv.setRight(r);
}

void Motion::tickAutoRearAlign(float dt) {
  const uint32_t nowMs = millis();

  const float distNow = _odo.distMm();
  const float backedMm = _autoRearAlignStartDist - distNow;
  const float remain = distNow - _autoRearAlignTargetDist;

  const float dd = -1.0f * (distNow - _distPrev);
  _distPrev = distNow;
  _vRaw = dd / dt;

  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  if (backedMm >= AUTO_REAR_ALIGN_STALL_ARM_MM && fabsf(_vFilt) < AUTO_REAR_ALIGN_STALL_V_MM_S) {
    if (_autoRearAlignStallSinceMs == 0) _autoRearAlignStallSinceMs = nowMs;
  } else {
    _autoRearAlignStallSinceMs = 0;
  }

  const bool stalled = (_autoRearAlignStallSinceMs != 0) &&
                       (nowMs - _autoRearAlignStallSinceMs >= AUTO_REAR_ALIGN_STALL_MS);
  const bool reached = (remain <= 0.5f);

  if (stalled || reached) {
    _drv.stop();
    _directMode = DirectMode::None;
    _autoRearAlignHitWall = stalled;
    _autoRearAlignBackedMm = backedMm > 0.0f ? backedMm : 0.0f;
    _autoRearAlignDoneFlag = true;
    return;
  }

  const int16_t err10 = yawDiff10(_autoRearAlignHeading10, _fusedYaw10);
  const int16_t derr10 = (int16_t)((int32_t)err10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = err10;
  float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f;
  int corr = (int)lroundf(AUTO_REAR_ALIGN_HEADING_KP * (float)err10 +
                          AUTO_REAR_ALIGN_HEADING_KD * derr10_s);
  corr = constrain(corr, -AUTO_REAR_ALIGN_CORR_LIM, +AUTO_REAR_ALIGN_CORR_LIM);

  const int base = constrain(AUTO_REAR_ALIGN_PWM, 0, PWM_MAX);
  const int l = constrain(-base + corr, -PWM_MAX, PWM_MAX);
  const int r = constrain(-base - corr, -PWM_MAX, PWM_MAX);
  _drv.setLeft(l);
  _drv.setRight(r);
}

void Motion::tickAutoTurn(float dt) {
  int16_t err10 = yawDiff10(_turnTargetYaw10, _fusedYaw10);

  float out = _turnPid.update((float)err10, dt);
  int cmd = (int)lroundf(out);

  const int16_t aerr = (int16_t)abs(err10);
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
    _drv.stop();
    _directMode = DirectMode::None;
    _autoTurnDoneFlag = true;
  }
}

void Motion::tickAutoSpeedTurn(float dt) {
  const float distNow = _odo.distMm();
  const float dd = distNow - _distPrev;
  _distPrev = distNow;
  _vRaw = (dt > 0.0f) ? (dd / dt) : 0.0f;

  const float alpha = dt / (SPEED_TAU_S + dt);
  _vFilt += alpha * (_vRaw - _vFilt);

  const float progressMm = distNow - _autoSpeedTurnStartDist;
  float arcProgressMm = progressMm;
  if (arcProgressMm < 0.0f) arcProgressMm = 0.0f;
  if (arcProgressMm > _autoSpeedTurnArcLenMm) arcProgressMm = _autoSpeedTurnArcLenMm;

  const float yawProgressDeg =
      (_autoSpeedTurnRadiusMm > 1e-3f) ? (arcProgressMm / _autoSpeedTurnRadiusMm) * (180.0f / PI_F) : 0.0f;
  const int32_t yawRef10Delta = (int32_t)lroundf((float)_autoSpeedTurnDir * yawProgressDeg * 10.0f);
  const int16_t yawRef10 = wrapYaw10((int32_t)_autoSpeedTurnStartYaw10 + yawRef10Delta);

  const int baseOpenLoop = constrain(SPEEDRUN_PRETURN_CENTER_PWM, 0, PWM_MAX);
  int base = baseOpenLoop;
  if (AUTO_SPEED_PI_ENABLE) {
    const float e = _autoSpeedTurnTargetSpeedMmS - _vFilt;
    _autoSpeedTurnSpeedI += e * dt;
    if (_autoSpeedTurnSpeedI >  AUTO_SPEED_I_LIM) _autoSpeedTurnSpeedI =  AUTO_SPEED_I_LIM;
    if (_autoSpeedTurnSpeedI < -AUTO_SPEED_I_LIM) _autoSpeedTurnSpeedI = -AUTO_SPEED_I_LIM;

    float trim = AUTO_SPEED_KP * e + AUTO_SPEED_KI * _autoSpeedTurnSpeedI;
    if (trim >  (float)AUTO_SPEED_TRIM_LIM) trim =  (float)AUTO_SPEED_TRIM_LIM;
    if (trim < -(float)AUTO_SPEED_TRIM_LIM) trim = -(float)AUTO_SPEED_TRIM_LIM;
    base = constrain(baseOpenLoop + (int)lroundf(trim), 0, PWM_MAX);
  } else {
    _autoSpeedTurnSpeedI = 0.0f;
  }
  _pwmCmd = base;

  const int16_t err10 = yawDiff10(yawRef10, _fusedYaw10);
  const int16_t derr10 = (int16_t)((int32_t)err10 - (int32_t)_yawErrPrev10);
  _yawErrPrev10 = err10;
  const float derr10_s = (dt > 0.0f) ? ((float)derr10 / dt) : 0.0f;
  int corr = (int)lroundf(AUTO_HEADING_KP * (float)err10 + AUTO_HEADING_KD * derr10_s);
  corr = constrain(corr, -SPEEDRUN_PRETURN_CORR_LIM, +SPEEDRUN_PRETURN_CORR_LIM);

  float leftScale = 1.0f;
  float rightScale = 1.0f;
  if (progressMm < _autoSpeedTurnArcLenMm) {
    const float halfWheelbase = 0.5f * WHEELBASE_MM;
    const float innerScale = (_autoSpeedTurnRadiusMm - halfWheelbase) / _autoSpeedTurnRadiusMm;
    const float outerScale = (_autoSpeedTurnRadiusMm + halfWheelbase) / _autoSpeedTurnRadiusMm;
    if (_autoSpeedTurnDir < 0) {
      leftScale = innerScale;
      rightScale = outerScale;
    } else {
      leftScale = outerScale;
      rightScale = innerScale;
    }
  }

  const int leftBase = constrain((int)lroundf((float)base * leftScale), 0, PWM_MAX);
  const int rightBase = constrain((int)lroundf((float)base * rightScale), 0, PWM_MAX);
  const int leftCmd = constrain(leftBase + corr, 0, PWM_MAX);
  const int rightCmd = constrain(rightBase - corr, 0, PWM_MAX);
  _drv.setLeft(leftCmd);
  _drv.setRight(rightCmd);

  const bool distAtGoal = progressMm >= (_autoSpeedTurnArcLenMm - POS_TOL_MM);

  if (distAtGoal) {
    // End the speed-turn at the geometric end of the arc. Waiting for yaw to
    // catch up here can keep the robot in a "post-arc recovery" phase where it
    // drives almost straight before AutoRunner can plan the next corner.
    _directMode = DirectMode::Straight;
    _autoStraightHeading10 = _autoSpeedTurnFinalYaw10;
    _autoStraightWallCorr = false;
    _autoStraightStartDist = distNow;
    _autoStraightSpeedI = 0.0f;
    _distPrev = distNow;
    _yawErrPrev10 = yawDiff10(_autoStraightHeading10, _fusedYaw10);
    _autoTurnDoneFlag = true;
  }
}

void Motion::tickAutoBrake(float /*dt*/) {
  const uint32_t now = millis();
  _drv.brake();

  if (_autoBrakeHold) {
    if (_autoBrakeUntilMs != 0 && (int32_t)(now - _autoBrakeUntilMs) >= 0) {
      // Keep holding brake indefinitely until an explicit mode change.
      _autoBrakeUntilMs = 0;
    }
    return;
  }

  if ((int32_t)(now - _autoBrakeUntilMs) >= 0) {
    _drv.stop();
    _directMode = DirectMode::None;
    _autoBrakeDoneFlag = true;
  }
}
