#pragma once
#include <Arduino.h>

#include "imu.h"
#include "Odometry.h"
#include "HwDrive.h"
#include "pid.h"
#include "AngleUtils.h"
#include "config.h"
#include "IrSensors.h"

class Motion {
private:
  enum class DirectMode : uint8_t { None, Straight, RunDistance, RearAlign, Turn, SpeedTurn, Brake };

public:
  void begin();
  void update(); // call continuously in loop()

  // Enable/disable motor outputs and script/direct progression.
  // When disabled, sensors (IMU/encoders/IR) still update.
  void setEnabled(bool en) { _enabled = en; if (!en) { _drv.stop(); _directMode = DirectMode::None; } }
  bool enabled() const { return _enabled; }

  void autoClear();
  void autoStartStraight(int16_t headingTarget10, bool useWallCorrection = true);
  void autoSetStraightHeading(int16_t headingTarget10);
  void autoSetStraightWallCorrection(bool en);
  void autoStartRunDistance(float mm, int16_t headingTarget10);
  void autoStartRearAlign(float maxBackMm, int16_t headingTarget10);
  void autoStartTurn(int deg, int dir); // dir: -1 left, +1 right
  void autoStartSpeedTurn(int dir, float radiusMm, float centerSpeedMmS); // dir: -1 left, +1 right
  void autoBrake(uint32_t holdMs, bool holdAfter = false);
  bool autoInStraight() const { return _directMode == DirectMode::Straight; }
  bool autoRunningDistance() const { return _directMode == DirectMode::RunDistance; }
  bool autoRearAligning() const { return _directMode == DirectMode::RearAlign; }
  bool autoTurning() const { return _directMode == DirectMode::Turn || _directMode == DirectMode::SpeedTurn; }
  bool autoSpeedTurning() const { return _directMode == DirectMode::SpeedTurn; }
  bool autoBraking() const { return _directMode == DirectMode::Brake; }
  bool consumeAutoRunDone();
  bool consumeAutoRearAlignDone();
  bool consumeAutoTurnDone();
  bool consumeAutoBrakeDone();
  bool autoRearAlignHitWall() const { return _autoRearAlignHitWall; }
  float autoRearAlignBackedMm() const { return _autoRearAlignBackedMm; }

  int16_t fusedYaw10() const { return _fusedYaw10; }
  float distMm() const { return _odo.distMm(); }

  // Access to IR readings (filtered)
  const IrSensors& ir() const { return _ir; }
  void setIrEnabled(bool en) { _ir.enableIR(en); }
  bool irEnabled() const { return _ir.irEnabled(); }
  
  float speedRawMmS() const { return _vRaw; }
  float speedFiltMmS() const { return _vFilt; }
  float autoSpeedI() const { return _autoStraightSpeedI; }
  float autoStraightMm() const { return _odo.distMm() - _autoStraightStartDist; }
  int pwmCmd() const { return _pwmCmd; }


  bool autoSpeedPiActive() const {
  return autoInStraight() && (( _odo.distMm() - _autoStraightStartDist ) >= AUTO_SPEED_PI_ARM_MM);
  }

private:
  // lifecycle
  void updateFusion();

  // direct auto handlers
  void tickAutoStraight(float dt);
  void tickAutoRunDistance(float dt);
  void tickAutoRearAlign(float dt);
  void tickAutoTurn(float dt);
  void tickAutoSpeedTurn(float dt);
  void tickAutoBrake(float dt);

  // helpers
  static int rampInt(int cur, int tgt, int step);

private:
  // devices
  ImuMsp   _imu;
  Odometry _odo;
  HwDrive  _drv;
  IrSensors _ir;

  bool _enabled = true;

  // fused yaw
  bool    _hasFused = false;
  int16_t _fusedYaw10 = 0;

  // timing
  uint32_t _lastCtlUs = 0;

  // MOVE state
  float   _moveTargetDist = 0.0f;
  int8_t  _moveDir = +1;
  int16_t _moveHeadingTarget10 = 0;

  // speed estimation
  float _distPrev = 0.0f;
  float _vRaw = 0.0f;     // mm/s (from encoder distance delta)
  float _vFilt = 0.0f;    // filtered mm/s

  // speed control
  float _vCmd = 0.0f;     // mm/s (outer profile)
  float _speedI = 0.0f;   // integral of speed error
  int   _pwmCmd = 0;      // slewed PWM command
  uint32_t _kickUntilMs = 0;

  // heading control memory (for D term)
  int16_t _yawErrPrev10 = 0;

  // TURN state
  PID _turnPid = PID(TURN_KP, TURN_KI, TURN_KD);
  int16_t _turnStartYaw10 = 0;
  int16_t _turnTargetYaw10 = 0;
  int _turnStable = 0;

  // Speed-turn state (continuous ARC cornering in speed-run).
  int8_t  _autoSpeedTurnDir = +1;
  float   _autoSpeedTurnStartDist = 0.0f;
  float   _autoSpeedTurnRadiusMm = 0.0f;
  float   _autoSpeedTurnArcLenMm = 0.0f;
  float   _autoSpeedTurnTargetSpeedMmS = 0.0f;
  float   _autoSpeedTurnSpeedI = 0.0f;
  int16_t _autoSpeedTurnStartYaw10 = 0;
  int16_t _autoSpeedTurnFinalYaw10 = 0;

  // direct AUTO state
  DirectMode _directMode = DirectMode::None;
  int16_t _autoStraightHeading10 = 0;
  bool _autoStraightWallCorr = true;
  float _autoStraightStartDist = 0.0f;
  float _autoStraightSpeedI = 0.0f;
  bool _autoRunDoneFlag = false;
  bool _autoRearAlignDoneFlag = false;
  bool _autoTurnDoneFlag = false;
  bool _autoBrakeDoneFlag = false;
  uint32_t _autoBrakeUntilMs = 0;
  bool _autoBrakeHold = false;
  float _autoRearAlignStartDist = 0.0f;
  float _autoRearAlignTargetDist = 0.0f;
  int16_t _autoRearAlignHeading10 = 0;
  uint32_t _autoRearAlignStallSinceMs = 0;
  bool _autoRearAlignHitWall = false;
  float _autoRearAlignBackedMm = 0.0f;
};
