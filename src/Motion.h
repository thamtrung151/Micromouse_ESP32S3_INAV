

#pragma once
#include <Arduino.h>

#include "imu.h"
#include "Odometry.h"
#include "HwDrive.h"
#include "pid.h"
#include "AngleUtils.h"
#include "config.h" 
#include "IrSensors.h"

// Script
// Keep the original Step struct (2 ints) so your existing scripts still compile.
// Each StepType interprets (a,b) as follows:
//  - MoveCells:    a=cells, b=unused
//  - RunDistance:  a=distance_mm (signed: +forward, -backward), b=unused
//  - DriftDeg:     a=deg, b=dir (-1 left, +1 right). Distance is derived from config (DRIFT_ARC_MM_PER_90).
//  - BackAlign:    a=max_back_mm (safety), b=unused
//  - DelayMs:      a=ms, b=unused
//  - TurnDeg:      a=deg, b=dir (-1 left, +1 right)
enum class StepType : uint8_t { MoveCells, RunDistance, DriftDeg, BackAlign, DelayMs, TurnDeg };

// a,b meaning depends on StepType (see above)
struct Step {
  StepType type;
  int a;
  int b;
};

class Motion {
public:
  void begin();
  void update(); // call continuously in loop()

  // Enable/disable motor outputs and script progression.
  // When disabled, sensors (IMU/encoders/IR) still update.
  void setEnabled(bool en) { _enabled = en; if (!en) _drv.stop(); }
  bool enabled() const { return _enabled; }

  void loadScript(const Step* steps, size_t n);
  bool done() const { return _done; }

  int16_t fusedYaw10() const { return _fusedYaw10; }
  float distMm() const { return _odo.distMm(); }

  // Access to IR readings (filtered)
  const IrSensors& ir() const { return _ir; }

  // Script vs Auto equivalence:
  // Auto mode (AutoRunner) does not implement a separate drive/motion stack.
  // Instead, it builds a short Step[] (turn + MoveCells) and runs it through
  // this same Motion engine via loadScript(). Therefore, when you tune CELL_MM,
  // MOVE_V_MAX, accel/decel, braking, PWM limits, etc. in config.h while testing
  // in SCRIPT mode, AUTO mode will inherit the exact same behavior for the
  // same primitives (MoveCells/TurnDeg/DriftDeg/RunDistance/etc.).

private:
  // lifecycle
  void updateFusion();
  void stopAndSettle(uint32_t nowMs);
  void startStep();
  void tickStep(float dt);

  // step handlers
  void tickMove(float dt);
  void tickRunDistance(float dt);
  void tickDrift(float dt);
  void tickBackAlign(float dt);
  void tickDelay(float dt);
  void tickTurn(float dt);

  // helpers
  static int rampInt(int cur, int tgt, int step);

private:
  // devices
  ImuMsp   _imu;
  Odometry _odo;
  HwDrive  _drv;
  IrSensors _ir;

  bool _enabled = true;

  //fused yaw
  bool    _hasFused = false;
  int16_t _fusedYaw10 = 0;

  // script
  const Step* _steps = nullptr;
  size_t _nSteps = 0;
  size_t _idx = 0;
  bool _done = true;

  // timing
  uint32_t _lastCtlMs = 0;

  // step state
  bool _stepInit = false;
  uint32_t _stepStartMs = 0;
  uint32_t _settleUntilMs = 0;

  //MOVE state
  float   _moveStartDist = 0.0f;
  float   _moveTargetDist = 0.0f;
  int8_t  _moveDir = +1; // +1 forward, -1 backward (used by RunDistance/BackAlign)
  int16_t _moveHeadingTarget10 = 0;

  // If set, the next Move/RunDistance will use this heading target instead of current fused yaw.
  bool    _forcedHeadingValid = false;
  int16_t _forcedHeading10 = 0;

  // When true, we carry the speed controller state into the next Move/RunDistance (used after Drift).
  bool    _carryMoveState = false;

  //DRIFT state
  float   _driftStartDist = 0.0f;
  float   _driftTargetDist = 0.0f;
  int16_t _driftStartYaw10 = 0;
  int16_t _driftTargetYaw10 = 0;

  // BACK-ALIGN state
  float   _backStartDist = 0.0f;
  float   _backTargetDist = 0.0f;
  uint32_t _backStallSinceMs = 0;

  // speed estimation
  float _distPrev = 0.0f;
  float _vRaw = 0.0f;     // mm/s (from encoder distance delta)
  float _vFilt = 0.0f;    // filtered mm/s
  float _aCmd = 0.0f;     // mm/s^2 (profile accel command)

  // speed control
  float _vCmd = 0.0f;     // mm/s (outer profile)
  float _speedI = 0.0f;   // ∫e dt in mm
  int   _pwmCmd = 0;      // slewed PWM command
  uint32_t _kickUntilMs = 0;

  // heading control memory (for D term)
  int16_t _yawErrPrev10 = 0;

  //URN state 
  PID _turnPid = PID(TURN_KP, TURN_KI, TURN_KD);
  int16_t _turnStartYaw10 = 0;
  int16_t _turnTargetYaw10 = 0;
  int _turnStable = 0;

  //DELAY state
  uint32_t _delayMs = 0;


};