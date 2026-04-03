#pragma once

#include <Arduino.h>

#include "Motion.h"
#include "Maze.h"
#include "FloodFill.h"

// Continuous AUTO runner.
// - Straight motion stays continuous with fixed PWM.
// - IR is kept ON continuously.
// - Front IR is only used as a simple brake trigger for a front wall.
// - Position re-anchor is explicit and physical:
//     * at start: reverse toward rear wall, then drive forward continuously to the cell center.
//     * after each completed turn: if map says there is a rear wall in the NEW heading,
//       reverse toward that wall, then drive forward continuously to the cell center.
// - Turns still use IMU-only yaw control.
class AutoRunner {
public:
  void begin(Motion& motion);
  void reset();

  bool startExplore();
  bool startSpeedRun();
  void stop();
  bool running() const { return _running; }
  bool savedMapReady() const { return _savedMapReady; }
  bool clearSavedMap();

  void update();

  int x() const { return _x; }
  int y() const { return _y; }
  uint8_t heading() const { return _heading; }
  bool reachedGoal() const { return _reachedGoal; }

private:
  enum class Phase : uint8_t {
    ExploreToGoal,
    ReturnToStart,
    StandbySpeedRun,
    SpeedRunToGoal,
    Finished
  };

  enum class State : uint8_t {
    Idle,
    StartReverse,
    StartCenterAdvance,
    Straight,
    FrontBrake,
    TurnBrake,
    Turn,
    TurnReverse,
    TurnCenterAdvance,
    GoalHold,
    Halted
  };

  void senseWallsAtCenter();
  void planFromCurrentCell(bool fromMoving);
  void startTurnNow(uint8_t nextDir);
  void handleForwardCenterSnap(float snappedCenterDist);
  void updateStraight();

  void beginStartReanchor();
  void beginPostTurnReanchor();
  void startCenterAdvance(State advanceState);
  void finishStartReanchor();
  void finishPostTurnReanchor();
  bool shouldPostTurnReanchor() const;
  void updateReverseReanchor(State reverseState, State advanceState);
  void updateCenterAdvance(State advanceState);

  bool isGoalCell(int x, int y) const;
  bool isStartCell(int x, int y) const;
  bool inSpeedRun() const { return _phase == Phase::SpeedRunToGoal; }
  bool inReturnPhase() const { return _phase == Phase::ReturnToStart; }
  bool irShouldBeOn(float phaseMm) const;
  bool wallCorrectionShouldBeOn(float phaseMm) const;
  bool frontBrakeHit(float phaseMm) const;
  int16_t headingToMapYaw10(uint8_t heading) const;
  void clearSpeedPreTurn();
  void armSpeedPreTurn(bool knownOpenOnly);
  void updateSpeedPreTurnSyncFromSideWall();
  bool tryStartSpeedPreTurn();
  void resetWallBiasState();
  void updateSlowWallCorrection(float phaseMm);
  void resetRuntimeToStart();
  void resetIndicators();
  bool launchSavedSpeedRun();
  bool canSaveCurrentMap() const;
  bool persistCurrentMapToFlash();
  void enterSpeedRunStandby();
  void startTurnTo(uint8_t nextDir);

private:
  Motion* _m = nullptr;
  Maze _maze;
  FloodFill _ff;

  bool _running = false;
  bool _reachedGoal = false;
  Phase _phase = Phase::ExploreToGoal;
  bool _launchReanchorAfterTurn = false;
  bool _standbyAfterTurn = false;
  bool _savedMapReady = false;

  int _x = 0;
  int _y = 0;
  uint8_t _heading = Maze::N;

  State _state = State::Idle;
  uint8_t _pendingTurnDir = Maze::N;

  float _centerDistMm = 0.0f;
  int16_t _straightHeading10 = 0;
  int16_t _alignHeading10 = 0;
  int16_t _mapNorthHeading10 = 0;

  // Speed-run pre-turn smoothing state.
  bool _speedPreTurnArmed = false;
  bool _speedPreTurnExecuting = false;
  uint8_t _speedPreTurnDir = Maze::N;
  float _speedPreTurnTriggerDistMm = 0.0f;
  float _speedPreTurnCenterDistMm = 0.0f;
  float _speedPreTurnRadiusMm = 0.0f;
  int8_t _speedPreTurnSign = 0;
  bool _speedPreTurnUseSideLossSync = false;
  bool _speedPreTurnTurnSideWallSeen = false;
  bool _speedPreTurnTurnSideLostSynced = false;
  float _speedPreTurnSideLossTriggerDistMm = 0.0f;

  float _frontBrakeSnapDist = 0.0f;
  uint8_t _frontDetectCount = 0;

  // Re-anchor runtime state.
  float _reanchorStartDistMm = 0.0f;
  float _reanchorLastBackedMm = 0.0f;
  uint32_t _reanchorLastProgressMs = 0;
  float _advanceTargetDistMm = 0.0f;

  // Slow wall-bias correction state.
  int16_t _wallBaseHeading10 = 0;
  int16_t _wallBias10 = 0;
  int16_t _wallBiasCmd10 = 0;
  uint32_t _wallLastUpdateMs = 0;
};
