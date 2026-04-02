#include "AutoRunner.h"
#include "MazeStorage.h"
#include "rgb.h"

void AutoRunner::begin(Motion& motion) {
  _m = &motion;
  const bool loaded = MazeStorage::load(_maze);
  _savedMapReady = loaded && canSaveCurrentMap();
  if (loaded && !_savedMapReady) {
    MazeStorage::clear();
  }
  if (!_savedMapReady) {
    _maze.begin();
  }
  reset();
}

void AutoRunner::reset() {
  resetRuntimeToStart();
  _phase = _savedMapReady ? Phase::StandbySpeedRun : Phase::ExploreToGoal;

  if (_m) {
    _m->autoClear();
    _m->setIrEnabled(true);
    _m->setEnabled(false);
  }

  resetIndicators();
  RGB_setSavedMapReady(_savedMapReady);
}

void AutoRunner::resetRuntimeToStart() {
  _x = 0;
  _y = 0;
  _heading = Maze::N;
  _mapNorthHeading10 = _m ? _m->fusedYaw10() : 0;
  _running = false;
  _reachedGoal = false;
  _launchReanchorAfterTurn = false;
  _standbyAfterTurn = false;
  _state = State::Idle;
  _pendingTurnDir = Maze::N;
  _centerDistMm = _m ? _m->distMm() : 0.0f;
  _straightHeading10 = headingToMapYaw10(_heading);
  _alignHeading10 = _straightHeading10;
  _frontBrakeSnapDist = 0.0f;
  _frontDetectCount = 0;
  _reanchorStartDistMm = _centerDistMm;
  _reanchorLastBackedMm = 0.0f;
  _reanchorLastProgressMs = _m ? millis() : 0;
  _advanceTargetDistMm = 0.0f;
  _wallBaseHeading10 = _straightHeading10;
  _wallBias10 = 0;
  _wallBiasCmd10 = 0;
  _wallLastUpdateMs = 0;
  clearSpeedPreTurn();
}

void AutoRunner::resetIndicators() {
  RGB_setRainbow(false);
  RGB_setArrived(false);
  RGB_setStarted(false);
  RGB_setFingerReady(false);
  RGB_setFingerArming(false);
  RGB_setSavedMapReady(false);
}

bool AutoRunner::startExplore() {
  if (!_m) return false;

  _maze.begin();
  _savedMapReady = false;
  resetRuntimeToStart();
  _running = true;
  _phase = Phase::ExploreToGoal;
  resetIndicators();

  _m->setEnabled(true);
  _m->autoClear();
  _m->setIrEnabled(true);

  beginStartReanchor();
  return true;
}

bool AutoRunner::startSpeedRun() {
  if (!_m || !_savedMapReady) return false;

  resetRuntimeToStart();
  _running = true;
  _phase = Phase::SpeedRunToGoal;
  resetIndicators();

  _m->setEnabled(true);
  _m->autoClear();
  _m->setIrEnabled(true);

  const bool ok = launchSavedSpeedRun();
  if (!ok) {
    enterSpeedRunStandby();
  }
  return ok;
}

void AutoRunner::stop() {
  if (!_m) return;
  _running = false;
  _state = State::Idle;
  _frontBrakeSnapDist = 0.0f;
  _frontDetectCount = 0;
  _reanchorStartDistMm = 0.0f;
  _reanchorLastBackedMm = 0.0f;
  _reanchorLastProgressMs = 0;
  _advanceTargetDistMm = 0.0f;
  _launchReanchorAfterTurn = false;
  _standbyAfterTurn = false;
  _wallBias10 = 0;
  _wallBiasCmd10 = 0;
  _wallLastUpdateMs = 0;
  clearSpeedPreTurn();

  _m->autoClear();
  _m->setEnabled(false);
  resetIndicators();
  RGB_setSavedMapReady(_savedMapReady);
}

bool AutoRunner::clearSavedMap() {
  const bool ok = MazeStorage::clear();
  _savedMapReady = false;
  _maze.begin();
  reset();
  return ok;
}

bool AutoRunner::isGoalCell(int x, int y) const {
  return (x == 7 || x == 8) && (y == 7 || y == 8);
}

bool AutoRunner::isStartCell(int x, int y) const {
  return (x == 0) && (y == 0);
}

int16_t AutoRunner::headingToMapYaw10(uint8_t heading) const {
  const uint8_t dir = (uint8_t)(heading & 0x03u);
  return wrapYaw10((int32_t)_mapNorthHeading10 + (int32_t)dir * 900);
}

void AutoRunner::clearSpeedPreTurn() {
  _speedPreTurnArmed = false;
  _speedPreTurnExecuting = false;
  _speedPreTurnDir = Maze::N;
  _speedPreTurnTriggerDistMm = 0.0f;
  _speedPreTurnCenterDistMm = 0.0f;
}

void AutoRunner::armSpeedPreTurn(bool knownOpenOnly) {
  if (!_m || !inSpeedRun() || !SPEEDRUN_PRETURN_ENABLE) {
    clearSpeedPreTurn();
    return;
  }

  int nx, ny;
  Maze::step(_x, _y, _heading, nx, ny);
  if (!_maze.inBounds(nx, ny)) {
    clearSpeedPreTurn();
    return;
  }

  uint8_t turnDir = _ff.chooseNext(_maze, nx, ny, _heading, knownOpenOnly);
  if (turnDir == 255 && knownOpenOnly) {
    turnDir = _ff.chooseNext(_maze, nx, ny, _heading, false);
  }
  if (turnDir == 255) {
    clearSpeedPreTurn();
    return;
  }

  const uint8_t delta = (uint8_t)((turnDir + 4 - _heading) & 3);
  if (delta != 1 && delta != 3) {
    clearSpeedPreTurn();
    return;
  }

  _speedPreTurnArmed = true;
  _speedPreTurnExecuting = false;
  _speedPreTurnDir = turnDir;
  _speedPreTurnCenterDistMm = _centerDistMm + AUTO_CELL_MM;
  _speedPreTurnTriggerDistMm = _speedPreTurnCenterDistMm - SPEEDRUN_PRETURN_LEAD_MM;
}

bool AutoRunner::irShouldBeOn(float /*phaseMm*/) const {
  return true;
}

bool AutoRunner::wallCorrectionShouldBeOn(float phaseMm) const {
  return (phaseMm >= WALL_CORR_PHASE_START_MM) && (phaseMm <= WALL_CORR_PHASE_END_MM);
}

bool AutoRunner::frontBrakeHit(float phaseMm) const {
  if (!_m) return false;
  if (phaseMm < AUTO_FRONT_BRAKE_ARM_FROM_CENTER_MM) return false;

  const IrSensors& ir = _m->ir();
  return (ir.ir2_mm() <= AUTO_FRONT_BRAKE_TH_MM) ||
         (ir.ir3_mm() <= AUTO_FRONT_BRAKE_TH_MM);
}

void AutoRunner::resetWallBiasState() {
  _wallBias10 = 0;
  _wallBiasCmd10 = 0;
  _wallLastUpdateMs = millis();
  _straightHeading10 = _wallBaseHeading10;
}

void AutoRunner::updateSlowWallCorrection(float phaseMm) {
  if (!_m) return;
  if (!WALL_CORR_ENABLE) {
    _wallBiasCmd10 = 0;
    _wallBias10 = 0;
    _straightHeading10 = _wallBaseHeading10;
    _m->autoSetStraightHeading(_straightHeading10);
    return;
  }

  const uint32_t nowMs = millis();
  if (_wallLastUpdateMs == 0) _wallLastUpdateMs = nowMs;

  bool refreshCmd = false;
  if ((uint32_t)(nowMs - _wallLastUpdateMs) >= WALL_CORR_UPDATE_MS) {
    _wallLastUpdateMs = nowMs;
    refreshCmd = true;
  }

  if (refreshCmd) {
    if (wallCorrectionShouldBeOn(phaseMm)) {
      const auto obs = _m->ir().observeWallError();
      if (obs.valid) {
        const float gain = obs.dualWall ? WALL_CORR_DUAL_GAIN_DEG_PER_MM
                                        : WALL_CORR_SINGLE_GAIN_DEG_PER_MM;
        int cmdDeg = (int)lroundf(gain * obs.errorMm);
        cmdDeg = constrain(cmdDeg, -WALL_CORR_BIAS_MAX_DEG, WALL_CORR_BIAS_MAX_DEG);
        _wallBiasCmd10 = (int16_t)(cmdDeg * 10);
      } else {
        _wallBiasCmd10 = 0;
      }
    } else {
      _wallBiasCmd10 = 0;
    }

    const float rateDegPerS = (_wallBiasCmd10 != 0) ? WALL_CORR_SLEW_DEG_PER_S
                                                     : WALL_CORR_DECAY_DEG_PER_S;
    int maxStepDeg = (int)lroundf(rateDegPerS * ((float)WALL_CORR_UPDATE_MS / 1000.0f));
    if (maxStepDeg < 1) maxStepDeg = 1;
    const int maxStep10 = maxStepDeg * 10;

    const int diff10 = (int)_wallBiasCmd10 - (int)_wallBias10;
    if (diff10 > maxStep10) {
      _wallBias10 = (int16_t)((int)_wallBias10 + maxStep10);
    } else if (diff10 < -maxStep10) {
      _wallBias10 = (int16_t)((int)_wallBias10 - maxStep10);
    } else {
      _wallBias10 = _wallBiasCmd10;
    }
  }

  const int16_t maxBias10 = (int16_t)(WALL_CORR_BIAS_MAX_DEG * 10);
  _wallBias10 = constrain(_wallBias10, (int16_t)-maxBias10, maxBias10);
  _straightHeading10 = wrapYaw10((int32_t)_wallBaseHeading10 - (int32_t)_wallBias10);
  _m->autoSetStraightHeading(_straightHeading10);
}

void AutoRunner::senseWallsAtCenter() {
  if (!_m) return;
  const IrSensors& ir = _m->ir();

  _maze.setVisited(_x, _y, true);

  const uint8_t fwd = _heading;
  const uint8_t left = (uint8_t)((_heading + 3) & 3);
  const uint8_t right = (uint8_t)((_heading + 1) & 3);

  _maze.setWall(_x, _y, left, ir.leftWall());
  _maze.setWall(_x, _y, right, ir.rightWall());

  const bool frontSeen = (ir.ir2_mm() < (float)IR_FRONT_WALL_TH_MM) ||
                         (ir.ir3_mm() < (float)IR_FRONT_WALL_TH_MM);
  _maze.setWall(_x, _y, fwd, frontSeen);
}

void AutoRunner::beginStartReanchor() {
  if (!_m) return;
  _frontDetectCount = 0;
  _alignHeading10 = _m->fusedYaw10();
  _reanchorStartDistMm = _m->distMm();
  _reanchorLastBackedMm = 0.0f;
  _reanchorLastProgressMs = millis();
  _m->autoStartRunDistance(-AUTO_REAR_REANCHOR_BACK_MM, _alignHeading10);
  _state = State::StartReverse;
}

bool AutoRunner::shouldPostTurnReanchor() const {
  if (inSpeedRun()) return false;
  const uint8_t rearDir = Maze::opposite(_heading);
  return _maze.hasWall(_x, _y, rearDir);
}

void AutoRunner::enterSpeedRunStandby() {
  if (!_m) return;

  _running = false;
  _phase = Phase::StandbySpeedRun;
  _state = State::Idle;
  _launchReanchorAfterTurn = false;
  _standbyAfterTurn = false;
  _pendingTurnDir = Maze::N;
  _heading = Maze::N;
  _x = 0;
  _y = 0;
  clearSpeedPreTurn();

  _m->autoClear();
  _m->setEnabled(false);
  resetIndicators();
  RGB_setSavedMapReady(true);
}

void AutoRunner::startTurnTo(uint8_t nextDir) {
  if (!_m) return;

  _pendingTurnDir = nextDir;
  const uint8_t delta = (uint8_t)((nextDir + 4 - _heading) & 3);

  if (delta == 0) {
    _state = State::Idle;
    if (_standbyAfterTurn) {
      enterSpeedRunStandby();
    } else {
      planFromCurrentCell(false);
    }
    return;
  }

  const int angle = (delta == 2) ? 180 : 90;
  const int dir = (delta == 3) ? -1 : +1;
  _m->autoStartTurn(angle, dir);
  _state = State::Turn;
}

bool AutoRunner::launchSavedSpeedRun() {
  if (!_m) return false;

  _ff.computeToCenter(_maze, true);
  uint8_t launchDir = _ff.chooseNext(_maze, _x, _y, Maze::N, true);
  if (launchDir == 255) {
    _ff.computeToCenter(_maze, false);
    launchDir = _ff.chooseNext(_maze, _x, _y, Maze::N, false);
    if (launchDir == 255) {
      return false;
    }
  }

  _frontDetectCount = 0;
  _frontBrakeSnapDist = 0.0f;

  if (launchDir == _heading) {
    beginStartReanchor();
    return true;
  }

  _launchReanchorAfterTurn = true;
  _standbyAfterTurn = false;
  startTurnTo(launchDir);
  return true;
}

void AutoRunner::beginPostTurnReanchor() {
  if (!_m) return;
  if (!shouldPostTurnReanchor()) {
    _centerDistMm = _m->distMm();
    _alignHeading10 = inSpeedRun() ? headingToMapYaw10(_heading) : _m->fusedYaw10();
    _wallBaseHeading10 = _alignHeading10;
    resetWallBiasState();
    _m->setIrEnabled(true);
    senseWallsAtCenter();
    planFromCurrentCell(false);
    return;
  }

  _frontDetectCount = 0;
  _alignHeading10 = _m->fusedYaw10();
  _reanchorStartDistMm = _m->distMm();
  _reanchorLastBackedMm = 0.0f;
  _reanchorLastProgressMs = millis();
  _m->autoStartRunDistance(-AUTO_REAR_REANCHOR_BACK_MM, _alignHeading10);
  _state = State::TurnReverse;
}

bool AutoRunner::canSaveCurrentMap() const {
  FloodFill ff;
  ff.computeToCenter(_maze, true);
  return ff.chooseNext(_maze, 0, 0, Maze::N, true) != 255;
}

void AutoRunner::startCenterAdvance(State advanceState) {
  if (!_m) return;
  _alignHeading10 = inSpeedRun() ? headingToMapYaw10(_heading) : _m->fusedYaw10();
  _wallBaseHeading10 = _alignHeading10;
  _wallBias10 = 0;
  _wallBiasCmd10 = 0;
  _wallLastUpdateMs = 0;
  _advanceTargetDistMm = _m->distMm() + AUTO_REAR_REANCHOR_TO_CENTER_MM;
  _m->autoStartStraight(_alignHeading10, false);
  _state = advanceState;
}

void AutoRunner::finishStartReanchor() {
  if (!_m) return;
  _centerDistMm = _advanceTargetDistMm;
  _advanceTargetDistMm = 0.0f;
  _wallBaseHeading10 = _alignHeading10;
  _straightHeading10 = _wallBaseHeading10;
  _wallBias10 = 0;
  _wallBiasCmd10 = 0;
  _wallLastUpdateMs = 0;
  _m->setIrEnabled(true);
  senseWallsAtCenter();
  planFromCurrentCell(false);
}

void AutoRunner::finishPostTurnReanchor() {
  if (!_m) return;
  _centerDistMm = _advanceTargetDistMm;
  _advanceTargetDistMm = 0.0f;
  _wallBaseHeading10 = _alignHeading10;
  _straightHeading10 = _wallBaseHeading10;
  _wallBias10 = 0;
  _wallBiasCmd10 = 0;
  _wallLastUpdateMs = 0;
  _m->setIrEnabled(true);
  senseWallsAtCenter();
  planFromCurrentCell(false);
}

void AutoRunner::startTurnNow(uint8_t nextDir) {
  startTurnTo(nextDir);
}

void AutoRunner::planFromCurrentCell(bool fromMoving) {
  if (!_m) return;
  if (!_running && _state != State::GoalHold) return;

  if (_phase == Phase::ExploreToGoal && isGoalCell(_x, _y)) {
    _reachedGoal = true;
    _phase = Phase::ReturnToStart;
  }

  if (_phase == Phase::ReturnToStart && isStartCell(_x, _y)) {
    _savedMapReady = canSaveCurrentMap() && MazeStorage::save(_maze);
    if (!_savedMapReady) {
      _running = false;
      _state = State::Halted;
      _m->autoClear();
      _m->setEnabled(false);
      resetIndicators();
      return;
    }

    if (_heading == Maze::N) {
      enterSpeedRunStandby();
    } else {
      _standbyAfterTurn = true;
      _launchReanchorAfterTurn = false;
      startTurnTo(Maze::N);
    }
    return;
  }

  if (_phase == Phase::SpeedRunToGoal && isGoalCell(_x, _y)) {
    _running = false;
    _phase = Phase::Finished;
    _state = State::GoalHold;
    resetIndicators();
    RGB_setRainbow(true);
    
    _m->autoBrake(AUTO_GOAL_BRAKE_MS, true);
    return;
  }

  bool knownOpenOnly = false;
  if (_phase == Phase::ReturnToStart) {
    // Keep solving on the way home instead of simply replaying the already-known
    // corridor. Unknown edges stay traversable here so the mouse can discover a
    // genuinely shorter return route if one exists.
    _ff.computeToCell(_maze, 0, 0, false);
    knownOpenOnly = false;
  } else if (_phase == Phase::SpeedRunToGoal) {
    // Speed-run should prefer the shortest path on the learned graph. If the
    // learned graph is not yet fully connected, fall back to solver-style flood
    // instead of stopping in place mid-run.
    _ff.computeToCenter(_maze, true);
    knownOpenOnly = true;
  } else {
    _ff.computeToCenter(_maze, false);
    knownOpenOnly = false;
  }

  uint8_t nextDir = _ff.chooseNext(_maze, _x, _y, _heading, knownOpenOnly);

  if (nextDir == 255 && _phase == Phase::SpeedRunToGoal) {
    _ff.computeToCenter(_maze, false);
    nextDir = _ff.chooseNext(_maze, _x, _y, _heading, false);
    knownOpenOnly = false;
  }

  if (nextDir == 255) {
    _running = false;
    _state = State::Halted;
    _m->autoClear();
    _m->setEnabled(false);
    resetIndicators();
    RGB_setSavedMapReady(_savedMapReady);
    return;
  }

  _frontDetectCount = 0;
  _frontBrakeSnapDist = 0.0f;

  if (nextDir == _heading) {
    if (inSpeedRun()) {
      armSpeedPreTurn(knownOpenOnly);
    } else {
      clearSpeedPreTurn();
    }

    if (_state != State::Straight) {
      resetWallBiasState();
      _m->autoStartStraight(_straightHeading10, false);
    } else {
      _m->autoSetStraightHeading(_straightHeading10);
    }
    _m->autoSetStraightWallCorrection(false);
    _state = State::Straight;
    return;
  }

  clearSpeedPreTurn();
  _pendingTurnDir = nextDir;
  if (fromMoving && AUTO_TURN_BRAKE_MS > 0) {
    _state = State::TurnBrake;
    _m->autoBrake(AUTO_TURN_BRAKE_MS, false);
  } else {
    startTurnTo(nextDir);
  }
}

void AutoRunner::handleForwardCenterSnap(float snappedCenterDist) {
  if (!_m) return;

  const int oldX = _x;
  const int oldY = _y;

  int nx, ny;
  Maze::step(_x, _y, _heading, nx, ny);
  if (_maze.inBounds(nx, ny)) {
    _maze.setOpen(oldX, oldY, _heading);
    _x = nx;
    _y = ny;
  } else {
    _running = false;
    _state = State::Halted;
    _m->autoClear();
    _m->setEnabled(false);
    resetIndicators();
    RGB_setSavedMapReady(_savedMapReady);
    return;
  }

  _centerDistMm = snappedCenterDist;
  _frontDetectCount = 0;
  _frontBrakeSnapDist = 0.0f;
  _m->setIrEnabled(true);

  if (inSpeedRun() && SPEEDRUN_PRETURN_ENABLE) {
    // In speed-run, freeze map updates to avoid corrupting the learned graph at high speed.
    // Also, while a pre-turn is armed/executing, do not re-plan at the turn-cell center.
    if (!_speedPreTurnArmed && !_speedPreTurnExecuting) {
      planFromCurrentCell(true);
    }
    return;
  }

  senseWallsAtCenter();
  planFromCurrentCell(true);
}

void AutoRunner::updateStraight() {
  if (!_m) return;
  if (_state != State::Straight) return;

  const float distNow = _m->distMm();
  const float phase = distNow - _centerDistMm;

  if (inSpeedRun() && SPEEDRUN_PRETURN_ENABLE) {
    _m->setIrEnabled(true);
    _m->autoSetStraightWallCorrection(false);

    while ((_state == State::Straight) && ((_m->distMm() - _centerDistMm) >= AUTO_CELL_MM)) {
      handleForwardCenterSnap(_centerDistMm + AUTO_CELL_MM);
    }
    if (_state != State::Straight) return;

    const float distSr = _m->distMm();
    const float phaseSr = distSr - _centerDistMm;
    updateSlowWallCorrection(phaseSr);

    const IrSensors& ir = _m->ir();
    const bool frontPreTurnHit =
        (ir.ir2_mm() <= SPEEDRUN_PRETURN_FRONT_TRIGGER_MM) ||
        (ir.ir3_mm() <= SPEEDRUN_PRETURN_FRONT_TRIGGER_MM);
    const bool inPreTurnWindow =
        (distSr + SPEEDRUN_PRETURN_TRIGGER_TOL_MM) >= _speedPreTurnTriggerDistMm;

    if (_speedPreTurnArmed && !_speedPreTurnExecuting && inPreTurnWindow && frontPreTurnHit) {
      const uint8_t delta = (uint8_t)((_speedPreTurnDir + 4 - _heading) & 3);
      if (delta == 1 || delta == 3) {
        _pendingTurnDir = _speedPreTurnDir;
        _speedPreTurnArmed = false;
        _speedPreTurnExecuting = true;
        _state = State::Turn;
        _m->autoStartSpeedTurn((delta == 3) ? -1 : +1);
        return;
      }
      clearSpeedPreTurn();
    }

    if (!_speedPreTurnArmed && frontBrakeHit(phaseSr)) {
      if (_frontDetectCount < 255) _frontDetectCount++;
    } else {
      _frontDetectCount = 0;
    }

    if (_frontDetectCount >= AUTO_FRONT_BRAKE_CONFIRM_CYCLES) {
      _frontDetectCount = 0;
      _frontBrakeSnapDist = _centerDistMm + AUTO_CELL_MM;
      _state = State::FrontBrake;
      _m->autoBrake(AUTO_FRONT_BRAKE_MS, false);
      return;
    }
    return;
  }

  _m->setIrEnabled(irShouldBeOn(phase));
  _m->autoSetStraightWallCorrection(false);
  updateSlowWallCorrection(phase);

  if (frontBrakeHit(phase)) {
    if (_frontDetectCount < 255) _frontDetectCount++;
  } else {
    _frontDetectCount = 0;
  }

  if (_frontDetectCount >= AUTO_FRONT_BRAKE_CONFIRM_CYCLES) {
    _frontDetectCount = 0;
    _frontBrakeSnapDist = _centerDistMm + AUTO_CELL_MM;
    _state = State::FrontBrake;
    _m->autoBrake(AUTO_FRONT_BRAKE_MS, false);
    return;
  }

  while ((_state == State::Straight) && ((distNow - _centerDistMm) >= AUTO_CELL_MM)) {
    handleForwardCenterSnap(_centerDistMm + AUTO_CELL_MM);
  }
}

void AutoRunner::updateReverseReanchor(State reverseState, State advanceState) {
  if (!_m) return;
  if (_state != reverseState) return;

  const uint32_t nowMs = millis();
  const float distNow = _m->distMm();
  const float backedMm = _reanchorStartDistMm - distNow;

  if (backedMm > _reanchorLastBackedMm + 0.5f) {
    _reanchorLastBackedMm = backedMm;
    _reanchorLastProgressMs = nowMs;
  }

  const bool hitMaxBack = (backedMm >= AUTO_REAR_REANCHOR_BACK_MM - POS_TOL_MM);
  const bool stallAfterArm =
      (backedMm >= AUTO_REAR_ALIGN_STALL_ARM_MM) &&
      ((nowMs - _reanchorLastProgressMs) >= AUTO_REAR_ALIGN_STALL_MS);

  if (!_m->autoRunningDistance()) {
    startCenterAdvance(advanceState);
    return;
  }

  if (hitMaxBack || stallAfterArm) {
    _m->autoClear();
    startCenterAdvance(advanceState);
  }
}

void AutoRunner::updateCenterAdvance(State advanceState) {
  if (!_m) return;
  if (_state != advanceState) return;

  _m->setIrEnabled(true);
  _m->autoSetStraightHeading(_alignHeading10);
  _m->autoSetStraightWallCorrection(false);

  const float distNow = _m->distMm();
  if (distNow >= _advanceTargetDistMm) {
    if (advanceState == State::StartCenterAdvance) {
      finishStartReanchor();
    } else {
      finishPostTurnReanchor();
    }
  }
}

void AutoRunner::update() {
  if (!_m) return;

  switch (_state) {
    case State::StartReverse:
      updateReverseReanchor(State::StartReverse, State::StartCenterAdvance);
      break;

    case State::StartCenterAdvance:
      updateCenterAdvance(State::StartCenterAdvance);
      break;

    case State::Straight:
      if (_running) updateStraight();
      break;

    case State::FrontBrake:
      if (_m->consumeAutoBrakeDone()) {
        handleForwardCenterSnap(_frontBrakeSnapDist);
      }
      break;

    case State::TurnBrake:
      if (_m->consumeAutoBrakeDone()) {
        startTurnTo(_pendingTurnDir);
      }
      break;

    case State::Turn:
      if (_m->consumeAutoTurnDone()) {
        if (_speedPreTurnExecuting && inSpeedRun() && _m->autoInStraight()) {
          // A speed-turn is planned from the next cell ahead. Once the turn completes,
          // commit that forward cell first, then rotate the logical heading.
          const uint8_t entryHeading = _heading;
          int nx, ny;
          Maze::step(_x, _y, entryHeading, nx, ny);
          if (_maze.inBounds(nx, ny)) {
            _maze.setOpen(_x, _y, entryHeading);
            _x = nx;
            _y = ny;
          } else {
            _running = false;
            _state = State::Halted;
            _m->autoClear();
            _m->setEnabled(false);
            resetIndicators();
            RGB_setSavedMapReady(_savedMapReady);
            return;
          }

          _centerDistMm = _speedPreTurnCenterDistMm;
          _heading = _pendingTurnDir;
          _alignHeading10 = headingToMapYaw10(_heading);
          _wallBaseHeading10 = _alignHeading10;
          _straightHeading10 = _alignHeading10;
          _wallBias10 = 0;
          _wallBiasCmd10 = 0;
          _wallLastUpdateMs = 0;
          _frontDetectCount = 0;
          _frontBrakeSnapDist = 0.0f;
          clearSpeedPreTurn();
          _m->setIrEnabled(true);
          _state = State::Straight;
          planFromCurrentCell(true);
          return;
        } else {
          _heading = _pendingTurnDir;
          if (_standbyAfterTurn) {
            enterSpeedRunStandby();
          } else if (_launchReanchorAfterTurn) {
            _launchReanchorAfterTurn = false;
            beginStartReanchor();
          } else {
            beginPostTurnReanchor();
          }
        }
      }
      break;

    case State::TurnReverse:
      updateReverseReanchor(State::TurnReverse, State::TurnCenterAdvance);
      break;

    case State::TurnCenterAdvance:
      updateCenterAdvance(State::TurnCenterAdvance);
      break;

    case State::GoalHold:
    case State::Halted:
    case State::Idle:
    default:
      break;
  }
}
