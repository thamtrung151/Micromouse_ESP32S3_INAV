#include "AutoRunner.h"
#include "rgb.h"

static inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi ? hi : v); }

void AutoRunner::begin(Motion& motion) {
  _m = &motion;
  _maze.begin();
  _ff.computeToCenter(_maze);
  reset();
}

void AutoRunner::reset() {
  _maze.begin();
  _x = 0;
  _y = 0;
  _heading = Maze::N; // typical start: facing north
  _running = false;
  _reachedGoal = false;
  _pendingAdvance = false;
  _advanceDir = Maze::N;
}

void AutoRunner::start() {
  if (!_m) return;
  _running = true;
  _reachedGoal = false;
  _pendingAdvance = false;
  // Ensure Motion is enabled and idle; next update() will schedule the first move.
  _m->setEnabled(true);
}

void AutoRunner::stop() {
  if (!_m) return;
  _running = false;
  _m->setEnabled(false);
}

bool AutoRunner::isGoalCell(int x, int y) const {
  return (x == 7 || x == 8) && (y == 7 || y == 8);
}

void AutoRunner::senseWalls() {
  if (!_m) return;
  const IrSensors& ir = _m->ir();

  // Update current cell info
  _maze.setVisited(_x, _y, true);

  // Relative directions
  const uint8_t fwd = _heading;
  const uint8_t left = (uint8_t)((_heading + 3) & 3);
  const uint8_t right = (uint8_t)((_heading + 1) & 3);

  // Only set walls we positively detect (inverted sensors => wall if ADC < threshold)
  // This keeps "unknown" edges as open until we learn otherwise.
  _maze.setWall(_x, _y, left,  ir.leftWall());
  _maze.setWall(_x, _y, right, ir.rightWall());
  _maze.setWall(_x, _y, fwd,   ir.frontWall());
}

void AutoRunner::planAndMove() {
  if (!_m) return;
  if (_reachedGoal) return;

  // Recompute distances every step (simple + robust for small maze)
  _ff.computeToCenter(_maze);
  const uint8_t nextDir = _ff.chooseNext(_maze, _x, _y, _heading);

  if (nextDir == 255) {
    // no legal move (shouldn't happen often unless boxed in by walls)
    _running = false;
    _m->setEnabled(false);
    return;
  }

  // Create a short script: optional turn + one cell forward
  _stepCount = 0;

  uint8_t delta = (uint8_t)((nextDir + 4 - _heading) & 3);
  if (delta == 1) {
    if (AUTO_TURN_STYLE == AUTO_TURN_DRIFT) {
      _stepBuf[_stepCount++] = {StepType::DriftDeg, 90, +1};
    } else {
      _stepBuf[_stepCount++] = {StepType::TurnDeg, 90, +1};
    }
    _heading = nextDir;
  } else if (delta == 3) {
    if (AUTO_TURN_STYLE == AUTO_TURN_DRIFT) {
      _stepBuf[_stepCount++] = {StepType::DriftDeg, 90, -1};
    } else {
      _stepBuf[_stepCount++] = {StepType::TurnDeg, 90, -1};
    }
    _heading = nextDir;
  } else if (delta == 2) {
    if (AUTO_TURN_STYLE == AUTO_TURN_DRIFT) {
      // 180 drift as two 90 drifts keeps the arc calibration consistent
      _stepBuf[_stepCount++] = {StepType::DriftDeg, 90, +1};
      _stepBuf[_stepCount++] = {StepType::DriftDeg, 90, +1};
    } else {
      _stepBuf[_stepCount++] = {StepType::TurnDeg, 180, +1};
    }
    _heading = nextDir;
  } else {
    // delta == 0 => already facing correct direction
  }

  _stepBuf[_stepCount++] = {StepType::MoveCells, 1, 0};

  // Schedule position update when motion completes.
  _pendingAdvance = true;
  _advanceDir = nextDir;

  _m->setEnabled(true);
  _m->loadScript(_stepBuf, _stepCount);
}

void AutoRunner::update() {
  if (!_m) return;
  if (!_running) return;

  // When current motion finishes, update logical position, sense, and schedule next.
  if (_m->done()) {
    if (_pendingAdvance) {
      int nx, ny;
      Maze::step(_x, _y, _advanceDir, nx, ny);
      if (_maze.inBounds(nx, ny)) {
        _x = nx;
        _y = ny;
      }
      _pendingAdvance = false;
    }

    // Sense walls at cell center, then decide next action
    senseWalls();

    if (isGoalCell(_x, _y)) {
      _reachedGoal = true;
      RGB_setArrived(true);
      _running = false;
      _m->setEnabled(false);
      return;
    }

    planAndMove();
  }
}


