#pragma once

#include <Arduino.h>

#include "Motion.h"
#include "Maze.h"
#include "FloodFill.h"

// Basic micromouse auto-solver using BFS flood-fill.
// Notes:
// - Unknown edges are treated as OPEN (not as walls).
// - Outer boundary is fixed walls.
// - At each cell center, we sense front/left/right walls using IR and update the map.
// - Then we recompute the distance map to the center and take the locally best neighbor.

class AutoRunner {
public:
  void begin(Motion& motion);
  void reset();

  void start();
  void stop();
  bool running() const { return _running; }

  // Call frequently from loop(); Motion::update() must be called separately.
  void update();

  int x() const { return _x; }
  int y() const { return _y; }
  uint8_t heading() const { return _heading; }
  bool reachedGoal() const { return _reachedGoal; }

private:
  void senseWalls();
  void planAndMove();
  bool isGoalCell(int x, int y) const;

private:
  Motion* _m = nullptr;
  Maze _maze;
  FloodFill _ff;

  bool _running = false;
  bool _reachedGoal = false;

  int _x = 0;
  int _y = 0;
  uint8_t _heading = Maze::N;

  // pending position update after a commanded cell move
  bool _pendingAdvance = false;
  uint8_t _advanceDir = Maze::N;

  // persistent step buffer (Motion stores pointer)
  Step _stepBuf[3];
  size_t _stepCount = 0;
};
