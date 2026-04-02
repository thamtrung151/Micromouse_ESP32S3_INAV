#pragma once

#include <Arduino.h>
#include "Maze.h"

// BFS flood-fill distance map to a goal region / cell.
class FloodFill {
public:
  static constexpr uint8_t INF = 255;

  void computeToCenter(const Maze& maze, bool knownOpenOnly = false);
  void computeToCell(const Maze& maze, uint8_t goalX, uint8_t goalY, bool knownOpenOnly = false);

  uint8_t dist(int x, int y) const { return _d[x][y]; }

  // Choose next direction from (x,y) given current heading.
  // Returns dir (Maze::N/E/S/W). If no move is possible, returns 255.
  uint8_t chooseNext(const Maze& maze, int x, int y, uint8_t heading, bool knownOpenOnly = false) const;

private:
  uint8_t _d[Maze::SIZE][Maze::SIZE];
};
