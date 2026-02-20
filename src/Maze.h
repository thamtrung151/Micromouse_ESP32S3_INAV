#pragma once

#include <Arduino.h>

// 16x16 micromouse maze representation.
// Walls are stored per-cell as a 4-bit mask: N,E,S,W.
// Unknown edges are represented by the absence of a wall bit (i.e., treated as open).
// This matches the requirement: DO NOT treat unknown as wall.

class Maze {
public:
  static constexpr uint8_t N = 0;
  static constexpr uint8_t E = 1;
  static constexpr uint8_t S = 2;
  static constexpr uint8_t W = 3;

  static constexpr uint8_t SIZE = 16;

  void begin();

  bool inBounds(int x, int y) const {
    return x >= 0 && x < (int)SIZE && y >= 0 && y < (int)SIZE;
  }

  uint8_t walls(int x, int y) const { return _walls[x][y]; }
  bool hasWall(int x, int y, uint8_t dir) const { return (_walls[x][y] & (1u << dir)) != 0; }

  // Set/clear a wall and mirror it in the neighbor cell.
  void setWall(int x, int y, uint8_t dir, bool present);

  // Mark a cell visited (useful for debugging / future improvements).
  void setVisited(int x, int y, bool v=true) { if (inBounds(x,y)) _visited[x][y] = v; }
  bool visited(int x, int y) const { return inBounds(x,y) ? _visited[x][y] : false; }

  static uint8_t opposite(uint8_t dir) { return (uint8_t)((dir + 2) & 3); }

  static void step(int x, int y, uint8_t dir, int &nx, int &ny) {
    nx = x; ny = y;
    switch (dir) {
      case N: ny = y + 1; break;
      case E: nx = x + 1; break;
      case S: ny = y - 1; break;
      case W: nx = x - 1; break;
      default: break;
    }
  }

private:
  uint8_t _walls[SIZE][SIZE] = {0};
  bool    _visited[SIZE][SIZE] = {false};
};
