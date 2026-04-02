#pragma once

#include <Arduino.h>

// 16x16 micromouse maze representation.
// Walls are stored per-cell as a 4-bit mask: N,E,S,W.
// Known-edge state is stored separately so the solver can distinguish
// between "unknown" and "known open" passages when needed.

class Maze {
public:
  static constexpr uint8_t N = 0;
  static constexpr uint8_t E = 1;
  static constexpr uint8_t S = 2;
  static constexpr uint8_t W = 3;

  static constexpr uint8_t SIZE = 16;
  static constexpr size_t VISITED_BYTES = (SIZE * SIZE + 7u) / 8u;

  void begin();

  bool inBounds(int x, int y) const {
    return x >= 0 && x < (int)SIZE && y >= 0 && y < (int)SIZE;
  }

  uint8_t walls(int x, int y) const { return _walls[x][y]; }
  bool hasWall(int x, int y, uint8_t dir) const { return (_walls[x][y] & (1u << dir)) != 0; }
  bool knowsEdge(int x, int y, uint8_t dir) const {
    return inBounds(x, y) ? ((_known[x][y] & (1u << dir)) != 0) : false;
  }
  bool isKnownOpen(int x, int y, uint8_t dir) const {
    return knowsEdge(x, y, dir) && !hasWall(x, y, dir);
  }

  // Set/clear a wall and mirror it in the neighbor cell.
  // Calling this also marks the edge as known on both sides.
  void setWall(int x, int y, uint8_t dir, bool present);

  // Mark an edge as known-open on both sides.
  void setOpen(int x, int y, uint8_t dir);

  // Mark a cell visited.
  void setVisited(int x, int y, bool v=true) { if (inBounds(x,y)) _visited[x][y] = v; }
  bool visited(int x, int y) const { return inBounds(x,y) ? _visited[x][y] : false; }

  void exportState(uint8_t walls[SIZE][SIZE],
                   uint8_t known[SIZE][SIZE],
                   uint8_t visited[VISITED_BYTES]) const;
  bool importState(const uint8_t walls[SIZE][SIZE],
                   const uint8_t known[SIZE][SIZE],
                   const uint8_t visited[VISITED_BYTES]);
  bool isConsistent() const;

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
  void setKnownEdgeLocal(int x, int y, uint8_t dir, bool known=true) {
    if (!inBounds(x, y)) return;
    const uint8_t mask = (1u << dir);
    if (known) _known[x][y] |= mask;
    else       _known[x][y] &= (uint8_t)~mask;
  }

private:
  uint8_t _walls[SIZE][SIZE] = {0};
  uint8_t _known[SIZE][SIZE] = {0};
  bool    _visited[SIZE][SIZE] = {false};
};
