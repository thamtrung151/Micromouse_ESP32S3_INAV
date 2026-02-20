#include "FloodFill.h"

struct Node { uint8_t x, y; };

void FloodFill::computeToCenter(const Maze& maze) {
  // Init all to INF
  for (uint8_t x = 0; x < Maze::SIZE; x++) {
    for (uint8_t y = 0; y < Maze::SIZE; y++) {
      _d[x][y] = INF;
    }
  }

  // Goal: center 2x2 block (7,7), (7,8), (8,7), (8,8)
  Node q[Maze::SIZE * Maze::SIZE];
  int qh = 0, qt = 0;

  auto push = [&](uint8_t gx, uint8_t gy) {
    if (gx < Maze::SIZE && gy < Maze::SIZE) {
      _d[gx][gy] = 0;
      q[qt++] = {gx, gy};
    }
  };

  push(7, 7);
  push(7, 8);
  push(8, 7);
  push(8, 8);

  while (qh < qt) {
    Node n = q[qh++];
    const uint8_t base = _d[n.x][n.y];

    for (uint8_t dir = 0; dir < 4; dir++) {
      if (maze.hasWall(n.x, n.y, dir)) continue; // wall blocks
      int nx, ny;
      Maze::step(n.x, n.y, dir, nx, ny);
      if (!maze.inBounds(nx, ny)) continue;
      if (_d[nx][ny] != INF) continue;
      _d[nx][ny] = (uint8_t)(base + 1);
      q[qt++] = {(uint8_t)nx, (uint8_t)ny};
    }
  }
}

uint8_t FloodFill::chooseNext(const Maze& maze, int x, int y, uint8_t heading) const {
  if (!maze.inBounds(x, y)) return 255;
  const uint8_t cur = _d[x][y];
  if (cur == INF) return 255;

  // Candidate directions, tie-broken by preference:
  // forward, left, right, back (common micromouse heuristic)
  uint8_t pref[4] = {
      heading,
      (uint8_t)((heading + 3) & 3),
      (uint8_t)((heading + 1) & 3),
      (uint8_t)((heading + 2) & 3)
  };

  uint8_t bestDir = 255;
  uint8_t bestD = INF;

  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t dir = pref[i];
    if (maze.hasWall(x, y, dir)) continue;
    int nx, ny;
    Maze::step(x, y, dir, nx, ny);
    if (!maze.inBounds(nx, ny)) continue;
    const uint8_t nd = _d[nx][ny];
    if (nd < bestD) {
      bestD = nd;
      bestDir = dir;
    }
  }

  return bestDir;
}
