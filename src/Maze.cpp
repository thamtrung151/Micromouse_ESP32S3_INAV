#include "Maze.h"

void Maze::begin() {
  // Clear
  for (uint8_t x = 0; x < SIZE; x++) {
    for (uint8_t y = 0; y < SIZE; y++) {
      _walls[x][y] = 0;
      _visited[x][y] = false;
    }
  }

  // Outer boundaries are always walls.
  for (uint8_t x = 0; x < SIZE; x++) {
    setWall(x, 0, S, true);
    setWall(x, SIZE - 1, N, true);
  }
  for (uint8_t y = 0; y < SIZE; y++) {
    setWall(0, y, W, true);
    setWall(SIZE - 1, y, E, true);
  }
}

void Maze::setWall(int x, int y, uint8_t dir, bool present) {
  if (!inBounds(x, y)) return;
  const uint8_t mask = (1u << dir);
  if (present) _walls[x][y] |= mask;
  else         _walls[x][y] &= (uint8_t)~mask;

  // Mirror into neighbor
  int nx, ny;
  step(x, y, dir, nx, ny);
  if (!inBounds(nx, ny)) return;
  const uint8_t od = opposite(dir);
  const uint8_t omask = (1u << od);
  if (present) _walls[nx][ny] |= omask;
  else         _walls[nx][ny] &= (uint8_t)~omask;
}
