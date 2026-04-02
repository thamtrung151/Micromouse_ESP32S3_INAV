#include "Maze.h"

void Maze::begin() {
  // Clear
  for (uint8_t x = 0; x < SIZE; x++) {
    for (uint8_t y = 0; y < SIZE; y++) {
      _walls[x][y] = 0;
      _known[x][y] = 0;
      _visited[x][y] = false;
    }
  }

  // Outer boundaries are always walls and always known.
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

  setKnownEdgeLocal(x, y, dir, true);

  const uint8_t mask = (1u << dir);
  if (present) _walls[x][y] |= mask;
  else         _walls[x][y] &= (uint8_t)~mask;

  // Mirror into neighbor
  int nx, ny;
  step(x, y, dir, nx, ny);
  if (!inBounds(nx, ny)) return;

  const uint8_t od = opposite(dir);
  setKnownEdgeLocal(nx, ny, od, true);

  const uint8_t omask = (1u << od);
  if (present) _walls[nx][ny] |= omask;
  else         _walls[nx][ny] &= (uint8_t)~omask;
}

void Maze::setOpen(int x, int y, uint8_t dir) {
  setWall(x, y, dir, false);
}

void Maze::exportState(uint8_t walls[SIZE][SIZE],
                       uint8_t known[SIZE][SIZE],
                       uint8_t visited[VISITED_BYTES]) const {
  for (uint8_t i = 0; i < VISITED_BYTES; i++) visited[i] = 0;

  for (uint8_t x = 0; x < SIZE; x++) {
    for (uint8_t y = 0; y < SIZE; y++) {
      walls[x][y] = _walls[x][y] & 0x0Fu;
      known[x][y] = _known[x][y] & 0x0Fu;

      const uint16_t idx = (uint16_t)y * SIZE + x;
      if (_visited[x][y]) {
        visited[idx >> 3] |= (uint8_t)(1u << (idx & 7u));
      }
    }
  }
}

bool Maze::importState(const uint8_t walls[SIZE][SIZE],
                       const uint8_t known[SIZE][SIZE],
                       const uint8_t visited[VISITED_BYTES]) {
  for (uint8_t x = 0; x < SIZE; x++) {
    for (uint8_t y = 0; y < SIZE; y++) {
      _walls[x][y] = walls[x][y] & 0x0Fu;
      _known[x][y] = known[x][y] & 0x0Fu;

      const uint16_t idx = (uint16_t)y * SIZE + x;
      _visited[x][y] = (visited[idx >> 3] & (uint8_t)(1u << (idx & 7u))) != 0;
    }
  }

  if (isConsistent()) return true;

  begin();
  return false;
}

bool Maze::isConsistent() const {
  for (int x = 0; x < (int)SIZE; x++) {
    for (int y = 0; y < (int)SIZE; y++) {
      const uint8_t walls = _walls[x][y] & 0x0Fu;
      const uint8_t known = _known[x][y] & 0x0Fu;

      for (uint8_t dir = 0; dir < 4; dir++) {
        const bool wall = (walls & (1u << dir)) != 0;
        const bool edgeKnown = (known & (1u << dir)) != 0;

        int nx, ny;
        step(x, y, dir, nx, ny);
        if (!inBounds(nx, ny)) {
          if (!edgeKnown || !wall) return false;
          continue;
        }

        const uint8_t od = opposite(dir);
        const bool neighborKnown = (_known[nx][ny] & (1u << od)) != 0;
        const bool neighborWall = (_walls[nx][ny] & (1u << od)) != 0;
        if (edgeKnown != neighborKnown) return false;
        if (wall != neighborWall) return false;
      }
    }
  }

  return true;
}
