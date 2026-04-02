#pragma once

#include <Arduino.h>

#include "Maze.h"

class MazeStorage {
public:
  static bool load(Maze& maze);
  static bool save(const Maze& maze);
  static bool clear();
};
