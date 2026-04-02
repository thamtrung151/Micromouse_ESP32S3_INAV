# Micromouse v1.4

## What changed

- AUTO mode no longer stops permanently at the center goal.
- When the mouse first reaches the goal cell, it immediately replans a shortest path back to `(0,0)`.
- The return path is computed from the maze knowledge collected during the outbound run.
- Unknown edges are still treated as open while exploring toward the center.
- On the return leg, only **known** passages are used, so the mouse can choose a different and shorter route back than the exploratory route it used to reach the goal.
- The run finishes only after the mouse returns to the start cell and brakes there.

## Implementation notes

- `Maze` now tracks whether each edge is known.
- Sensed left/right/front edges are marked known every time the mouse reaches a cell center.
- Traversed edges are marked known-open when the mouse moves into the next cell.
- `FloodFill` now supports:
  - flood to center (exploration mode)
  - flood to a specific cell such as `(0,0)`
  - optional filtering to use only known-open edges
- `AutoRunner` switches mode at the goal:
  - outbound: flood-fill to center, unknown treated as open
  - inbound: flood-fill to start, only known passages allowed
