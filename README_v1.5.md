# micromouse v1.5 changes

## What changed
- After the first center hit, the mouse still replans back to `(0,0)` from the learned maze.
- When it reaches the start cell, it now automatically launches a **speedrun** back to the center.
- The speedrun uses the **same straight/turn speed settings as solve mode**; it only changes the planner mode:
  - shortest path is taken from the learned map
  - only the **start cell** does a rear re-anchor
  - all later turns in speedrun skip the rear re-anchor/back-up step
- Return-to-start planning is now more robust:
  - it first prefers a shortest path on **known-open** edges
  - if that graph is temporarily disconnected, it falls back to normal flood-fill instead of halting immediately

## Auto phases now
1. Explore to center (`unknown` edges treated as open)
2. Return to start (`known-open` edges preferred)
3. Speedrun to center (`known-open` shortest path, no post-turn rear re-anchor)
4. Brake and hold at the center goal

## Files changed
- `src/AutoRunner.h`
- `src/AutoRunner.cpp`
