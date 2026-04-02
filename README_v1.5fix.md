# micromouse v1.5fix

## Fixes in this package

- Return phase now floods back to `(0,0)` with unknown edges treated as traversable, so the mouse keeps solving and can discover a shorter way home instead of just replaying the already-known route.
- Speed-run still prefers the shortest path on the learned map, but now falls back to solver-style flood if the learned graph is temporarily incomplete, which avoids the stop-in-place failure seen in v1.5.
- Removed stray top-level `config.h`; use `src/config.h`.
