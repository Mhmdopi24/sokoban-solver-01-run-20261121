[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Mhmdopi24/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Mhmdopi24/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Sokoban solver

A push-optimal Sokoban solver for the [make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty "Build the best Sokoban solver, 21 November 2026" (`sokoban-solver-01-run-20261121`).

## How it works

A generated level is a one-wide hallway with 1–4 rooms off it, and a box
pushed out of a room can never be pushed back — so each room is an
independent sub-problem and the level's push-optimal count is the sum of
the rooms' optima. The solver:

1. splits the level into rooms by removing the hallway row the player
   starts in (each room keeps its single door cell);
2. solves each room push-optimally with A\* over pushes, using an
   admissible minimum-assignment box-to-goal heuristic, dead-square
   pruning (reverse reachability from the goals) and frozen-box
   deadlock pruning, with player positions canonicalised to their
   reachable region;
3. falls back to greedy best-first search if the optimal search ever
   exhausts its fixed node budget, so an instance is never left
   unanswered;
4. stitches the room solutions together with shortest player walks
   through the hallway, then replays the full move string on the level
   to validate it before writing it out.

The whole run is budgeted by fixed node-expansion counts, never by the
clock, so two executions write byte-identical output.

## Usage

```sh
node main.ts   # reads /input/<id>/level.txt, writes /output/<id>/solution.txt
```

Node 22, no dependencies, no network. `main.ts` is plain TypeScript with
erasable syntax (runs as-is under Node 22's type stripping).

## Practice score

| instance | pushes | P\* | points |
|----------|--------|-----|--------|
| i001 | 13 | 13 | 1,000 |
| i002 | 16 | 16 | 1,000 |
| i003 | 29 | 29 | 1,000 |
| i004 | 47 | 47 | 1,000 |
| i005 | 60 | 60 | 1,000 |

Total practice score: **1,000,000** (house baseline plain-astar 1.0.0
scores 200,000 on the same practice set). Every instance is solved in
exactly its push-optimal count.

## License

MIT — see [LICENSE](LICENSE).
