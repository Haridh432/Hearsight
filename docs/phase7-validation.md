# Phase 7 — Validation walks

Measured walk tests with pass / fail rules. Each walk is recorded by the app's test mode
and scored on the PC; nothing is judged by eye alone.

## How to run a test

1. Phone on the head mount, D455 connected, voice on.
2. Tap the **✓ test button** (bottom left, next to the language button) and pick a scenario.
   The camera starts and an amber banner shows **TEST S1 · …**.
3. Do the walk as described below.
4. Tap the banner (or the ⏹ test button) to finish. One CSV file per walk is saved on the phone.
5. On the PC (phone connected by USB or wireless debugging):

   ```bash
   python3 tools/validate_walks.py --pull --out validation_runs
   ```

   This pulls every walk, scores it and writes `validation_runs/validation_report.md`.

Repeat each scenario **3 times**; a scenario passes when all 3 walks pass.

**Start clear** (S1 wall, S5 person, S9 chair): begin with nothing within 1.5 m in front for at
least 1 s. The scorer only counts the approach after such a clear second; a walk that never had
one is reported **INVALID** (restart facing open space) rather than FAIL.

## Checks applied to every walk

| Check | Rule |
|---|---|
| Safety invariant | Never a walking instruction (go straight / bear / turn / door) unless the safety validator allows movement |
| Frame rate | ≥ 20 fps average |

## Scenarios

| ID | Scenario | How | Pass when |
|---|---|---|---|
| S1 | Wall approach | Start 3 m from a wall, walk slowly straight at it, stop when told, stand 5 s, step back | First STOP at 0.3–0.5 m from the wall; STOP spoken within 0.6 s; no straight-on advice with anything ≤ 0.7 m ahead (a detour is allowed) |
| S2 | Clear corridor | Walk 10 m along a clear corridor at normal pace | STOP ≤ 10 % of the time; walking advice allowed ≥ 50 %; ≤ 15 instruction changes / min; no camera faults |
| S3 | Doorway | Start 3 m before an open door, walk through | STOP ≤ 3 s in total; "No clear way" ≤ 30 % of the time; walking advice allowed ≥ 40 % |
| S4 | Corner turn | Walk to a corridor corner and follow it round | A turn / bear instruction given; ≤ 20 changes / min; STOP ≤ 20 % |
| S5 | Person approaching | Stand still; a helper walks towards you from 4 m | CAUTION / STOP for the person (moving or ahead); STOP before 0.3 m |
| S6 | Step down / stairs | **With a helper**: walk slowly towards the top of a step or stairs | STOP for the drop / stairs at ≥ 0.8 m |
| S7 | Camera covered | While walking, cover the camera for 3 s, then uncover | Camera STOP within 1.0 s; "camera blocked" spoken within 1.5 s; recovers after uncovering |
| S8 | Standing still | Stand still 30 s facing an open space | No STOP; ≤ 6 spoken messages per 30 s |
| S9 | Chair in path | A chair in the middle of a corridor; walk towards it | STOP at 0.3–0.5 m or a detour (bear / turn) instruction; no straight-on advice with the chair ≤ 0.7 m ahead |

## What the CSV contains

`T` rows (~10 per second): verdict (GO / CAUTION / HALT), the check that decided it, whether
walking advice is allowed, distances (object ahead, obstacle, head height, drop-off), the
instruction shown, camera health and depth coverage, the plan (ok / blocked / clearance),
walking speed. `S` rows: every spoken message. `E` rows: start / end / camera unplugged.

## Known limits

- Distances are the app's own depth measurements, not tape-measured ground truth. For S1, S6
  and S9, mark the floor at 0.5 m / 0.8 m to confirm by eye that the STOP came in time.
- The camera mount tilt (12–25°) is not fixed yet; false STOP / drop-off results in S2–S4 and
  S8 may come from it rather than the software.
