# Behavior Tree

Decision-making is handled by a behavior tree (`src/behavior/`). This keeps the race logic organized as a priority-ordered set of conditions and actions, rather than a tangle of if/else statements.

## Framework (`src/behavior/tree.py`)

Each node in the tree returns one of three statuses: `SUCCESS`, `FAILURE`, or `RUNNING`.

- **Sequence**: runs children left to right; stops and returns `FAILURE` if any child fails. Think of it as AND.
- **Fallback**: runs children left to right; stops and returns `SUCCESS` or `RUNNING` on the first non-failure. Think of it as OR / priority selector.
- **Condition**: wraps a boolean function; returns `SUCCESS` or `FAILURE`.
- **Action**: wraps a function that returns a `Status` directly.

## Race Context

All nodes share a `RaceContext` object that holds the current state of the world: pose, velocity, lidar scan, door angle, color detection results, current phase, and the output drive command. Nodes read from it and write their speed/angle output back to `ctx.speed` and `ctx.angle`.

## Race Logic (`src/behavior/race_tree.py`)

The tree is a single top-level Fallback that tries each condition in priority order. The first one that succeeds or is running wins the frame.

**Priority order:**

1. **Sim reset detected**: if the car teleported or crashed, re-initialize everything and replan
2. **Path finished**: stop
3. **Wall too close (< 50 cm)**: emergency stop
4. **Phase 1, in gate zone**: hand off to stopper PID to creep to the final stop point
5. **Phase 1, approaching gate**: ramp speed down smoothly as distance to gate decreases
6. **Phase 2, waiting**: watch door angle; stay stopped until blades reach ~44.5°
7. **Phase 2, passing**: rush through at full speed
8. **Phase 3, green detected within 5 m**: slow to 0.20 for 60 frames (~1 second)
9. **Obstacle ahead (< 150 cm)**: adaptive slowdown proportional to distance
10. **Default**: pure pursuit path following at cruise speed

## Phase transitions

- **Phase 1 → 2:** Stopper reaches the 20 cm stop target at the gate entry point
- **Phase 2 → 3:** Car reaches within 1 m of the gate exit waypoint after passing through
- **Any phase → 1 (reset):** Position jump > 5 m, IMU crash spike, out-of-bounds, or orange detected in camera during phases 2/3 (indicates teleport back to start)

## Reset detection

The system watches for several signals that mean the simulator reset or the car crashed:
- Position jumps more than 5 m between frames
- IMU acceleration magnitude exceeds 5.0 m/s² (crash)
- Car is outside the map bounds
- Orange color detected in camera during phase 2 or 3 (only the start area has orange)

On reset, the ICP localizer is re-initialized, the path is replanned from start A, and the phase returns to 1.
