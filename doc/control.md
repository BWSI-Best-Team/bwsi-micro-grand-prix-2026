# Control

Two controllers handle motion: Pure Pursuit for general path following, and a three-stage PID stopper for the precision stop before the revolving door.

## Pure Pursuit (`src/control/path_tracker.py`)

Pure Pursuit is a geometric path tracking algorithm. Each frame it:

1. Finds the nearest point on the path to the car's current position
2. Looks ahead a variable distance along the path to pick a target point
3. Steers toward that target using Ackermann geometry: `angle = arctan(2L sin(α) / lookahead)`

The lookahead distance scales between 0.4 m and 1.5 m depending on how much curvature is coming up, tighter curves use a shorter lookahead so the car reacts sooner. Speed also scales down through curves: the tracker looks `CURVE_HORIZON_M = 1.5 m` ahead and reduces speed proportionally to the curvature it finds there.

Physical parameters: wheelbase = 0.32 m, max steer = ±0.35 rad (~20°).

## Stopper (`src/control/stopper.py`)

Used in Phase 1 to stop precisely 20 cm in front of the gate entry point. It runs in three stages as the car closes in:

**Stage 1 - Distance PID:** When the car is far from the target, the error is simply the remaining distance. The output drives speed down as the car approaches.

**Stage 2 - Speed PID:** Switches to velocity control, a feedforward term plus feedback on measured speed. This prevents overshoot from momentum.

**Stage 3 - Fine-tune PID:** Kicks in within 35 cm. Switches to a very slow creep to the final 20 cm stop point, using a tighter integral term to eliminate steady-state error.

Max speed allowed: 3.0 m/s.

## PID (`src/util/pid.py`)

Standard PID with configurable output limits and integral clamping to prevent windup:

```python
pid = PID(kp, ki, kd, output_min, output_max, integral_min, integral_max)
output = pid.update(error, dt)
```

Used by the stopper (three separate PID instances for the three stages) and available for any other control loop that needs it.

## Simulator speed hack

In simulation, the physics engine doesn't match hardware 1:1. We multiply speed commands by `SIM_HACK_MAX_SPEED = 1.8` via a direct packet to the simulator to compensate. This is disabled on hardware by setting `ENABLE_SIM_SPEED_HACK = False` in `constants.py`.
