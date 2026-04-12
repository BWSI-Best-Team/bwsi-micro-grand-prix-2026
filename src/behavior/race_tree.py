"""
1. drive to spin door and using stopper to the gate
2. pass through spin door
3. drive to the finish line
"""
import math
import numpy as np
from .tree import Status, Sequence, Fallback, Condition, Action
from util.constants import *


class RaceContext:
    """Shared state passed through the tree each tick."""
    def __init__(self):
        self.tracker = None
        self.stopper = None # Stopper
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.lidar = None
        self.dt = 0.016
        self.v_fwd = 0.0 # forward velocity
        self.speed = 0.0
        self.angle = 0.0
        self.gate_enter_xy = None # stop point before gate
        self.phase = 1 # 1=to gate, 2=through gate, 3=to finish
        self.passed_gate = False
        self.rc = None
        self.gate_go_pressed = False
        self.door_angle_rad = None  # rotating door global angle
        self.green_detected = False
        self.depth_center_cm = 0.0


# helpers

def _min_front_cm(ctx, cone_deg=60):
    lidar = ctx.lidar
    if lidar is None:
        return 9999.0
    n = len(lidar)
    half = int(cone_deg / 360.0 * n / 2)
    front = np.concatenate([lidar[:half], lidar[-half:]])
    front = front[front > 0]
    return float(front.min()) if len(front) > 0 else 9999.0

def _dist_to_gate(ctx):
    if ctx.gate_enter_xy is None:
        return 9999.0
    return math.hypot(ctx.x - ctx.gate_enter_xy[0], ctx.y - ctx.gate_enter_xy[1])

def _pure_pursuit(ctx):
    cmd = ctx.tracker.update(ctx.x, ctx.y, ctx.yaw)
    ctx.speed = cmd.speed
    ctx.angle = -cmd.angle


# conditions

def sim_reset(ctx):
    # near start point + not in phase 1 = sim teleported car back
    return ctx.phase != 1 and _near_start(ctx)

def is_finished(ctx):
    return ctx.tracker.finished

def wall_too_close(ctx):
    if ctx.phase == 2:
        return False  # no emergency stop during gate pass
    close = _min_front_cm(ctx, cone_deg=30) < WALL_STOP_CM
    if not close:
        ctx._estop_printed = False
    return close

def obstacle_ahead(ctx):
    return _min_front_cm(ctx, cone_deg=40) < OBSTACLE_SLOW_CM

def approaching_gate(ctx):
    d = _dist_to_gate(ctx)
    return GATE_ZONE_RADIUS_M < d < GATE_APPROACH_RADIUS_M and ctx.phase == 1

def reached_gate(ctx):
    return _dist_to_gate(ctx) < GATE_ZONE_RADIUS_M and ctx.phase == 1

GREEN_SLOW_DIST_CM = 500 # 5m
GREEN_SLOW_FRAMES = 60

def green_nearby(ctx):
    return ctx.phase == 3 and ctx.green_detected and ctx.depth_center_cm < GREEN_SLOW_DIST_CM

def waiting_for_gate_go(ctx):
    return ctx.phase == 2 and not ctx.gate_go_pressed

def passing_gate(ctx):
    return ctx.phase == 2 and ctx.gate_go_pressed


# actions


def stop_at_gate(ctx):
    dist = _dist_to_gate(ctx)
    dist_cm = dist * 100.0

    if not getattr(ctx, '_gate_active', False):
        ctx._gate_active = True
        ctx._gate_locked = False
        ctx._gate_stop_frames = 0
        ctx.stopper.reset()
        print(f"[BT] gate zone, creeping to target (dist={dist:.2f}m)")

    # changed to phase 2 once stopped
    if ctx._gate_locked:
        ctx.speed = 0.0
        ctx.angle = 0.0
        if ctx.phase == 1:
            ctx.phase = 2
            print("[BT] Phase 2: waiting for button press to pass gate")
        return Status.SUCCESS

    _pure_pursuit(ctx)

    throttle = ctx.stopper.update(dist_cm, ctx.v_fwd, ctx.dt)
    ctx.speed = throttle
    ctx.angle = max(-0.3, min(0.3, ctx.angle))

    if ctx.stopper.stopped:
        ctx.speed = 0.0
        ctx.angle = 0.0
        ctx._gate_stop_frames += 1
        if ctx._gate_stop_frames > 15:
            ctx._gate_locked = True
            print(f"[BT] stopped at gate (dist={dist:.2f}m)")

    return Status.SUCCESS

from util.constants import DOOR_GO_ANGLE_DEG, DOOR_GO_TOLERANCE_DEG, DOOR_STABLE_FRAMES

def wait_for_gate(ctx):
    if not hasattr(ctx, '_door_stable_count'):
        ctx._door_stable_count = 0

    if ctx.door_angle_rad is not None:
        angle_deg = math.degrees(ctx.door_angle_rad)
        diff = abs(angle_deg - DOOR_GO_ANGLE_DEG)
        if diff < DOOR_GO_TOLERANCE_DEG:
            ctx._door_stable_count += 1
            if ctx._door_stable_count >= DOOR_STABLE_FRAMES:
                ctx.gate_go_pressed = True
                print(f"[BT] door at {angle_deg:.1f}° for {DOOR_STABLE_FRAMES} frames, GO!")
        else:
            ctx._door_stable_count = 0
    else:
        ctx._door_stable_count = 0

    # fallback: button A
    rc = ctx.rc
    if not ctx.gate_go_pressed and rc and rc.controller.was_pressed(rc.controller.Button.A):
        ctx.gate_go_pressed = True
        print("[BT] manual GO!")
    ctx.speed = 0.0
    ctx.angle = 0.0
    return Status.SUCCESS

def pass_gate(ctx):
    _pure_pursuit(ctx)
    exit_dist = math.hypot(ctx.x - GATE_EXIT_XY[0], ctx.y - GATE_EXIT_XY[1])
    ctx.speed = 1.0 # full speed when passing through the gate
    ctx.angle *= 1.0
    ctx.angle = max(-1.0, min(1.0, ctx.angle))
    if not getattr(ctx, '_pass_gate_printed', False):
        print(f"[BT] pass_gate ACTIVE, speed={ctx.speed}")
        ctx._pass_gate_printed = True
    if exit_dist < GATE_EXIT_REACHED_M:
        ctx.phase = 3
        print("[BT] Phase 3: normal driving to finish")
    return Status.SUCCESS

def do_reset(ctx):
    _reset_all(ctx)
    ctx._reset_detected = True
    ctx.speed = 0.0
    ctx.angle = 0.0
    return Status.SUCCESS

def slow_for_green(ctx):
    # slow to 0.5 for 1 second, then back to 1.0
    if not hasattr(ctx, '_green_slow_counter'):
        ctx._green_slow_counter = 0
    ctx._green_slow_counter += 1
    _pure_pursuit(ctx)
    if ctx._green_slow_counter <= GREEN_SLOW_FRAMES:
        ctx.speed *= 0.20
        if ctx._green_slow_counter == 1:
            print("[BT] green detected < 5m, slowing to 0.20")
    else:
        if ctx._green_slow_counter == GREEN_SLOW_FRAMES + 1:
            print("[BT] green slowdown done, back to 1.0")
    return Status.SUCCESS

def emergency_stop(ctx):
    if not getattr(ctx, '_estop_printed', False):
        front = _min_front_cm(ctx)
        print(f"[BT] emergency stop (front={front:.0f}cm)")
        ctx._estop_printed = True
    ctx.speed = 0.0
    ctx.angle = 0.0
    return Status.SUCCESS

def _near_start(ctx):
    return math.hypot(ctx.x - START_XY[0], ctx.y - START_XY[1]) < START_RESET_RADIUS_M

def _reset_all(ctx):
    ctx.phase = 1
    ctx._gate_active = False
    ctx._gate_locked = False
    ctx._gate_stop_frames = 0
    ctx.gate_go_pressed = False
    ctx._approach_printed = False
    print("[BT] sim reset detected, back to Phase 1")

def follow_path(ctx):
    if ctx.phase == 3:
        orig_slow = ctx.tracker._slow
        ctx.tracker._slow = 0.85
        _pure_pursuit(ctx)
        ctx.tracker._slow = orig_slow
        ctx.speed *= 0.6
        ctx.angle *= 1.5
        ctx.angle = max(-1.0, min(1.0, ctx.angle))
    else:
        _pure_pursuit(ctx)
    return Status.SUCCESS

def slow_for_gate_approach(ctx):
    d = _dist_to_gate(ctx)
    factor = max(0.0, (d - GATE_ZONE_RADIUS_M) / (GATE_APPROACH_RADIUS_M - GATE_ZONE_RADIUS_M))
    _pure_pursuit(ctx)
    before = ctx.speed
    ctx.speed *= factor
    if not getattr(ctx, '_approach_printed', False):
        print(f"[BT] approach zone: dist={d:.1f}m factor={factor:.2f} speed {before:.2f}->{ctx.speed:.2f}")
        ctx._approach_printed = True
    return Status.SUCCESS

def slow_for_obstacle(ctx):
    d = _min_front_cm(ctx)
    factor = max(0.0, (d - WALL_STOP_CM) / (OBSTACLE_SLOW_CM - WALL_STOP_CM))
    _pure_pursuit(ctx)
    ctx.speed *= factor
    return Status.SUCCESS


# build tree

def build_race_tree():
    """
    Fallback:
      1. finished? -> stop
      2. wall < 40cm? -> emergency stop
      3. near gate (phase 1)? -> Stopper PID cascade
      4. obstacle < 150cm? -> slow down
      5. normal -> follow path
    """
    return Fallback(
        # sim reset: car teleported back to start
        Sequence(Condition(sim_reset), Action(do_reset)),
        Sequence(Condition(is_finished), Action(emergency_stop)),
        Sequence(Condition(wall_too_close), Action(emergency_stop)),
        # phase 1: approach + stop at gate
        Sequence(Condition(reached_gate), Action(stop_at_gate)),
        Sequence(Condition(approaching_gate), Action(slow_for_gate_approach)),
        # phase 2: wait for button press and then go
        Sequence(Condition(waiting_for_gate_go), Action(wait_for_gate)),
        Sequence(Condition(passing_gate), Action(pass_gate)),
        # phase 3: green < 5m -> slow 0.5 for 1s
        Sequence(Condition(green_nearby), Action(slow_for_green)),
        # general
        Sequence(Condition(obstacle_ahead), Action(slow_for_obstacle)),
        Action(follow_path),
    )
