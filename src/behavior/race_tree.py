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

def is_finished(ctx):
    return ctx.tracker.finished

def wall_too_close(ctx):
    return _min_front_cm(ctx) < WALL_STOP_CM

def obstacle_ahead(ctx):
    return _min_front_cm(ctx) < OBSTACLE_SLOW_CM

def approaching_gate(ctx):
    d = _dist_to_gate(ctx)
    return GATE_ZONE_RADIUS_M < d < GATE_APPROACH_RADIUS_M and ctx.phase == 1

def reached_gate(ctx):
    return _dist_to_gate(ctx) < GATE_ZONE_RADIUS_M and ctx.phase == 1

def waiting_for_gate_go(ctx):
    return ctx.phase == 2 and not ctx.gate_go_pressed

def passing_gate(ctx):
    return ctx.phase == 2 and ctx.gate_go_pressed


# actions


def stop_at_gate(ctx):
    dist = _dist_to_gate(ctx)

    if not getattr(ctx, '_gate_active', False):
        ctx._gate_active = True
        ctx._gate_locked = False
        ctx._gate_stop_frames = 0
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

    if dist < GATE_STOP_M:
        ctx.speed = 0.0
        ctx.angle = 0.0
        ctx._gate_stop_frames += 1
        # wait for 15 frames to let car completely stop
        if ctx._gate_stop_frames > 15:
            ctx._gate_locked = True
            print(f"[BT] stopped at gate (dist={dist:.2f}m)")
    else:
        ctx.speed = min(0.2, (dist - GATE_STOP_M) * 0.15)
        ctx.angle = max(-0.3, min(0.3, ctx.angle))

    return Status.SUCCESS

def wait_for_button(ctx):
    # check button A to pass gate
    rc = ctx.rc
    if rc and rc.controller.was_pressed(rc.controller.Button.A):
        ctx.gate_go_pressed = True
        print("[BT] GO! Passing gate at full throttle")
    ctx.speed = 0.0
    ctx.angle = 0.0
    return Status.SUCCESS

def pass_gate(ctx):
    if _detect_reset(ctx):
        _reset_all(ctx)
        return Status.FAILURE
    _pure_pursuit(ctx)
    ctx.speed = 1.0 # full speed when passing through the gate
    dist = _dist_to_gate(ctx)
    if dist > GATE_EXIT_DIST_M:
        ctx.phase = 3
        print("[BT] Phase 3: normal driving to finish")
    return Status.SUCCESS

def emergency_stop(ctx):
    ctx.speed = 0.0
    ctx.angle = 0.0
    return Status.SUCCESS

def _detect_reset(ctx):
    return getattr(ctx, '_reset_detected', False)

def _reset_all(ctx):
    ctx.phase = 1
    ctx._gate_active = False
    ctx._gate_locked = False
    ctx._gate_stop_frames = 0
    ctx.gate_go_pressed = False
    ctx._approach_printed = False
    print("[BT] sim reset detected, back to Phase 1")

def follow_path(ctx):
    if _detect_reset(ctx):
        _reset_all(ctx)
        return Status.FAILURE
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
        Sequence(Condition(is_finished), Action(emergency_stop)),
        Sequence(Condition(wall_too_close), Action(emergency_stop)),
        # phase 1: approach + stop at gate
        Sequence(Condition(reached_gate), Action(stop_at_gate)),
        Sequence(Condition(approaching_gate), Action(slow_for_gate_approach)),
        # phase 2: wait for button press and then go
        Sequence(Condition(waiting_for_gate_go), Action(wait_for_button)),
        Sequence(Condition(passing_gate), Action(pass_gate)),
        # phase 3 + drive to final
        Sequence(Condition(obstacle_ahead), Action(slow_for_obstacle)),
        Action(follow_path),
    )
