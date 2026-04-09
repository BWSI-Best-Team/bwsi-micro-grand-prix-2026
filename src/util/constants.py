""" Project-wide tunable constants """

UNITY_TO_M = 0.10 # Unity unit (dm) to meters

# simulator speed override
ENABLE_SIM_SPEED_HACK = True
SIM_HACK_MAX_SPEED = 1.8

# waypoints in meters
START_XY_A = (17.68, 24.80)
START_XY_B = (17.68, 26.80)
START_XY = START_XY_A  # default, auto-detected at runtime
GATE_ENTER_XY = (28.50, 25.0) # stop point before spin door
GATE_EXIT_XY = (29.93, 21.32) # past gate
GOAL_XY = (14.08, 23.82) # finish line
DOOR_CENTER_XY = (31.85, 25.76) # yellow rotating door center

# reset detection
START_RESET_RADIUS_M = 5.0  # if car is within this radius of start -> sim reset

# behavior tree
WALL_STOP_CM = 50 # emergency stop distance
OBSTACLE_SLOW_CM = 150  # start slowing down
GATE_APPROACH_RADIUS_M = 10.0 # start reducing speed
GATE_ZONE_RADIUS_M = 1.5 # enter creep mode
GATE_STOP_M = 0.3 # stop this far from gate target
GATE_EXIT_DIST_M = 3.0 # past gate -> phase 3
DOOR_GO_ANGLE_DEG = 47.2  # rush when blades at this angle
DOOR_GO_TOLERANCE_DEG = 1.5  # +-tolerance
DOOR_STABLE_FRAMES = 2  # frames

# stopper PID
STOPPER_TARGET_CM = 20     # PID stop distance (cm)
STOPPER_MAX_SPEED_MS = 3.0

# path tracker
CRUISE_SPEED = 1.0
CURVE_SLOWDOWN = 0.5
CURVE_HORIZON_M = 1.5

# costmap
INFLATE_M = 0.45            # default wall inflation
GATE_INFLATE_M = 0.21       # tighter inflation for gate segment
PHASE3_INFLATE_M = 0.55     # phase 3 for extra 0.2 inflation for safety
CORNER_EXTRA_M = 0.25       # extra inflation at sharp corners

# smoothing
SAVGOL_WINDOW_M = 8.0       # smoothing window length
SMOOTH_PASSES = 2           # number of savgol passes

# ICP localization
ICP_CONFIG = {
    "map_resolution": 0.01,
    "icp_max_iter": 5,
    "icp_converge_thresh": 1e-4,
    "icp_initial_sigma": 0.5,
    "lost_fitness_thresh": 0.15,
    "lost_frame_count": 15,
}
