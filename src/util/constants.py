""" Project-wide tunable constants """

UNITY_TO_M = 0.10 # Unity unit (dm) to meters

# waypoints in meters
GATE_ENTER_XY = (28.80, 24.95) # stop point before spin door
GATE_EXIT_XY = (29.93, 21.32) # past gate
GOAL_XY = (14.08, 23.82) # finish line

# behavior tree
WALL_STOP_CM = 40 # emergency stop distance
OBSTACLE_SLOW_CM = 150  # start slowing down
GATE_APPROACH_RADIUS_M = 10.0 # start reducing speed
GATE_ZONE_RADIUS_M = 1.5 # enter creep mode
GATE_STOP_M = 0.5 # stop this far from gate target
GATE_EXIT_DIST_M = 3.0 # past gate -> phase 3

# stopper PID
STOPPER_TARGET_CM = 20     # PID stop distance (cm)
STOPPER_MAX_SPEED_MS = 3.0

# path tracker
CRUISE_SPEED = 1.0
CURVE_SLOWDOWN = 0.3
CURVE_HORIZON_M = 2.0
