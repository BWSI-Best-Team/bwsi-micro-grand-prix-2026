# Perception

Perception runs every frame alongside localization. Three things are tracked: the revolving door angle, detected colors, and depth.

## Door Tracker (`src/perception/door_tracker.py`)

The revolving door has 4 blades arranged in a cross. To time our pass-through, we need to know the current angle of that cross from lidar data.

The algorithm:
1. Filter lidar points to only those within 1.5 m of the door center, excluding the hub (< 0.12 m) to avoid matching the center post
2. Try every possible cross orientation and score it by the average perpendicular distance of points to the nearest blade axis, the best angle minimizes this
3. Run three refinement passes: coarse (1° step) → fine (0.25°) → precise (0.05°)
4. Return the angle and a confidence score based on how many points were found and how well they fit

The confidence score lets the behavior tree decide whether to trust the reading before committing to a pass.

## Color Detector (`src/perception/color_detector.py`)

HSV-based detection that finds the largest contour above 1000 px² in the camera image. Colors used in the race:

- **Orange**: start position detection (at startup and reset detection in phases 2/3)
- **Green**: Phase 3 slowdown trigger

The detector returns the color name, the center pixel of the contour, and the contour area.

## Depth Detector (`src/perception/depth_detector.py`)

Thin wrapper around `racecar_utils` that reads the depth image. Returns the depth at the image center and the closest pixel overall. Used to get the distance to the detected green zone.

## Input Manager (`src/perception/input_manager.py`)

Collects all raw sensor data each frame into a single `InputState` object:
- `color_image`, `depth_image` from the camera
- `lidar_scan`: 720-beam array in cm
- `imu_accel_forward_mps2`, `imu_accel_right_mps2`, `imu_yaw_rate_rad_per_s` from the IMU

Everything else reads from this rather than calling sensor APIs directly.

## Start Position Detection

At startup (and after a reset), the car looks at the camera for an orange marker to determine whether it's at start position A or B. It detects the orange contour via HSV thresholding, then reads the depth at the contour center:

- Depth < 225 cm → start B (17.68, 26.80)
- Depth ≥ 225 cm → start A (17.68, 24.80)

If no orange is detected or the contour is too small, it defaults to start A.
