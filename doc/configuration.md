# Configuration

Two files control how the controller runs: `controller_config.json` for runtime toggles and `src/util/constants.py` for all tunable race parameters.

## Runtime config (`controller_config.json`)

Only two keys are active. Any other key raises an error on startup.

| Key | Type | Default | Effect |
| :--- | :--- | :--- | :--- |
| `print_log` | bool | `true` | Print per-frame debug info to stdout |
| `show_visualizer` | bool | `false` | Open the live map visualizer window |

```json
{
  "print_log": true,
  "show_visualizer": false
}
```

The file is optional. If missing, defaults above apply. The loader is `src/controller_config.py`.

## Tunable constants (`src/util/constants.py`)

### Waypoints (meters)

| Constant | Value | Description |
| :--- | :--- | :--- |
| `START_XY_A` | `(17.68, 24.80)` | Start position A |
| `START_XY_B` | `(17.68, 26.80)` | Start position B |
| `GATE_ENTER_XY` | `(28.50, 25.00)` | Stop point before the revolving door |
| `GATE_EXIT_XY` | `(29.93, 21.32)` | Waypoint past the door |
| `GOAL_XY` | `(14.08, 23.82)` | Finish line |
| `DOOR_CENTER_XY` | `(31.85, 25.76)` | Center of the revolving door |

### Behavior tree thresholds

| Constant | Value | Description |
| :--- | :--- | :--- |
| `WALL_STOP_CM` | `50` | Emergency stop if wall is closer than this |
| `OBSTACLE_SLOW_CM` | `150` | Start adaptive slowdown below this distance |
| `GATE_APPROACH_RADIUS_M` | `10.0` | Begin speed reduction this far from gate |
| `GATE_ZONE_RADIUS_M` | `3.0` | Enter stopper PID creep mode |
| `GATE_STOP_M` | `0.3` | Stop this far from the gate entry point |
| `GATE_EXIT_REACHED_M` | `1.0` | Threshold to transition from Phase 2 to Phase 3 |
| `DOOR_GO_ANGLE_DEG` | `44.5` | Blade angle at which to rush through the door |
| `DOOR_GO_TOLERANCE_DEG` | `1.5` | Tolerance around `DOOR_GO_ANGLE_DEG` |
| `DOOR_STABLE_FRAMES` | `2` | Frames the angle must hold before going |
| `START_RESET_RADIUS_M` | `2.0` | Proximity to start that triggers reset detection |

### Path tracker

| Constant | Value | Description |
| :--- | :--- | :--- |
| `CRUISE_SPEED` | `1.0` | Default speed on straights |
| `CURVE_SLOWDOWN` | `0.5` | Minimum speed multiplier through curves |
| `CURVE_HORIZON_M` | `1.5` | Look-ahead distance for curvature detection |

### Stopper PID

| Constant | Value | Description |
| :--- | :--- | :--- |
| `STOPPER_TARGET_CM` | `20` | Final stop distance from gate entry point |
| `STOPPER_FINE_TUNE_M` | `0.35` | Switch to fine-tune stage within this distance |
| `STOPPER_MAX_SPEED_MS` | `3.0` | Speed cap during stopper stages |

### Costmap inflation

| Constant | Value | Description |
| :--- | :--- | :--- |
| `INFLATE_M` | `0.45` | Default wall inflation margin |
| `GATE_INFLATE_M` | `0.25` | Tighter margin for the gate corridor segment |
| `PHASE3_INFLATE_M` | `0.55` | Wider margin after the door |
| `CORNER_EXTRA_M` | `0.35` | Extra inflation added at sharp corners |
| `PHASE3_CORNER_EXTRA_M` | `0.55` | Corner inflation for Phase 3 |

### Path smoothing

| Constant | Value | Description |
| :--- | :--- | :--- |
| `SAVGOL_WINDOW_M` | `8.0` | Savitzky-Golay window length |
| `SMOOTH_PASSES` | `2` | Number of smoothing passes |

### ICP localization

| Key | Value | Description |
| :--- | :--- | :--- |
| `map_resolution` | `0.01` | Map resolution (m/cell) |
| `icp_max_iter` | `5` | Maximum ICP iterations per frame |
| `icp_converge_thresh` | `1e-4` | Convergence threshold |
| `icp_initial_sigma` | `0.5` | Initial uncertainty for EKF |
| `lost_fitness_thresh` | `0.15` | ICP fitness below this triggers "lost" state |
| `lost_frame_count` | `15` | Frames below threshold before declaring lost |

### Simulator speed hack

| Constant | Value | Description |
| :--- | :--- | :--- |
| `ENABLE_SIM_SPEED_HACK` | `True` | Multiply speed commands in simulation |
| `SIM_HACK_MAX_SPEED` | `1.8` | Multiplier applied when hack is enabled |

Set `ENABLE_SIM_SPEED_HACK = False` when running on hardware.
