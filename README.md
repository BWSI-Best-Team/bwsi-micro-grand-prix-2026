# BWSI 2026 Micro Grand Prix

Autonomous racing controller for the BWSI 2026 Micro Grand Prix Pre-Competition. The car navigates a complex indoor track in three phases: approach a spinning revolving door, time the pass-through, then sprint to the finish.

## Competition Calendar

| Date | Event |
| :--- | :--- |
| 3/21 – 4/1 | Pre-Dev: Algorithm development |
| **3/30 (Mon)** | **BWSI Application Deadline**: Submit by 8:59 PM PT |
| **4/1 (Wed)** | **Registration Deadline**: [Intent to Compete Form](https://forms.gle/B98we9rZGydtAgRQ8) due 11:59 PM PT |
| 4/1 (Wed) | **Map Release**: New competition map released |
| 4/2 – 4/8 | Development Week: Test and tuning |
| **4/3 (Fri)** | **Teacher Recommendation Deadline**: Due by 8:59 PM PT |
| 4/9 – 4/11 | Code Adjudication: Team reviews code for violations |
| **4/12 (Sun)** | **Finals**: 12:00 PM – 1:00 PM PT via Zoom |

---

## Track Map

![Track Map](images/track_map.png)

| Feature | Color in map | Description |
| :--- | :--- | :--- |
| **Start line** | Checkered (left side) | Two positions A (x=17.68, y=24.80) and B (x=17.68, y=26.80), auto-detected via orange color + depth camera (orange closer than 225 cm → start B, farther → start A) |
| **Orange gates** | Orange vertical lines | Two barriers that physically move up and down in the corridor; this controller uses a static map with them baked in as fixed walls, no dynamic detection of gate position |
| **Green zone** | Green lines | Trigger area near the finish; when camera detects green and depth < 5 m, car slows to 0.20 for ~1 second |
| **Revolving door** | Yellow X (right side) | 4-blade spinning door at (x=31.85, y=25.76); car times entry when blades reach ~44.5° |
| **Finish line** | Checkered (left side) | End goal at (x=14.08, y=23.82) |

**Map data files:**
| File | Description |
| :--- | :--- |
| `data/track_map.npy` | Binary occupancy grid for the global planner; includes purple walls and orange gates baked in, excludes green gates, and has a manually added wall between the start and finish lines |
| `data/track_map.json` | Grid metadata: resolution (1 cm/cell), world bounds |
| `data/map_features.json` | Door position, gate locations, wall definitions |
| `data/track_map_inference2.npy` | Clean map for ICP (no dynamic objects) |

---

## Usage

```bash
git clone https://github.com/BWSI-Best-Team/bwsi-micro-grand-prix-2026
cd bwsi-micro-grand-prix-2026

# Run in simulator
racecar sim src/main.py

# Manual drive mode (for testing sensors)
racecar sim src/test.py
```

**Configuration** (`controller_config.json`): only two keys are active, rest is legacy:
```json
{
  "print_log": true,
  "show_visualizer": false
}
```

---

## Architecture

Inspired by ROS2 Nav2. The system follows a sense → localize → plan → decide → act pipeline.

```mermaid
flowchart LR
    A["<b>1. Sense</b>: InputManager<br/>color_image, depth_image<br/>lidar_scan (720 beams, 360°)<br/>imu (accel, gyro)"]
    B["<b>2. Localize</b>: ICP + EKF<br/>ICP aligns lidar to map<br/>EKF fuses motion + ICP<br/>Fallback: IMU dead-reckoning"]
    C["<b>3. Perceive</b>: parallel<br/>DoorTracker → blade angle<br/>ColorDetector → name, px, area<br/>DepthDetector → distance"]
    D["<b>4. Decide</b>: behavior tree (race_tree.py)<br/>Context: phase, pose, velocity, sensors<br/>Priority fallback (first match wins):<br/>1. Sim reset → re-initialize<br/>2. Path finished → stop<br/>3. Wall &lt; 50 cm → emergency stop<br/>4. Phase 1 in gate → stopper PID<br/>5. Phase 1 approaching → ramp down<br/>6. Phase 2 waiting → watch door<br/>7. Phase 2 passing → full speed<br/>8. Phase 3 green → slow to 0.2<br/>9. Obstacle ahead → adaptive slowdown<br/>10. Default → pure pursuit"]
    E["<b>5. Act</b><br/>drive.set_speed_angle(speed, angle)"]
    A --> B --> C --> D --> E
    E -. loop .-> A
```

### Startup (`start()`)

- Load config from `controller_config.json`
- Load track map + features
- Initialize ICP localizer (bake door blades into wall map)
- Detect start position A or B via orange color
- Plan global path: Dijkstra → resample → Savitzky-Golay smooth
- Open run log (`tmp/run_log.csv`)