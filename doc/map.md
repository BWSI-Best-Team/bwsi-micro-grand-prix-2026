
## Track Map

![Track Map](../images/track_map.png)

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
