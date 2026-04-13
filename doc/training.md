# ML Training (offline)

These tools are for training and evaluating an ONNX-based pose estimator from lidar scans. This is separate from the main race controller, as it runs offline before competition.

## Data Collection (`src/data_collection/collect_lidar_dataset.py`)

Randomly teleports the car to 100 valid positions in the arena, waits 4 frames for the physics to settle, captures the lidar scan, and records the ground truth pose. Output is saved as `.npz` files with a metadata CSV.

Valid positions are sampled by BFS from a seed point on the free-space map, with a 3 m clearance from walls. This ensures the car spawns somewhere physically reachable rather than inside a wall.

Settings: 100 samples, random seed 13371337, cell size 2.0 m.

## Model (`src/training/train_lidar_locator.py`)

A 1D CNN that maps a single lidar scan to an (x, z, yaw) pose estimate.

```
Input:  (batch, 1, 720)  - normalized lidar beams [0, 1]
Conv1d  1→32,  kernel=7  → MaxPool
Conv1d  32→64, kernel=5  → MaxPool
Conv1d  64→128,kernel=5  → MaxPool
Conv1d  128→128,kernel=3 → AdaptiveAvgPool
Linear: 2048 → 256 → 128 → 4
Output: [x_norm, z_norm, sin(yaw), cos(yaw)]
```

Yaw is encoded as (sin, cos) so the network doesn't have to deal with angle wraparound. The x and z outputs are normalized by the training set mean/std.

Training uses AdamW (lr=1e-3), batch size 256, 20 epochs, 15% validation split, GPU if available.

## Inference (`src/perception/ml_estimator.py`)

Loads the trained model as an ONNX file via `onnxruntime`, with GPU preferred and CPU fallback. The ONNX runtime itself is vendored in `vendor/` to avoid dependency issues.

In the race controller this is available as an optional localization seed or cross-check, though the primary localization relies on ICP + EKF.
