# Lane · baseline (classical pipeline)

**Owner:** Alessandro

```
Camera RGB → IPM (BEV) → colour filters (HLS white, LAB yellow) + Sobel-X → sliding window → RANSAC parabola → e_y, e_ψ, κ
```

To add here: the `lane_detection/` package (`perspective.py`, `evaluation.py`, ...).
Documents (PROJECTION, `planar_homography_design_decisions.md`, LaneDetectionPipeline) go in `../../../docs/`.

**Note:** with the current camera (FOV 90°, 1.5 m high, pitch −8°) the BEV numbers change:
bottom row v = 539 → 3.47 m, row v = 275 → 11.42 m from the vehicle centre (add ~1.4 m for base_link at the rear axle), horizon v = 202.5 px.
Read the camera values from `camera_info.json`.

Evaluation: same protocol as `../unet/` (masks at 960×540, same test frames, IoU / precision / recall / F1, ms per frame),
plus the error on `e_y` and `e_ψ` against the CARLA truth in `info.csv`.
