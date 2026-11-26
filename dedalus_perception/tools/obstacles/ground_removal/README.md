# Obstacles · ground removal

**Owners:** Angelica (Patchwork++), Giulia (RANSAC baseline)

Input: a 3D LiDAR scan (`lidar/*.npy`, N×4). Output: ground points and non-ground points.

- **Patchwork++** (main method): region-wise ground fitting with adaptive parameters, so no fixed threshold
  for the whole scene. Paper: Lee, Lim, Myung, IROS 2022 — https://arxiv.org/abs/2207.11919 ·
  code: https://github.com/url-kaist/patchwork-plusplus
- **RANSAC plane** (Open3D `segment_plane`): one plane for the whole scan, used as a baseline for the comparison.

Comparison on the same frames: correct / wrong ground points (truth from CARLA's semantic LiDAR), time per frame,
behaviour on slopes and with many cars.

To add here: the code of each method.
