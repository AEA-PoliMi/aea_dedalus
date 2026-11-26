# Camera · recording and shared camera model

**Owner:** Jamileth

| File | What |
|---|---|
| `01_registra.py` | Records a session in CARLA: RGB, depth, segmentation, 3D LiDAR. Saves `camera_info.json` and `info.csv` (with the CARLA truth for the lane: `e_y`, `e_psi`). |
| `test_sensori.py` | Quick check that CARLA sends sensor data. |
| `04_vista_strada.py` | Puts the spectator at street level, for screen recordings. |
| `02_riconosci.py` | YOLO on a recorded session + distance from depth. |
| `03_demo_live.py` | Live demo in CARLA. |
| `camera.py` | Shared camera model: K, pose, pixel ↔ base_link, ground homography, bird's-eye view. |
| `camera_info.json` | Template; the real one is written in each session folder. |
| `LEGGIMI.md` | Step-by-step guide (Italian). |

```python
from camera import CameraModel
cam = CameraModel.from_json("data/train_s02/camera_info.json")   # base_link = rear axle
```
