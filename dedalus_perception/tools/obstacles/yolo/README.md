# Obstacles · YOLO

**Owner:** Jamileth

- `DEDALUS_demo_Kaggle.ipynb`: YOLO on recorded CARLA frames + distance from the depth image.
- `DEDALUS_video_Kaggle.ipynb`: YOLO on screen-recorded CARLA videos.

Next steps:
1. Distance from the LiDAR points inside each box (after ground removal and clustering).
2. Fusion: YOLO class + LiDAR position and size → `/perception/objects` for Control.
3. Tracking (stable ID, speed, heading).
4. Fine-tuning YOLO on CARLA labels (pretrained confidence is often low; COCO has no "pole" class).
