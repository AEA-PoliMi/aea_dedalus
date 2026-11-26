# Obstacles · clustering

**Owner:** Giulia — _in progress_

Input: the non-ground LiDAR points from `../ground_removal/`.
Output: one group of points per object → 3D position, size, distance.

Idea: cluster nearby points (e.g. DBSCAN) with a merging distance that **grows with the range**
(far away the LiDAR points are sparser), so we avoid a single fixed threshold.

These objects are then matched with the YOLO boxes in `../yolo/` (class from YOLO, distance and size from the LiDAR).
