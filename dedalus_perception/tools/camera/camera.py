"""
camera.py - Shared camera geometry for the DEDALUS Perception team.

One place for the camera numbers, so that the lane (Alessandro, Angelica) and
the obstacles (Jamileth, Giulia) use exactly the same values.

Frames used here
----------------
* base_link (vehicle frame, ROS convention, as in the aea_dedalus README):
  origin on the ground under the centre of the REAR AXLE, x forward, y LEFT, z up.
  Distances we send to Control use this.
  The camera mount (1.5 m ahead, 1.5 m high) is given from the vehicle CENTRE,
  as in CARLA; the code converts it to base_link.
* camera optical frame (OpenCV convention): x right, y down, z forward
  (along the optical axis).
* image: u = column (0 = left), v = row (0 = top), in pixels.

Note: CARLA itself uses x forward, y RIGHT, z up. The values below are already
converted to base_link (y left).

What is inside
--------------
* CameraModel: holds K (intrinsics) and the camera pose on the car.
* vehicle_to_pixel(...)   : 3D point in base_link  -> pixel (u, v)
* pixel_depth_to_vehicle(...): pixel + depth (metres) -> 3D point in base_link
* ground_homography(...)  : homography road plane (z = 0) -> image
* birds_eye_view(...)     : bird's-eye image of the road, in metres

Usage
-----
    from camera import CameraModel
    cam = CameraModel()                       # default values of our recording
    cam = CameraModel.from_json("dati/train_s02/camera_info.json")   # if present
"""

import json
import math

import numpy as np

# ---------------- DEFAULT VALUES (recording dedalus_semplice/01_registra.py) -------------
WIDTH, HEIGHT = 960, 540        # image size in pixels
FOV_DEG = 90.0                  # horizontal field of view (CARLA-only setup, decided by the team)

# Camera position in CARLA, relative to the vehicle CENTRE (CARLA actor origin, on the ground)
CAM_X_FROM_CENTER = 1.5         # metres forward
CAM_Y_FROM_CENTER = 0.0         # metres to the left
CAM_Z = 1.5                     # metres above the ground (Tesla Model 3 setup agreed with Oguz)
CAM_PITCH_DEG = -8.0            # CARLA pitch: negative = looking down

# Where the base_link origin (rear axle) is, relative to the vehicle centre (metres, negative = behind).
# ESTIMATE for the Tesla Model 3 (wheelbase ~2.9 m). The exact value is measured by 01_registra.py
# and saved in camera_info.json ("base_link_from_center"): use CameraModel.from_json(...).
BASE_LINK_FROM_CENTER = -1.4
# -------------------------------------------------------------------------------------------


class CameraModel:
    def __init__(self, width=WIDTH, height=HEIGHT, fov_deg=FOV_DEG,
                 cam_x_from_center=CAM_X_FROM_CENTER, cam_y_from_center=CAM_Y_FROM_CENTER,
                 cam_z=CAM_Z, cam_pitch_deg=CAM_PITCH_DEG,
                 base_link_from_center=BASE_LINK_FROM_CENTER):
        self.width = width
        self.height = height
        self.fov_deg = fov_deg

        # ---- Intrinsics K: same formula as in 02_riconosci.py ----
        f = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))   # 480 px with FOV 90 and width 960
        self.K = np.array([[f, 0.0, width / 2.0],
                           [0.0, f, height / 2.0],
                           [0.0, 0.0, 1.0]])

        # ---- Camera position in base_link ----
        self.t = np.array([cam_x_from_center - base_link_from_center,   # ~1.5 + 1.4 = 2.9 m
                           cam_y_from_center,
                           cam_z])

        # ---- Camera orientation: columns = camera axes written in base_link ----
        # pitch_down > 0 means the camera looks down
        pitch_down = math.radians(-cam_pitch_deg)
        s, c = math.sin(pitch_down), math.cos(pitch_down)
        x_cam = np.array([0.0, -1.0, 0.0])      # camera "right" = vehicle -y
        z_cam = np.array([c, 0.0, -s])          # optical axis: forward and a bit down
        y_cam = np.cross(z_cam, x_cam)          # camera "down"
        self.R = np.column_stack([x_cam, y_cam, z_cam])   # camera -> base_link

    @classmethod
    def from_json(cls, path):
        """Reads the values saved by the recording script (camera_info.json)."""
        with open(path) as f:
            d = json.load(f)
        # keep only the camera numbers; other fields (vehicle, notes...) are just information
        names = ("width", "height", "fov_deg", "cam_x_from_center", "cam_y_from_center",
                 "cam_z", "cam_pitch_deg", "base_link_from_center")
        valori = {k: d[k] for k in names if k in d}
        # older files: take the rear axle from the measured vehicle data, if present
        if "base_link_from_center" not in valori and "rear_axle_from_center" in d.get("vehicle", {}):
            valori["base_link_from_center"] = d["vehicle"]["rear_axle_from_center"]
        return cls(**valori)

    # ------------------------------------------------------------------------------
    def vehicle_to_pixel(self, points):
        """3D points in base_link (N x 3, metres) -> pixels (N x 2) and depth (N,).
        Points behind the camera get depth <= 0: ignore them."""
        points = np.atleast_2d(np.asarray(points, dtype=float))
        p_cam = (points - self.t) @ self.R            # base_link -> camera
        depth = p_cam[:, 2]
        uv = (p_cam @ self.K.T)[:, :2] / depth[:, None]
        return uv, depth

    def pixel_depth_to_vehicle(self, u, v, depth):
        """Pixel (u, v) + depth in metres -> 3D point in base_link.
        Works with single numbers or whole arrays (e.g. a depth image).
        CARLA depth = distance along the optical axis (that's what we save)."""
        u, v, depth = np.asarray(u, float), np.asarray(v, float), np.asarray(depth, float)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        p_cam = np.stack([(u - cx) / fx * depth, (v - cy) / fy * depth, depth], axis=-1)
        return p_cam @ self.R.T + self.t              # camera -> base_link

    # ------------------------------------------------------------------------------
    def ground_homography(self):
        """3x3 matrix H: road point (X, Y, 1) in metres (z = 0) -> image pixel (u, v, 1).
        No points picked by hand: it comes from K and the camera pose.
        Assumes a FLAT road."""
        M = np.array([[1.0, 0.0, -self.t[0]],
                      [0.0, 1.0, -self.t[1]],
                      [0.0, 0.0, -self.t[2]]])        # (X, Y, 1) -> point minus camera position
        return self.K @ self.R.T @ M

    def birds_eye_view(self, image, x_range=(5.0, 35.0), y_half=6.0, resolution=0.05):
        """Bird's-eye view of the road (flat-road assumption).
        x_range: metres ahead of the REAR AXLE (from, to); y_half: metres left and right;
        resolution: metres per pixel. Returns (bev_image, A), where A maps
        BEV pixel (col, row, 1) -> road point (X, Y, 1) in metres."""
        import cv2
        x_min, x_max = x_range
        cols = int(round(2 * y_half / resolution))
        rows = int(round((x_max - x_min) / resolution))
        # BEV pixel -> metres: left of the car at col 0, far away at row 0
        A = np.array([[0.0, -resolution, x_max],
                      [-resolution, 0.0, y_half],
                      [0.0, 0.0, 1.0]])
        bev_to_image = self.ground_homography() @ A
        bev = cv2.warpPerspective(image, bev_to_image, (cols, rows),
                                  flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        return bev, A


if __name__ == "__main__":
    cam = CameraModel()
    np.set_printoptions(precision=2, suppress=True)
    print("K =\n", cam.K)
    print("Camera position in base_link (m):", cam.t)
    # A road point 10 m in front of the camera, on the car's centre line:
    p = np.array([[cam.t[0] + 10.0, 0.0, 0.0]])
    uv, d = cam.vehicle_to_pixel(p)
    print("Road point 10 m ahead of the camera ->", uv[0], "px")
    back = cam.pixel_depth_to_vehicle(uv[0, 0], uv[0, 1], d[0])
    print("...and back to base_link:", back)
