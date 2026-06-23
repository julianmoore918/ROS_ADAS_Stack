#!/usr/bin/env python3
"""
ADAS IPM Bird's-Eye View (ROS 2 Humble)
========================================
Warps the live CARLA front-camera frame to a top-down, metric view of
the road plane using a fixed homography derived from the camera's
intrinsics + extrinsics (the same numbers lane_detection_node uses for
its IPM). Overlays the UFLD ego-left / ego-right polylines on top so
the operator can see UFLD's prediction laid on the actual road texture.

Why warp the camera (vs. drawing on a blank canvas)?
----------------------------------------------------
The blank-canvas version showed *what UFLD believes the lanes are* but
nothing about *what the road actually looks like*. If UFLD draws a
straight lane on a curving road, the blank-canvas view can't tell you
that — both look like straight lines. The warped camera shows the real
asphalt + lane markings; if UFLD's blue/green dots don't follow the
warped lane paint, you can see it immediately. Doubles as a sanity
check on the camera extrinsics (1.35 m / 0.6 m) — get them wrong and
the warped lanes diverge from the painted ones with distance.

Subscribed topics:
    /Car_1/camera/front/compressed   (sensor_msgs/CompressedImage)
    /LKAS/ego_lane_left              (nav_msgs/Path)
    /LKAS/ego_lane_right             (nav_msgs/Path)

Published topics:
    /ADAS/ipm/debug_image            (sensor_msgs/CompressedImage)
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage


# ── BEV image / scale parameters ────────────────────────────────────────
IMG_W            = 320      # pixels wide
IMG_H            = 480      # pixels tall
PX_PER_M         = 15.0     # 1 m on the ground = 15 px in the image
EGO_BOTTOM_PAD   = 20       # pixels of breathing room below the ego
ORIGIN_X         = IMG_W // 2
ORIGIN_Y         = IMG_H - EGO_BOTTOM_PAD
FORWARD_MAX_M    = ORIGIN_Y / PX_PER_M                # ≈ 30.7 m forward
LATERAL_MAX_M    = (IMG_W / 2) / PX_PER_M             # ≈ ±10.7 m lateral

# ── Camera intrinsics / extrinsics ──────────────────────────────────────
# Must match the rig spawned by the bridge and used by
# lane_detection_node. If you change them in one place, change them
# everywhere — they're the contract for the IPM.
CAM_FOV_DEG      = 90.0
CAM_H_M          = 1.35
CAM_X_OFF        = 0.6
CAM_FOV_RAD      = math.radians(CAM_FOV_DEG)

# ── Ground control points for the homography (vehicle frame) ────────────
# Trapezoid in front of the ego: near-row + far-row, ±3 m laterally.
# Near row is 5 m forward (camera can see it cleanly without bumper
# clipping); far row is 25 m which is the comfortable IPM trust limit
# (beyond that the ground-plane assumption + image-pixel quantisation
# degrade the warp). Keep these points symmetric about Y=0.
HOM_POINTS_GROUND = [
    ( 5.0,  3.0),  # near-left
    ( 5.0, -3.0),  # near-right
    (25.0,  3.0),  # far-left
    (25.0, -3.0),  # far-right
]

# ── Render rate ─────────────────────────────────────────────────────────
PUB_HZ           = 10.0


def veh_to_bev(x_fwd: float, y_left: float):
    """(X_forward, Y_left) [m] → (u, v) [px] in the BEV image.
    Y_left positive (left of ego) maps to smaller u (image-left), so the
    rendering matches what you see looking out of the windshield.
    Returns None for points outside the BEV image."""
    u = int(round(ORIGIN_X - y_left * PX_PER_M))
    v = int(round(ORIGIN_Y - x_fwd * PX_PER_M))
    if 0 <= u < IMG_W and 0 <= v < IMG_H:
        return u, v
    return None


def veh_to_bev_unclipped(x_fwd: float, y_left: float):
    """Same as veh_to_bev but doesn't gate on image bounds — used for
    homography control points so the warp is well-defined even if a
    control point lies right at the image edge."""
    u = ORIGIN_X - y_left * PX_PER_M
    v = ORIGIN_Y - x_fwd * PX_PER_M
    return [float(u), float(v)]


def compute_homography(img_w: int, img_h: int) -> np.ndarray:
    """Return the 3×3 perspective transform that warps the forward
    camera image (img_w × img_h) onto the BEV canvas. Computed once per
    camera resolution change."""
    focal_px = img_w / (2.0 * math.tan(CAM_FOV_RAD / 2.0))
    cx, cy = img_w / 2.0, img_h / 2.0

    def ground_to_img(x_fwd: float, y_left: float):
        # Inverse of lane_detection_node.ipm_pixel_to_vehicle: project a
        # point on the road plane forward into the camera. Mirrors the
        # same camera model so the warp lines up with UFLD's lanes.
        dx = x_fwd - CAM_X_OFF
        u = cx + (-y_left) * focal_px / dx
        v = cy + CAM_H_M * focal_px / dx
        return [float(u), float(v)]

    src = np.float32([ground_to_img(*p) for p in HOM_POINTS_GROUND])
    dst = np.float32([veh_to_bev_unclipped(*p) for p in HOM_POINTS_GROUND])
    return cv2.getPerspectiveTransform(src, dst)


class IPMViewNode(Node):
    def __init__(self):
        super().__init__('IPM_View_Node', namespace='ADAS')
        self.left_veh:  list[tuple[float, float]] = []
        self.right_veh: list[tuple[float, float]] = []
        self.latest_bgr: np.ndarray | None = None
        self.H: np.ndarray | None = None      # homography, lazy
        self.H_for_shape: tuple[int, int] | None = None  # (w, h) of H

        self.create_subscription(CompressedImage,
                                 '/Car_1/camera/front/compressed',
                                 self._on_camera, 10)
        self.create_subscription(Path, '/LKAS/ego_lane_left',
                                 self._on_left,  10)
        self.create_subscription(Path, '/LKAS/ego_lane_right',
                                 self._on_right, 10)
        self.debug_pub = self.create_publisher(
            CompressedImage, 'ipm/debug_image', 10)
        self.create_timer(1.0 / PUB_HZ, self._publish)

        self.get_logger().info(
            f"IPM view node started — warped camera + lane overlay on "
            f"/ADAS/ipm/debug_image @ {PUB_HZ:.0f} Hz "
            f"({FORWARD_MAX_M:.0f} m forward × ±{LATERAL_MAX_M:.0f} m lateral)")

    # ── Subscriptions ───────────────────────────────────────────────────
    def _on_camera(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        self.latest_bgr = bgr
        # (Re-)compute the homography on first frame or when the camera
        # resolution changes (e.g., the bridge swap between 720p/1080p).
        shape = (bgr.shape[1], bgr.shape[0])
        if self.H is None or shape != self.H_for_shape:
            self.H = compute_homography(*shape)
            self.H_for_shape = shape
            self.get_logger().info(
                f"Homography computed for {shape[0]}×{shape[1]} camera")

    def _on_left(self, msg: Path):
        self.left_veh = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_right(self, msg: Path):
        self.right_veh = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses]

    # ── Rendering ───────────────────────────────────────────────────────
    def _warp_camera(self) -> np.ndarray:
        """Warped camera if we have one, else a near-black background.
        Either way the result is IMG_W × IMG_H × 3."""
        if self.latest_bgr is None or self.H is None:
            return np.full((IMG_H, IMG_W, 3), 28, dtype=np.uint8)
        warped = cv2.warpPerspective(
            self.latest_bgr, self.H, (IMG_W, IMG_H),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(28, 28, 28))
        # Slight dimming so the bright-coloured overlays stand out
        # against the warped asphalt. Multiplicative — preserves the
        # texture's structure for sanity-checking the warp.
        return (warped * 0.7).astype(np.uint8)

    def _draw_lane(self, img: np.ndarray,
                   pts: list[tuple[float, float]],
                   colour: tuple[int, int, int]):
        prev = None
        for (x, y) in pts:
            pt = veh_to_bev(x, y)
            if pt is None:
                prev = None
                continue
            cv2.circle(img, pt, 3, colour, -1)
            if prev is not None:
                cv2.line(img, prev, pt, colour, 1, cv2.LINE_AA)
            prev = pt

    def _draw_distance_ticks(self, img: np.ndarray):
        """Just forward-distance labels every 5 m on the left edge.
        Skipping the full grid — would clutter the warped road texture."""
        for x_m in range(5, int(FORWARD_MAX_M) + 1, 5):
            v = ORIGIN_Y - int(round(x_m * PX_PER_M))
            if 0 <= v < IMG_H:
                cv2.line(img, (0, v), (8, v), (200, 200, 200), 1)
                cv2.putText(img, f'{x_m}m', (10, v + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                            (220, 220, 220), 1, cv2.LINE_AA)

    def _draw_ego(self, img: np.ndarray):
        # Small red wedge at the bottom — origin of the vehicle frame.
        cv2.rectangle(img,
                      (ORIGIN_X - 5, ORIGIN_Y - 8),
                      (ORIGIN_X + 5, ORIGIN_Y + 2),
                      (0, 0, 220), -1)
        cv2.line(img,
                 (ORIGIN_X, ORIGIN_Y - 8),
                 (ORIGIN_X, ORIGIN_Y - 14),
                 (0, 0, 220), 1)

    def _render(self) -> np.ndarray:
        img = self._warp_camera()
        self._draw_distance_ticks(img)
        # Match the LKAS perception-debug colour convention: left blue,
        # right green.
        self._draw_lane(img, self.left_veh,  (255, 80, 80))
        self._draw_lane(img, self.right_veh, (80, 255, 80))
        self._draw_ego(img)
        cv2.putText(img, "IPM bird's-eye (vehicle frame)",
                    (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (240, 240, 240), 1, cv2.LINE_AA)
        if self.latest_bgr is None:
            cv2.putText(img, 'waiting for camera…',
                        (60, IMG_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (180, 180, 180), 1, cv2.LINE_AA)
        return img

    def _publish(self):
        img = self._render()
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IPMViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
