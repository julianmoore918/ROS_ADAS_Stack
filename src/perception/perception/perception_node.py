#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Path
import cv2
import numpy as np
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
from sensor_msgs.msg import CompressedImage


# ========================
# CONFIG FLAGS
# ========================
USE_ROI             = False   # ← flip to True to enable static-polygon ROI mask
# Filter YOLO detections to those whose ground-projected (X, Y) lies
# between UFLD's ego-left and ego-right Paths in the vehicle frame.
# Re-uses the same IPM the lane_detection_node uses to project pixels to
# the road plane — no new topics, no second projection model. Falls back
# to the legacy "within 20% of image centre" rule when either lane Path
# is empty (UFLD warm-up or the bridge has paused inference inside a
# junction zone).
USE_LANE_ROI        = True
MIN_CONFIDENCE      = 0.1     # detections below this are ignored
PUBLISH_INF         = True    # publish inf when no vehicle detected. Set False to publish nothing on no-detection
ENABLE_LOGGING      = False  # set True to enable terminal output


class YoloDetection(Node):
    def __init__(self):
        super().__init__('Perception_Node', namespace='ACC')
        self.get_logger().info("=== Perception Node starting ===")
        self.get_logger().info(f"    ROI mask:   {'ON' if USE_ROI else 'OFF'}")
        self.get_logger().info(f"    Min conf:   {MIN_CONFIDENCE}")
        self.get_logger().info(f"    Publish inf: {PUBLISH_INF}")

        # Subscribers
        self.create_subscription(
            CompressedImage,
            '/Car_1/camera/front/compressed',
            self.listener_callback,
            10
        )
        # UFLD lanes in the vehicle frame (REP 103: X forward, Y left).
        # Used by the lane-ROI filter — see _in_ego_lane. Each list holds
        # (X, Y) sorted by ascending X so interp_y_at_x can binary-search.
        # No header timestamp matching: YOLO and UFLD run within ~50 ms
        # of each other and the lane geometry doesn't move that fast.
        self.left_veh:  list[tuple[float, float]] = []
        self.right_veh: list[tuple[float, float]] = []
        self.create_subscription(
            Path, '/LKAS/ego_lane_left',
            lambda m: self._on_lane('left', m), 10)
        self.create_subscription(
            Path, '/LKAS/ego_lane_right',
            lambda m: self._on_lane('right', m), 10)

        # Publishers
        self.dist_pub  = self.create_publisher(Float32,          '/ACC/lead_vehicle_distance',   10)
        self.debug_pub = self.create_publisher(CompressedImage,  '/ACC/perception/debug_image',  10)

        # Load YOLO model
        pkg_share = get_package_share_directory('perception')
        model_path = os.path.join(pkg_share, 'models', 'best.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded from {model_path}")

        self.last_log_time = 0.0

        # Camera FOV — bridge spawns the front camera at 90° by default
        # (carlaAccSimTown.py: --cam-fov, default 90). FOCAL_LENGTH is
        # derived from the actual image width per frame in
        # `estimateLeadDist` so the distance formula stays correct when
        # the bridge runs at 720p / 1080p / anything else. Parameterise
        # the FOV here in case the bridge ever ships a different camera.
        self.declare_parameter('camera_fov_deg', 90.0)
        self.CAM_FOV_RAD = math.radians(
            self.get_parameter('camera_fov_deg').value)
        # Camera extrinsics — must match the rig spawned by the bridge
        # and used by lane_detection_node's IPM (same 1.35 m / 0.6 m
        # defaults). Used by _pixel_to_vehicle so the lane-ROI filter
        # ground-projects detections into the same vehicle frame as
        # UFLD's /LKAS/ego_lane_* Paths.
        self.declare_parameter('cam_height_m', 1.35)
        self.declare_parameter('cam_x_offset', 0.6)
        self.cam_h_m   = self.get_parameter('cam_height_m').value
        self.cam_x_off = self.get_parameter('cam_x_offset').value

        self.OBJECT_HEIGHTS = {
            "car":        1.5,
            "truck":      3.0,
            "bus":        3.2,
            "motorcycle": 1.2,
        }

    # ========================
    # LANE ROI from UFLD (vehicle-frame IPM)
    # ========================
    @staticmethod
    def _path_to_veh(msg: Path) -> list[tuple[float, float]]:
        """Convert a nav_msgs/Path to a list of (X_forward, Y_left)
        sorted by ascending X. lane_detection_node already publishes in
        REP 103, so no axis flipping needed here."""
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        pts.sort(key=lambda xy: xy[0])
        return pts

    def _on_lane(self, side: str, msg: Path):
        veh = self._path_to_veh(msg)
        if side == 'left':
            self.left_veh = veh
        else:
            self.right_veh = veh

    def _pixel_to_vehicle(self, u: float, v: float, img_w: int, img_h: int):
        """Inverse-perspective-project an image pixel onto the road
        plane. Returns (X_forward, Y_left) in metres, or None for
        pixels at or above the horizon. Mirrors
        lane_detection_node.ipm_pixel_to_vehicle so detections and lane
        polylines live in the same coordinate system."""
        focal_px = img_w / (2.0 * math.tan(self.CAM_FOV_RAD / 2.0))
        cx, cy = img_w / 2.0, img_h / 2.0
        dv = v - cy
        if dv <= 1e-3:
            return None
        forward = self.cam_x_off + self.cam_h_m * focal_px / dv
        y_right = self.cam_h_m * (u - cx) / dv
        return forward, -y_right   # flip to ROS / REP 103 (Y left positive)

    @staticmethod
    def _interp_y_at_x(polyline: list[tuple[float, float]], x_target: float):
        """Linear-interpolate Y at X along a polyline that's sorted by X.
        Returns None outside the polyline's X range — caller decides
        whether to treat that as "lane unknown here"."""
        if len(polyline) < 2:
            return None
        if x_target < polyline[0][0] or x_target > polyline[-1][0]:
            return None
        for (x0, y0), (x1, y1) in zip(polyline, polyline[1:]):
            if x0 <= x_target <= x1:
                if x1 == x0:
                    return y0
                t = (x_target - x0) / (x1 - x0)
                return y0 + t * (y1 - y0)
        return None

    def _in_ego_lane(self, x_fwd: float, y_left: float) -> bool | None:
        """True iff (x_fwd, y_left) lies between the left and right
        UFLD lanes at this X. None means "can't tell" — either lane is
        empty, or X falls outside both polylines' overlap. Caller
        should fall back to the legacy centre-strip filter in that case."""
        left_y  = self._interp_y_at_x(self.left_veh,  x_fwd)
        right_y = self._interp_y_at_x(self.right_veh, x_fwd)
        if left_y is None or right_y is None:
            return None
        # REP 103: Y positive = left. Left lane → larger Y, right lane →
        # smaller (possibly negative) Y. Inside the lane means Y is
        # bounded by the two.
        lo, hi = sorted((left_y, right_y))
        return lo <= y_left <= hi

    # ========================
    # STATIC ROI MASK (optional, legacy)
    # ========================
    def apply_roi(self, img):
        h, w, _ = img.shape
        polygon = np.array([[
            (int(w * 0.1), int(h * 0.75)),
            (int(w * 0.4), int(h * 0.40)),
            (int(w * 0.6), int(h * 0.40)),
            (int(w * 0.9), int(h * 0.75)),
        ]])
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, polygon, 255)
        return cv2.bitwise_and(img, img, mask=mask)

    # ========================
    # CALLBACK
    # ========================
    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_img is None:
            return

        # Optionally apply ROI
        img_proc = self.apply_roi(cv_img) if USE_ROI else cv_img.copy()

        # Run YOLO
        results = self.model(img_proc, verbose=False)
        boxes   = results[0].boxes.xyxy.cpu().numpy()
        confs   = results[0].boxes.conf.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy()

        self.estimateLeadDist(img_proc, boxes, confs, classes, msg.header)

    # ========================
    # DISTANCE ESTIMATION
    # ========================
    def estimateLeadDist(self, img, boxes, confs, classes, header):
        min_distance = None

        h, w, _ = img.shape
        img_center_x = w / 2
        # Focal length in pixels = image_width / (2 * tan(FOV/2)). Computed
        # per frame so the formula adapts to whatever camera resolution
        # the bridge is publishing (was hard-coded at 640 for 1280-wide).
        focal_px = w / (2.0 * math.tan(self.CAM_FOV_RAD / 2.0))

        for box, conf, cls in zip(boxes, confs, classes):
            x_min, y_min, x_max, y_max = box.astype(int)
            class_name = self.model.names[int(cls)]

            if class_name not in self.OBJECT_HEIGHTS:
                continue

            if conf < MIN_CONFIDENCE:
                continue

            # Where the rear of the vehicle meets the road — the
            # natural anchor for "is this vehicle in my lane". IPM
            # turns that ground pixel into (X_forward, Y_left) in the
            # same vehicle frame UFLD uses, so we just check whether Y
            # is between the left and right lanes at that X.
            box_center_x = (x_min + x_max) / 2
            box_bottom_y = y_max
            if USE_LANE_ROI:
                ground = self._pixel_to_vehicle(
                    box_center_x, box_bottom_y, w, h)
                in_lane = (self._in_ego_lane(*ground)
                           if ground is not None else None)
                if in_lane is False:
                    continue
                if in_lane is None and \
                        abs(box_center_x - img_center_x) > w * 0.2:
                    # UFLD hasn't published a usable lane at this X
                    # (warm-up, junction pause, or detection beyond the
                    # lane polyline's range) — fall back to the legacy
                    # centre-strip filter so ACC isn't blind.
                    continue
            elif abs(box_center_x - img_center_x) > w * 0.2:
                continue

            box_height = y_max - y_min
            if box_height <= 0:
                continue

            real_height  = self.OBJECT_HEIGHTS[class_name]
            distance_m   = (focal_px * real_height) / box_height

            if min_distance is None or distance_m < min_distance:
                min_distance = distance_m

            # Draw bounding box on debug image
            label = f"{class_name} {conf:.2f}  {distance_m:.1f}m"
            cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(img, label, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # ── Distance publish ──────────────────────────────────
        if min_distance is not None:
            msg = Float32()
            msg.data = float(min_distance)
            self.dist_pub.publish(msg)
            if ENABLE_LOGGING:
                self._log_throttled(f"Closest vehicle: {min_distance:.2f} m  |  conf: {best_conf:.2f}")
        elif PUBLISH_INF:
            msg = Float32()
            msg.data = float('inf')
            self.dist_pub.publish(msg)

        # ── Debug image → Foxglove ────────────────────────────
        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        debug_msg = CompressedImage()
        debug_msg.header         = header   # keep original timestamp
        debug_msg.format         = 'jpeg'
        debug_msg.data           = buf.tobytes()
        self.debug_pub.publish(debug_msg)

    # ========================
    # THROTTLED LOGGING
    # ========================
    def _log_throttled(self, text):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            print(f"\r[Perception] {text}          ", end='', flush=True)
            self.last_log_time = now


# ========================
# MAIN
# ========================
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetection()  # or YoloDetection()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass  # clean shutdown, no traceback
    finally:
        node.destroy_node()
        if rclpy.ok():          # ← only shutdown if not already shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()