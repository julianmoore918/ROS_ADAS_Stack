#!/usr/bin/env python3
"""
LKAS Lane Detection Node (ROS2 Humble)
======================================
Runs UFLD V2 inference on the CARLA front camera feed and publishes the
ego-left / ego-right lane polylines in the vehicle frame (REP 103: X forward,
Y left, Z up).

Ported from `02_UFLD_V2/lkas_validate_0.10.0.py` — the `UFLDInference`,
`ipm_pixel_to_vehicle`, and `polyline_to_vehicle` helpers are lifted unchanged.
Junction handling and the Stanley controller live in `stanley_node`.

Subscribed topics:
    /Car_1/camera/front/compressed   (sensor_msgs/CompressedImage)
    /Car_1/in_junction               (std_msgs/Bool)
        Published by the bridge's junction monitor. While True, UFLD
        inference is skipped — junction geometry (curbs, crosswalk
        markings) is exactly where UFLD is least trustworthy, and the
        bridge's pure-pursuit / hold-straight policies own steer
        anyway. Empty Paths are emitted so Stanley enters HOLD and
        stops fighting PP on cmd_steer.

Published topics:
    /LKAS/ego_lane_left              (nav_msgs/Path)
    /LKAS/ego_lane_right             (nav_msgs/Path)
        Vehicle-frame polylines (REP 103: X forward, Y left). Consumed
        by stanley_node (lateral controller) and perception_node (ACC
        lane-ROI filter — same IPM, same coordinate frame, no second
        projection model).
    /LKAS/perception/debug_image     (sensor_msgs/CompressedImage)
"""

import math
import os
os.environ.setdefault('OMP_NUM_THREADS',      '2')
os.environ.setdefault('MKL_NUM_THREADS',      '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '2')
os.environ.setdefault('NUMEXPR_NUM_THREADS',  '2')

import sys
import importlib

import cv2
cv2.setNumThreads(2)
import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32

import torch
torch.set_num_threads(2)
torch.set_num_interop_threads(2)



# Camera intrinsics / extrinsics — defaults match the validate-script CARLA rig.
DEFAULT_CAM_FOV_DEG   = 90.0
DEFAULT_CAM_HEIGHT_M  = 1.35
DEFAULT_CAM_X_OFFSET  = 0.6   # camera mounted 0.6 m forward of vehicle origin

# UFLD prediction is trusted only if this fraction of row anchors is "valid".
EXIST_MIN_RATIO = 0.25


class UFLDInference:
    """Loads a UFLD V2 model and produces ego-left / ego-right polylines
    (image-space pixel coordinates) for a given BGR frame.

    Mirrors the class in `lkas_validate_0.10.0.py` so the controller behaviour
    is unchanged when ported into ROS."""

    def __init__(self, config_path: str, model_path: str, ufld_repo: str,
                 device: str = 'cuda'):
        if ufld_repo not in sys.path:
            sys.path.insert(0, ufld_repo)
        from utils.config import Config

        import torch
        import torchvision.transforms as transforms
        self._torch = torch

        self.cfg = Config.fromfile(config_path)
        self.cfg.batch_size = 1
        self.cfg.row_anchor = np.linspace(0.42, 1.0, self.cfg.num_row)
        self.cfg.col_anchor = np.linspace(0.0,  1.0, self.cfg.num_col)
        self.device = device

        net = importlib.import_module(
            'model.model_' + self.cfg.dataset.lower()
        ).get_model(self.cfg)
        state = torch.load(model_path, map_location=device)['model']
        compatible = {k[7:] if k.startswith('module.') else k: v
                      for k, v in state.items()}
        net.load_state_dict(compatible, strict=False)
        net.eval().to(device)
        self.net = net

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406),
                                 (0.229, 0.224, 0.225)),
        ])
        self.resize_w = self.cfg.train_width
        self.resize_h = int(self.cfg.train_height / self.cfg.crop_ratio)
        self.crop_h = self.cfg.train_height
        self.row_anchor = np.asarray(self.cfg.row_anchor)

    def preprocess(self, bgr: np.ndarray):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(rgb, (self.resize_w, self.resize_h),
                         interpolation=cv2.INTER_LINEAR)
        img = img[-self.crop_h:, :, :]
        return self.transform(img).unsqueeze(0).to(self.device)

    def __call__(self, bgr: np.ndarray, img_w: int, img_h: int):
        """Returns (ego_left_polyline, ego_right_polyline) where each polyline
        is a list of (x, y) ints in the original image coordinates. Empty list
        means lane not detected."""
        with self._torch.no_grad():
            tensor = self.preprocess(bgr)
            pred = self.net(tensor)

        loc_row = pred['loc_row'].cpu()
        exist_row = pred['exist_row'].cpu()
        num_grid_row = loc_row.shape[1]
        num_cls_row = loc_row.shape[2]

        # ── NEW: soft confidences from the same logits (before argmax discards them)
        exist_soft = self._torch.softmax(exist_row, dim=1)[:, 1]   # (1, num_row, num_lanes)
        loc_soft   = self._torch.softmax(loc_row,   dim=1)         # (1, C, num_row, num_lanes)
        pos_peak   = loc_soft.max(dim=1).values                     # (1, num_row, num_lanes)

        def lane_conf(lane_idx: int) -> float:
            mask = exist_soft[0, :, lane_idx] > 0.5
            if mask.sum() == 0:
                return 0.0
            return float((exist_soft[0, mask, lane_idx]
                        * pos_peak[0, mask, lane_idx]).mean())

        left_conf  = lane_conf(1)   # ego_left
        right_conf = lane_conf(2)   # ego_right

        max_idx = loc_row.argmax(1)
        valid = exist_row.argmax(1)

        polylines = {}
        for lane_idx, key in [(1, 'ego_left'), (2, 'ego_right')]:
            pts = []
            if valid[0, :, lane_idx].sum() < num_cls_row * EXIST_MIN_RATIO:
                polylines[key] = []
                continue
            for k in range(num_cls_row):
                if not valid[0, k, lane_idx]:
                    continue
                center = int(max_idx[0, k, lane_idx])
                lo = max(0, center - 1)
                hi = min(num_grid_row - 1, center + 1) + 1
                window = loc_row[0, lo:hi, k, lane_idx]
                inds = self._torch.arange(lo, hi, dtype=self._torch.float32)
                x_cell = (window.softmax(0) * inds).sum().item() + 0.5
                x = x_cell / (num_grid_row - 1) * img_w
                y = self.row_anchor[k] * img_h
                pts.append((int(round(x)), int(round(y))))
            polylines[key] = pts
        return polylines['ego_left'], polylines['ego_right'], left_conf, right_conf

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('Lane_Detection_Node', namespace='LKAS')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('ufld_repo',
            '/home/sirius/workspace/01_CV_Models/01_Ultra_Fast_Lane_Detection_V2/Ultra-Fast-Lane-Detection-V2')
        self.declare_parameter('ufld_config_rel', 'configs/culane_res34.py')
        self.declare_parameter('model_filename', 'UFLD_best.pth')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('camera_topic', '/Car_1/camera/front/compressed')
        self.declare_parameter('frame_id', 'base_link')

        self.declare_parameter('cam_fov_deg',  DEFAULT_CAM_FOV_DEG)
        self.declare_parameter('cam_height_m', DEFAULT_CAM_HEIGHT_M)
        self.declare_parameter('cam_x_offset', DEFAULT_CAM_X_OFFSET)

        # Run UFLD on every Nth camera frame. Camera publishes at ~20 Hz;
        # N=4 gives ~5 Hz inference, which is plenty at the 20 km/h target.
        self.declare_parameter('inference_skip_n', 4)

        ufld_repo   = self.get_parameter('ufld_repo').value
        cfg_rel     = self.get_parameter('ufld_config_rel').value
        model_name  = self.get_parameter('model_filename').value
        device      = self.get_parameter('device').value
        cam_topic   = self.get_parameter('camera_topic').value
        self.frame_id   = self.get_parameter('frame_id').value
        self.cam_fov    = math.radians(self.get_parameter('cam_fov_deg').value)
        self.cam_h_m    = self.get_parameter('cam_height_m').value
        self.cam_x_off  = self.get_parameter('cam_x_offset').value
        self.skip_n     = max(1, int(self.get_parameter('inference_skip_n').value))

        # ── Resolve weights ─────────────────────────────────────────────
        # `model_filename` accepts either a bare filename (resolved
        # against `share/perception/models/`) or an absolute path (used
        # by the UI dropdown when the operator wants to point at a
        # freshly-trained checkpoint outside the package).
        pkg_share = get_package_share_directory('perception')
        if os.path.isabs(model_name):
            model_path = model_name
        else:
            model_path = os.path.join(pkg_share, 'models', model_name)
        cfg_path = os.path.join(ufld_repo, cfg_rel)

        self.get_logger().info("=== Lane Detection Node starting ===")
        self.get_logger().info(f"    UFLD repo:   {ufld_repo}")
        self.get_logger().info(f"    Config:      {cfg_path}")
        self.get_logger().info(f"    Weights:     {model_path}")
        self.get_logger().info(f"    Device:      {device}")
        self.get_logger().info(f"    Camera:      {cam_topic}")

        self.get_logger().info("Loading UFLD V2 (this can take ~10-30s for the 1.7 GB state dict)…")
        self.infer = UFLDInference(cfg_path, model_path, ufld_repo, device)
        self.get_logger().info("UFLD loaded — waiting for first camera frame")

        # ── I/O ──────────────────────────────────────────────────────────
        self.create_subscription(CompressedImage, cam_topic,
                                 self.camera_callback, 10)
        # Bridge-published junction state. While True we skip UFLD
        # inference and emit empty Paths — see module docstring.
        self.create_subscription(Bool, '/Car_1/in_junction',
                                 self._in_junction_cb, 1)
        self._in_junction = False
        self.left_pub  = self.create_publisher(Path, 'ego_lane_left',  10)
        self.right_pub = self.create_publisher(Path, 'ego_lane_right', 10)
        self.debug_pub = self.create_publisher(CompressedImage,
                                               'perception/debug_image', 10)
        # NEW — per-lane soft confidence (YOLO-analogue). Range [0, 1].
        # 0.0 during junctions and when the lane isn't detected.
        self.left_conf_pub  = self.create_publisher(Float32, 'ego_lane_left_conf',  10)
        self.right_conf_pub = self.create_publisher(Float32, 'ego_lane_right_conf', 10)

        # Warn periodically if no frames are arriving on the camera topic.
        self.cam_topic = cam_topic
        self.frame_count = 0
        self.create_timer(5.0, self._heartbeat)

    def _in_junction_cb(self, msg: Bool):
        was = self._in_junction
        self._in_junction = bool(msg.data)
        if self._in_junction != was:
            state = 'ENTER' if self._in_junction else 'EXIT'
            self.get_logger().info(
                f'[junction] {state} — UFLD inference '
                f'{"paused" if self._in_junction else "resumed"}')

    # ─────────────────────────────────────────────────────────────────────
    # Geometry: pixel → vehicle frame (REP 103 — Y is LEFT positive)
    # ─────────────────────────────────────────────────────────────────────
    def ipm_pixel_to_vehicle(self, u, v, img_w, img_h):
        """Returns (X_forward, Y_left) in metres, or None if above the
        horizon. Internally uses the camera-frame "Y right positive"
        convention from the validate script and negates at the end so the
        output matches REP 103."""
        focal = img_w / (2.0 * math.tan(self.cam_fov / 2.0))
        cx, cy = img_w / 2.0, img_h / 2.0
        dv = v - cy
        if dv <= 1e-3:
            return None
        forward = self.cam_x_off + self.cam_h_m * focal / dv
        y_right = self.cam_h_m * (u - cx) / dv
        return forward, -y_right   # flip to ROS convention

    def polyline_to_vehicle(self, polyline_px, img_w, img_h):
        out = []
        for u, v in polyline_px:
            p = self.ipm_pixel_to_vehicle(u, v, img_w, img_h)
            if p is not None:
                out.append(p)
        return out

    # ─────────────────────────────────────────────────────────────────────
    # Publishing
    # ─────────────────────────────────────────────────────────────────────
    def to_path(self, polyline_veh, header):
        path = Path()
        path.header = header
        path.header.frame_id = self.frame_id
        for x_fwd, y_left in polyline_veh:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x_fwd)
            pose.pose.position.y = float(y_left)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def annotate(self, bgr, left_px, right_px, left_conf=0.0, right_conf=0.0):
        out = bgr.copy()
        for u, v in left_px:
            cv2.circle(out, (u, v), 4, (255, 80, 80), -1)
        for u, v in right_px:
            cv2.circle(out, (u, v), 4, (80, 255, 80), -1)

        # Confidence overlay — top-left corner. Colour tiered so a glance
        # tells you if the lane is trusted: green ≥ 0.7, yellow 0.3-0.7,
        # red < 0.3. Two rows: L (ego-left, blue), R (ego-right, green).
        def _conf_colour(c: float):
            if c >= 0.7: return (60, 220, 60)     # green
            if c >= 0.3: return (60, 220, 220)    # yellow
            return (60, 60, 220)                  # red
        cv2.putText(out, f'L: {left_conf:.2f}',  (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    _conf_colour(left_conf), 2, cv2.LINE_AA)
        cv2.putText(out, f'R: {right_conf:.2f}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    _conf_colour(right_conf), 2, cv2.LINE_AA)
        return out

    # ─────────────────────────────────────────────────────────────────────
    # Callback
    # ─────────────────────────────────────────────────────────────────────
    def _heartbeat(self):
        if self.frame_count == 0:
            self.get_logger().warn(
                f'No camera frames yet on {self.cam_topic} — is CARLA + the ROS '
                f'bridge running? Check with: ros2 topic hz {self.cam_topic}'
            )

    def camera_callback(self, msg: CompressedImage):
        self.frame_count += 1
        if (self.frame_count - 1) % self.skip_n != 0:
            return
        arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        if self.frame_count == 1:
            self.get_logger().info(
                f'First camera frame received ({bgr.shape[1]}x{bgr.shape[0]}) — '
                f'inference active (running every {self.skip_n} frames)'
            )
        img_h, img_w = bgr.shape[:2]

        # In a junction zone, emit empty Paths and the raw camera frame
        # (with a "JUNCTION" overlay) instead of running UFLD. Stanley
        # interprets empty Paths as HOLD and stops publishing cmd_steer,
        # so the bridge's PP / hold-straight policies own steer cleanly.
        if self._in_junction:
            empty_header = msg.header
            empty_header.frame_id = self.frame_id
            self.left_pub.publish(self.to_path([], empty_header))
            self.right_pub.publish(self.to_path([], empty_header))
            # NEW — zero confidence during junctions
            zero = Float32(); zero.data = 0.0
            self.left_conf_pub.publish(zero)
            self.right_conf_pub.publish(zero)
            overlay = bgr.copy()
            cv2.putText(overlay, 'JUNCTION (UFLD paused)', (10, 20),
                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.9, (0, 0, 0), 2)
            _, buf = cv2.imencode('.jpg', overlay,
                                  [cv2.IMWRITE_JPEG_QUALITY, 85])
            dbg = CompressedImage()
            dbg.header = msg.header
            dbg.format = 'jpeg'
            dbg.data = buf.tobytes()
            self.debug_pub.publish(dbg)
            return

        left_px, right_px, left_conf, right_conf = self.infer(bgr, img_w, img_h)
        left_veh  = self.polyline_to_vehicle(left_px,  img_w, img_h)
        right_veh = self.polyline_to_vehicle(right_px, img_w, img_h)

        self.left_pub.publish(self.to_path(left_veh,   msg.header))
        self.right_pub.publish(self.to_path(right_veh, msg.header))

        # NEW — publish confidence alongside polylines
        lc = Float32(); lc.data = float(left_conf);  self.left_conf_pub.publish(lc)
        rc = Float32(); rc.data = float(right_conf); self.right_conf_pub.publish(rc)

        annotated = self.annotate(bgr, left_px, right_px,
                                  left_conf, right_conf)
        _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        dbg = CompressedImage()
        dbg.header = msg.header
        dbg.format = 'jpeg'
        dbg.data = buf.tobytes()
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
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
