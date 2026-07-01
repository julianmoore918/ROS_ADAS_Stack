#!/usr/bin/env python3
import os
os.environ.setdefault('OMP_NUM_THREADS',      '2')
os.environ.setdefault('MKL_NUM_THREADS',      '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '2')
os.environ.setdefault('NUMEXPR_NUM_THREADS',  '2')

import math
import cv2
cv2.setNumThreads(2)
import numpy as np
import torch
torch.set_num_threads(2)
torch.set_num_interop_threads(2)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory



# ========================
# CONFIG FLAGS
# ========================
# Filter YOLO detections to those whose ground-projected (X, Y) lies
# between UFLD's ego-left and ego-right Paths in the vehicle frame.
# Re-uses the same IPM the lane_detection_node uses to project pixels to
# the road plane — no new topics, no second projection model. Falls back
# to the legacy "within 20% of image centre" rule when either lane Path
# is empty (UFLD warm-up or the bridge has paused inference inside a
# junction zone).
USE_LANE_ROI        = True
MIN_CONFIDENCE      = 0.8     # detections below this are ignored.
# Was 0.1 — far too permissive: weak YOLO boxes on buildings / poles /
# foliage at ~0.3-0.5 confidence were slipping through the lane-ROI
# filter (when the UFLD polylines either didn't reach the false bb's
# projected X or the bb-bottom IPM-projected to a Y that happened to
# land inside the polylines despite being well off to the image side).
# That made EMERGENCY brake fire on phantom objects.
# 0.6 sits well below every real-lead confidence the
# carlaaccsim/ipm_validate.py sweep observed at d ∈ {3, 5, 7, 10, 15} m
# (0.90-0.95) and well above the false-positive band we were seeing.
# See DEBUG §22 follow-up.
PUBLISH_INF         = True    # publish inf when no vehicle detected. Set False to publish nothing on no-detection
ENABLE_LOGGING      = True  # set True to enable terminal output

# YOLO class names that count as a lead vehicle for ACC purposes.
# Pinhole's OBJECT_HEIGHTS table is gone — IPM gets distance from
# camera-mount geometry, not from per-class real-world height — so we
# only need a *set* of vehicle classes to filter on. See DEBUG §22.
VEHICLE_CLASSES     = {'car', 'truck', 'bus', 'motorcycle'}
# Lower-bound clamp on the published distance. IPM saturates at small
# gaps (bb-bottom clips at frame edge → distance can come out near 0
# or negative). We clamp at 0.1 m so the value is still positive (the
# controller resets state when d ≤ 0), but well below
# `emergency_distance` so the controller's EMERGENCY brake fires.
MIN_PUBLISHED_GAP_M = 0.1
# Centerline is now strict-overlap only: it exists only where both
# UFLD polylines reach the same X. No half-lane offset for the
# single-polyline case, no forward extrapolation past either polyline
# tip. Both fallbacks inherited UFLD's per-polyline drift and shifted
# the centerline by up to ~0.5 m, which was enough to flip the
# bb-intersects-centerline gate on cars just outside the ego lane.


class YoloDetection(Node):
    def __init__(self):
        super().__init__('Perception_Node', namespace='ACC')
        self.get_logger().info("=== Perception Node starting ===")
        self.get_logger().info(f"    Min conf:   {MIN_CONFIDENCE}")
        self.get_logger().info(f"    Publish inf: {PUBLISH_INF}")

        # Subscribers — single-threaded executor (see main()). The
        # MultiThreadedExecutor + callback-group split was an attempt
        # to decouple the centerline timer from the slow YOLO callback,
        # but it caused YOLO itself to stop firing in some runs
        # (camera sub appeared to go silent under multi-threading).
        # Reverted to the original setup that ACC was working under.
        self.create_subscription(
            CompressedImage,
            '/Car_1/camera/front/compressed',
            self.listener_callback,
            10,
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
        # Bridge sets True while ego is inside a CARLA junction zone.
        # We only allow the centre-strip safety net to fire when this
        # is True — outside junctions the centerline gate is the sole
        # authority, and a missing centerline means "no ACC", not
        # "guess via image-space". UFLD is intentionally paused inside
        # junction zones, so without this fallback ACC would also be
        # blind through every intersection.
        self._in_junction = False
        self.create_subscription(
            Bool, '/Car_1/in_junction',
            self._on_in_junction, 10)

        # Publishers
        self.dist_pub  = self.create_publisher(Float32,          '/ACC/lead_vehicle_distance',   10)
        self.debug_pub = self.create_publisher(CompressedImage,  '/ACC/perception/debug_image',  10)
        # Closest in-lane lead's bb-bottom edge, IPM-projected to vehicle
        # frame as (left_corner, right_corner). Two-pose Path so
        # ipm_view_node can draw the ground line corresponding to the
        # distance it just published on /ACC/lead_vehicle_distance.
        # Empty path when no in-lane lead.
        self.bb_ground_pub = self.create_publisher(Path,         '/ACC/lead_bb_ground',          10)
        # Interpolated centerline — exactly what `_centerline_at` returns,
        # sampled at 1 m intervals across the overlap of the two UFLD
        # polylines (single source of truth so the BEV overlay matches
        # the corridor the gate uses). Consumed by ipm_view_node.
        self.centerline_pub = self.create_publisher(Path,        '/LKAS/centerline_debug',       10)

        # Centerline is published from inside estimateLeadDist (per
        # YOLO frame). The independent 10 Hz timer attempt required
        # MultiThreadedExecutor which destabilised YOLO — reverted.

        # Load YOLO model. `model_filename` accepts either a bare
        # filename (resolved against share/perception/models/) or an
        # absolute path (used by the UI dropdown when pointing at a
        # freshly-trained checkpoint outside the package). Mirrors the
        # same convention as lane_detection_node.model_filename.
        self.declare_parameter('model_filename', 'best.pt')
        model_name = self.get_parameter('model_filename').value
        pkg_share  = get_package_share_directory('perception')
        if os.path.isabs(model_name):
            model_path = model_name
        else:
            model_path = os.path.join(pkg_share, 'models', model_name)
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

        # Ego half-length along forward axis (vehicle origin → front
        # bumper). Used to convert the IPM's vehicle-frame X (which is
        # measured from the pivot) into a bumper-to-bumper gap. Default
        # 2.504 m is the CARLA Dodge Charger value; switch to whatever
        # ego blueprint the bridge spawns. A future improvement is for
        # the bridge to publish ego.bounding_box.extent.x on a latched
        # topic so this stays correct across blueprint changes without
        # touching parameters here. See DEBUG §22.
        self.declare_parameter('ego_extent_x', 2.504)
        self.ego_extent_x = self.get_parameter('ego_extent_x').value

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

    def _on_in_junction(self, msg: Bool):
        self._in_junction = bool(msg.data)

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
    
    

    def _centerline_at(self, x_fwd: float) -> float | None:
        """Lane centerline Y_left at the given X_forward. Strict overlap:
        returns the midpoint of the left and right UFLD polylines only
        when *both* reach this X. No half-lane offset for single-polyline
        cases, no forward extrapolation past either tip.

        Returns None outside the overlap zone — caller falls back to
        the image-space centre-strip safety net.

        Earlier variants synthesised the centerline from one polyline
        with a fixed half-lane offset, or extrapolated the further
        polyline forward by a bounded distance. Both inherited UFLD's
        per-polyline drift directly into the centerline and shifted it
        by up to ~0.5 m, which was enough to flip the gate decision on
        cars sitting just outside the ego lane.
        REP 103 axes: Y positive = left of ego."""
        left_y  = self._interp_y_at_x(self.left_veh,  x_fwd)
        right_y = self._interp_y_at_x(self.right_veh, x_fwd)
        if left_y is None or right_y is None:
            return None
        return 0.5 * (left_y + right_y)

    def _bb_intersects_centerline(self, left_g: tuple[float, float],
                                  right_g: tuple[float, float]) -> bool | None:
        """True iff the lane centerline at the lead's forward distance
        passes through (or touches) the horizontal segment spanned by
        the bb's two bottom corners in BEV.

        Geometric intuition: both bb-bottom corners are projected via
        IPM to the road plane and share the same X_forward (same image
        v → same depth in the pinhole model), differing only in Y_left.
        That makes the bb's ground footprint a horizontal segment in
        BEV. If the centerline polyline crosses this segment (i.e. its
        Y at this X falls inside the segment's [y_lo, y_hi]), the lead
        is sitting over the lane centre → engage ACC. Otherwise the
        lead is off to the side → ignore.

        Why this is better than the old "bb-center within ±half-lane
        of centerline" or "bb-center between left and right polylines"
        gates:
          * uses the bb's *full* width as the tolerance, so a wide
            close lead needs less precise centerline alignment than a
            narrow far one — self-scaling
          * correctly admits lane-changers whose footprint partially
            overhangs the centerline
          * cleanly rejects parked / oncoming cars whose footprint sits
            entirely off-corridor, even when UFLD polylines drift

        Returns None when _centerline_at has no signal at this X
        (only one polyline reaches X, or neither does) — caller falls
        back to the image-space centre-strip safety net."""
        x_fwd = left_g[0]  # == right_g[0] (same image v → same depth)
        yc = self._centerline_at(x_fwd)
        if yc is None:
            return None
        y_lo, y_hi = sorted((left_g[1], right_g[1]))
        return y_lo <= yc <= y_hi

    def _publish_centerline_debug(self, header):
        """Sample _centerline_at at 1 m intervals across the overlap of
        both polylines and publish the result as a Path. Same logic as
        the gate, so the red BEV overlay shows exactly the centerline
        used to accept/reject detections. Empty path → no overlap zone
        (one or both polylines are too short)."""
        path = Path()
        path.header = header
        path.header.frame_id = 'base_link'
        # Sample only across the overlap of the two polylines (the
        # shorter tip bounds the centerline length). Beyond that,
        # _centerline_at returns None and there's nothing to draw.
        max_x = 0.0
        if len(self.left_veh) >= 2 and len(self.right_veh) >= 2:
            max_x = min(self.left_veh[-1][0], self.right_veh[-1][0])
        x = 1.0
        while x <= max_x:
            y = self._centerline_at(x)
            if y is not None:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                path.poses.append(pose)
            x += 1.0
        self.centerline_pub.publish(path)

    # ========================
    # CALLBACK
    # ========================
    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_img is None:
            return

        # Lane-shape trapezoid (post-YOLO) is the new ROI — see the
        # bbox-centre gate in estimateLeadDist. No pre-YOLO masking
        # is needed; YOLO runs on the full frame and the trapezoid
        # filters detections at the right pipeline stage.
        img_proc = cv_img.copy()

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
        best_conf    = None        # YOLO conf of the currently-selected lead
        closest_bb_ground = None   # ((X_left,Y_left), (X_right,Y_right)) for BEV

        h, w, _ = img.shape
        img_center_x = w / 2

        for box, conf, cls in zip(boxes, confs, classes):
            x_min, y_min, x_max, y_max = box.astype(int)
            class_name = self.model.names[int(cls)]

            if class_name not in VEHICLE_CLASSES:
                continue

            if conf < MIN_CONFIDENCE:
                continue

            box_center_x = (x_min + x_max) / 2

            # Side-pass guard: a vehicle directly beside the ego in an
            # adjacent lane (overtaking or being overtaken) usually
            # appears as a bbox clipped against exactly ONE image side.
            # Its bb-bottom is then often clipped at the image bottom
            # too — the IPM projection saturates to forward ≈ 3 m and
            # the bb's wide ground footprint spans the lane centerline,
            # falsely triggering the gate and the EMERGENCY brake. A
            # genuine in-front lead is centred: either both sides
            # clipped (very wide truck) or neither — never just one.
            # XOR of left/right clip discriminates the side-pass case
            # without rejecting real centred close leads.
            clipped_left  = x_min <= 5
            clipped_right = x_max >= w - 5
            if clipped_left ^ clipped_right:
                continue

            # Lane-shape keep-zone trapezoid — TEMPORARILY DISABLED to
            # test whether the BEV centerline-intersection gate alone
            # is sufficient. If passing cars still trigger emergency
            # brake with this commented out (and the fallback below
            # also disabled), the false positives are coming from the
            # BEV gate itself (UFLD drift / wide bboxes / lane-changers)
            # not from the fallback. Re-enable by uncommenting.
            if y_max > h * 0.55:
                 t = (y_max - h * 0.55) / (h - h * 0.55)   # 0 → 1
                 keep_half_w = w * (0.05 + 0.18 * t)        # 0.07 → 0.23
                 if abs(box_center_x - img_center_x) > keep_half_w:
                     continue

            # IPM-project BOTH bb-bottom corners up front: they feed
            # the gate (centerline-intersects-bb-segment test) AND the
            # BEV visualisation. Same v (y_max) for both → same
            # X_forward; only Y_left differs, so this is a horizontal
            # ground segment in BEV.
            left_g  = self._pixel_to_vehicle(x_min, y_max, w, h)
            right_g = self._pixel_to_vehicle(x_max, y_max, w, h)
            if left_g is None or right_g is None:
                # bb-bottom at or above horizon — can't ground-project.
                continue

            if USE_LANE_ROI:
                in_lane = self._bb_intersects_centerline(left_g, right_g)
                if in_lane is False:
                    continue
                if in_lane is None:
                    # FALLBACK DISABLED for the diagnostic test. The
                    # gate is now strictly BEV-centerline-intersects-
                    # bb-ground: anything we can't evaluate (no
                    # centerline at this X) is dropped. If passing
                    # cars still trigger emergency brake under this,
                    # the BEV test itself is the source of false
                    # positives (not the fallback). Original branch:
                    #   if not self._in_junction: continue
                    #   if abs(box_center_x - img_center_x) > w * 0.05:
                    #       continue
                    continue
            elif abs(box_center_x - img_center_x) > w * 0.2:
                continue

            # Bumper-to-bumper gap = lead's bb-bottom ground X − ego
            # front bumper X (= ego_extent_x in vehicle frame). Clamped
            # at MIN_PUBLISHED_GAP_M so close-range IPM saturation
            # (gap ≤ 2 m → bb clips at frame edge → IPM under-reads)
            # still trips the controller's EMERGENCY brake instead of
            # being filtered out as d ≤ 0.
            distance_m = max(MIN_PUBLISHED_GAP_M,
                             left_g[0] - self.ego_extent_x)

            if min_distance is None or distance_m < min_distance:
                min_distance = distance_m
                best_conf    = float(conf)   # track conf of selected lead
                closest_bb_ground = (left_g, right_g)

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
                conf_text = (f"{best_conf:.2f}"
                             if best_conf is not None else "n/a")
                self._log_throttled(
                    f"Closest vehicle: {min_distance:.2f} m  |  conf: {conf_text}")
        elif PUBLISH_INF:
            msg = Float32()
            msg.data = float('inf')
            self.dist_pub.publish(msg)

        # ── BB-bottom ground line publish (for ipm_view_node BEV) ──
        bb_path = Path()
        bb_path.header = header
        bb_path.header.frame_id = 'base_link'
        if closest_bb_ground is not None:
            for (x_fwd, y_left) in closest_bb_ground:
                pose = PoseStamped()
                pose.header = bb_path.header
                pose.pose.position.x = float(x_fwd)
                pose.pose.position.y = float(y_left)
                bb_path.poses.append(pose)
        self.bb_ground_pub.publish(bb_path)

        # ── Centerline debug → ipm_view_node BEV ──────────────
        # Same _centerline_at the gate uses, sampled at 1 m intervals.
        # Publishes at YOLO rate (per camera frame). When YOLO is slow
        # the BEV centerline updates slower too — accepted tradeoff
        # since the MultiThreadedExecutor alternative destabilised YOLO.
        self._publish_centerline_debug(header)

        # ── DEBUG: keep-zone trapezoid preview (TEMP) ─────────
        # Yellow outline of the lane-shape keep-zone — anything whose
        # bbox-centre lies outside this trapezoid in the lower band is
        # rejected before reaching the BEV gate. Same numbers as the
        # gate above so the operator can verify alignment with the
        # hand-drawn target. Remove once the geometry is confirmed.
        y_top  = int(h * 0.55)
        h_top  = int(w * 0.05)   # half-width at top of band
        h_bot  = int(w * 0.23)   # half-width at image bottom
        c_img  = w // 2
        trap_pts = np.array([
            (c_img - h_top, y_top),
            (c_img + h_top, y_top),
            (c_img + h_bot, h - 1),
            (c_img - h_bot, h - 1),
        ], dtype=np.int32)
        cv2.polylines(img, [trap_pts], isClosed=True,
                      color=(0, 255, 255), thickness=2,
                      lineType=cv2.LINE_AA)

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
    node = YoloDetection()
    # Single-threaded executor — original setup that YOLO was working
    # under. The MultiThreadedExecutor variant decoupled the centerline
    # timer from YOLO but caused the camera subscription to stop
    # delivering messages in some runs, leaving ACC blind. Reverted.
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