#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
from sensor_msgs.msg import CompressedImage


# ========================
# CONFIG FLAGS
# ========================
USE_ROI             = False   # ← flip to True to enable ROI mask
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

        # Subscriber
        self.create_subscription(
            CompressedImage,
            '/Car_1/camera/front/compressed',
            self.listener_callback,
            10
        )

        # Publishers
        self.dist_pub  = self.create_publisher(Float32,          '/ACC/lead_vehicle_distance',   10)
        self.debug_pub = self.create_publisher(CompressedImage,  '/ACC/perception/debug_image',  10)

        # Load YOLO model
        pkg_share = get_package_share_directory('perception')
        model_path = os.path.join(pkg_share, 'models', 'best.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded from {model_path}")

        self.last_log_time = 0.0

        # Focal Length: 1280 / (2 * tan(45°)) = 640
        self.FOCAL_LENGTH = 640.0

        self.OBJECT_HEIGHTS = {
            "car":        1.5,
            "truck":      3.0,
            "bus":        3.2,
            "motorcycle": 1.2,
        }

    # ========================
    # ROI MASK (optional)
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

        for box, conf, cls in zip(boxes, confs, classes):
            x_min, y_min, x_max, y_max = box.astype(int)
            class_name = self.model.names[int(cls)]

            if class_name not in self.OBJECT_HEIGHTS:
                continue

            if conf < MIN_CONFIDENCE:
                continue

            box_center_x = (x_min + x_max) / 2
            if abs(box_center_x - img_center_x) > w * 0.2:
                continue

            box_height = y_max - y_min
            if box_height <= 0:
                continue

            real_height  = self.OBJECT_HEIGHTS[class_name]
            distance_m   = (self.FOCAL_LENGTH * real_height) / box_height

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