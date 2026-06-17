#!/usr/bin/env python3
"""Combined-overlay debug image.

Earlier version of this node did `cv2.max(acc_debug, lkas_debug)`, which
preserved bright overlays cleanly but blended two different camera frames
together for the background — the visible "two video sequences laid on
top of each other" symptom users reported. Both perception nodes process
the camera at different rates (YOLO ~5–10 Hz, UFLD ~10–15 Hz), so their
debug images almost never share a timestamp.

This version takes a different approach:
  1. Subscribe to the raw camera as well and keep a short ring buffer
     keyed by header.stamp.
  2. When each perception debug image arrives, look up the raw frame it
     was computed from (matched by header.stamp — the perception nodes
     preserve the original camera header), and extract the OVERLAY-ONLY
     pixels as a boolean mask: `mask = |debug − raw| > threshold`.
  3. On each publish tick, paint the most recent ACC and LKAS overlay
     masks onto the LATEST raw frame.

Result:
  * Background = latest raw frame → smooth, no scene ghosting.
  * Overlays = at the pixel positions they were originally drawn at →
    no double-image artifacts. Mild lag visible at higher speeds (the
    YOLO box may trail the actual lead by a frame or two) but that's
    a much milder artifact than the ghosting.

Subscribes:
    /Car_1/camera/front/compressed   (raw camera — for timestamp matching)
    /ACC/perception/debug_image      (CompressedImage)
    /LKAS/perception/debug_image     (CompressedImage)
Publishes:
    /ADAS/perception/debug_image     (CompressedImage)
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


RAW_TOPIC      = '/Car_1/camera/front/compressed'
ACC_TOPIC      = '/ACC/perception/debug_image'
LKAS_TOPIC     = '/LKAS/perception/debug_image'
COMBINED_TOPIC = '/ADAS/perception/debug_image'
PUB_HZ         = 10
JPEG_QUALITY   = 75
# How many recent raw frames to keep for timestamp matching. At a 20 Hz
# camera the buffer covers ~1.5 s, which comfortably absorbs YOLO's
# 50–200 ms inference latency before a perception debug arrives.
RAW_BUFFER_LEN = 30
# Pixel-channel threshold for marking a debug pixel as "overlay" vs raw.
# Empirical: ~25 catches the bright YOLO greens and UFLD reds/greens while
# rejecting JPEG re-encoding noise (raw is JPEG q=95 from the bridge;
# perception debugs are JPEG q=85, so even non-overlay pixels differ
# slightly after decode).
OVERLAY_THRESHOLD = 25
# Reject the timestamp match if the closest raw is more than this many
# nanoseconds away — happens when the camera buffer hasn't caught up yet.
MATCH_TOLERANCE_NS = 150_000_000   # 150 ms


class DebugImageFusionNode(Node):
    def __init__(self):
        super().__init__('debug_image_fusion_node')
        # Ring buffer of (stamp_ns, bgr) for raw camera frames.
        self._raw_buffer: list[tuple[int, np.ndarray]] = []
        # Cached (mask, debug_bgr) pairs — re-painted onto the latest raw
        # frame on every publish tick.
        self._acc_overlay: tuple[np.ndarray, np.ndarray] | None = None
        self._lkas_overlay: tuple[np.ndarray, np.ndarray] | None = None
        self._first_publish_logged = False

        self.create_subscription(CompressedImage, RAW_TOPIC,
                                 self._on_raw, 10)
        self.create_subscription(CompressedImage, ACC_TOPIC,
                                 self._on_acc, 10)
        self.create_subscription(CompressedImage, LKAS_TOPIC,
                                 self._on_lkas, 10)
        self.pub = self.create_publisher(CompressedImage,
                                         COMBINED_TOPIC, 10)
        self.create_timer(1.0 / PUB_HZ, self._publish)
        self.create_timer(5.0, self._heartbeat)
        self.get_logger().info(
            f"Debug-image fusion node started — masking {ACC_TOPIC} + "
            f"{LKAS_TOPIC} against {RAW_TOPIC} → {COMBINED_TOPIC} @ {PUB_HZ} Hz")

    @staticmethod
    def _stamp_ns(msg: CompressedImage) -> int:
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    @staticmethod
    def _decode(msg: CompressedImage) -> np.ndarray | None:
        arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _heartbeat(self):
        if not self._first_publish_logged:
            missing = []
            if not self._raw_buffer:
                missing.append(RAW_TOPIC)
            if self._acc_overlay is None:
                missing.append(ACC_TOPIC)
            if self._lkas_overlay is None:
                missing.append(LKAS_TOPIC)
            if missing:
                self.get_logger().warn(
                    'Still waiting for first frame on: '
                    + ', '.join(missing))

    def _on_raw(self, msg: CompressedImage):
        bgr = self._decode(msg)
        if bgr is None:
            return
        self._raw_buffer.append((self._stamp_ns(msg), bgr))
        if len(self._raw_buffer) > RAW_BUFFER_LEN:
            self._raw_buffer.pop(0)

    def _find_matching_raw(self, target_ns: int) -> np.ndarray | None:
        if not self._raw_buffer:
            return None
        best_ns, best_bgr = min(self._raw_buffer,
                                key=lambda t: abs(t[0] - target_ns))
        if abs(best_ns - target_ns) > MATCH_TOLERANCE_NS:
            return None
        return best_bgr

    def _extract_overlay(self, debug_msg: CompressedImage):
        """Returns (mask, debug_bgr) or None if no matching raw exists.

        `mask` is a HxW bool array; True at pixels the perception node
        painted over (i.e. where the debug image differs from its raw
        source above OVERLAY_THRESHOLD).
        """
        debug_bgr = self._decode(debug_msg)
        if debug_bgr is None:
            return None
        raw = self._find_matching_raw(self._stamp_ns(debug_msg))
        if raw is None or raw.shape != debug_bgr.shape:
            return None
        diff = cv2.absdiff(debug_bgr, raw)
        mask = np.max(diff, axis=-1) > OVERLAY_THRESHOLD
        return (mask, debug_bgr)

    def _on_acc(self, msg: CompressedImage):
        result = self._extract_overlay(msg)
        if result is not None:
            self._acc_overlay = result

    def _on_lkas(self, msg: CompressedImage):
        result = self._extract_overlay(msg)
        if result is not None:
            self._lkas_overlay = result

    def _publish(self):
        if not self._raw_buffer:
            return
        _, latest_raw = self._raw_buffer[-1]
        out_bgr = latest_raw.copy()
        if self._acc_overlay is not None:
            mask, src = self._acc_overlay
            if src.shape == out_bgr.shape:
                out_bgr[mask] = src[mask]
        if self._lkas_overlay is not None:
            mask, src = self._lkas_overlay
            if src.shape == out_bgr.shape:
                out_bgr[mask] = src[mask]

        _, buf = cv2.imencode('.jpg', out_bgr,
                              [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'Car_1/camera/front'
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub.publish(out)
        if not self._first_publish_logged:
            self.get_logger().info(
                f'First fused frame published on {COMBINED_TOPIC}.')
            self._first_publish_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = DebugImageFusionNode()
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
