#!/usr/bin/env python3
"""
LKAS Stanley Controller Node (ROS2 Humble)
==========================================
Consumes the ego-left / ego-right lane polylines from `lane_detection_node`
and ego speed, and publishes a normalised steer command in [-1, 1].

Ported from `02_UFLD_V2/lkas_validate_0.10.0.py` — `stanley_steer` and
`lane_center_at_lookahead` are lifted unchanged. The CARLA-map-driven junction
policy is deliberately NOT ported here (would leak the simulator into the
controller); when detection is unreliable the node falls back to steer=0
(HOLD mode), matching the validate script.

Frame convention: incoming Path messages are REP 103 (X forward, Y LEFT).
Internally the Stanley math uses Y RIGHT positive to match CARLA's steer
sign (positive = right), so we negate on input.

Subscribed topics:
    /LKAS/ego_lane_left   (nav_msgs/Path)
    /LKAS/ego_lane_right  (nav_msgs/Path)
    /Car_1/vehicle/speed  (std_msgs/Float64)

Published topics:
    /Car_1/cmd_steer      (std_msgs/Float32)  — normalised steer in [-1, 1].
                          Consumed directly by the CARLA bridge; the ACC
                          controller no longer relays it via angular.z.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import Float32, Float64


# Stanley gains — matched to lkas_validate_0.9.16.py (the CARLA version this
# ROS stack actually runs against). The 0.10.0 tuning (k=1.0, max=40°) is
# sharper and oscillates on 0.9.16 physics.
STANLEY_K     = 0.5
STANLEY_EPS   = 0.5
MAX_STEER_RAD = math.radians(70)
LOOKAHEAD_M   = 5.0


def stanley_steer(e_lat: float, e_head: float, speed_mps: float) -> float:
    """Returns a normalised steer ∈ [-1, 1]. Positive = right."""
    delta = e_head + math.atan2(STANLEY_K * e_lat, speed_mps + STANLEY_EPS)
    return max(-1.0, min(1.0, delta / MAX_STEER_RAD))


def lane_center_at_lookahead(left_veh, right_veh, lookahead_m: float):
    """Returns ((x_near, y_near), (x_far, y_far)) in vehicle frame metres
    (Y RIGHT positive — caller is responsible for sign). None if not enough
    data to interpolate at the lookahead."""
    if not left_veh or not right_veh:
        return None

    def interp_y_at_x(poly, x_target):
        xs = np.array([p[0] for p in poly])
        ys = np.array([p[1] for p in poly])
        order = np.argsort(xs)
        xs, ys = xs[order], ys[order]
        if x_target < xs[0] or x_target > xs[-1]:
            return None
        return float(np.interp(x_target, xs, ys))

    left_y  = interp_y_at_x(left_veh,  lookahead_m)
    right_y = interp_y_at_x(right_veh, lookahead_m)
    if left_y is None or right_y is None:
        return None
    y_center = (left_y + right_y) / 2.0

    x_far = lookahead_m + 1.0
    lf = interp_y_at_x(left_veh,  x_far)
    rf = interp_y_at_x(right_veh, x_far)
    if lf is None or rf is None:
        x_far = lookahead_m + 0.5
        lf = interp_y_at_x(left_veh,  x_far)
        rf = interp_y_at_x(right_veh, x_far)
        if lf is None or rf is None:
            return ((lookahead_m, y_center), None)
    return ((lookahead_m, y_center), (x_far, (lf + rf) / 2.0))


class StanleyNode(Node):
    def __init__(self):
        super().__init__('Stanley_Node', namespace='LKAS')
        self.get_logger().info("=== Stanley Node starting ===")

        self.declare_parameter('lookahead_m', LOOKAHEAD_M)
        self.declare_parameter('speed_topic', '/Car_1/vehicle/speed')
        self.declare_parameter('control_rate_hz', 20.0)
        self.lookahead = self.get_parameter('lookahead_m').value
        speed_topic    = self.get_parameter('speed_topic').value
        rate           = self.get_parameter('control_rate_hz').value

        self.create_subscription(Path, 'ego_lane_left',
                                 self.left_callback,  10)
        self.create_subscription(Path, 'ego_lane_right',
                                 self.right_callback, 10)
        self.create_subscription(Float64, speed_topic,
                                 self.speed_callback, 20)
        # Absolute topic name — bypasses the LKAS namespace so steer lands on
        # the same /Car_1/* tree the bridge already owns.
        self.steer_pub = self.create_publisher(Float32, '/Car_1/cmd_steer', 20)

        self.left_veh  = []      # list of (X_forward, Y_right)
        self.right_veh = []
        self.speed     = 0.0
        self.last_log_time = 0.0

        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f"Stanley initialised | lookahead={self.lookahead} m | "
            f"rate={rate} Hz | speed_topic={speed_topic}"
        )

    # ── Convert nav_msgs/Path (REP 103, Y LEFT) → list of (X_fwd, Y_right) ─
    @staticmethod
    def _path_to_veh(path: Path):
        return [(pose.pose.position.x, -pose.pose.position.y)
                for pose in path.poses]

    def left_callback(self, msg: Path):
        self.left_veh = self._path_to_veh(msg)

    def right_callback(self, msg: Path):
        self.right_veh = self._path_to_veh(msg)

    def speed_callback(self, msg: Float64):
        self.speed = abs(msg.data)

    def control_loop(self):
        lookahead = lane_center_at_lookahead(self.left_veh, self.right_veh,
                                             self.lookahead)
        if lookahead is None:
            steer = 0.0
            mode = 'HOLD'
            e_lat = e_head = float('nan')
        else:
            near_pt, far_pt = lookahead
            e_lat = near_pt[1]
            if far_pt is None:
                e_head = 0.0
            else:
                dx = far_pt[0] - near_pt[0]
                dy = far_pt[1] - near_pt[1]
                e_head = math.atan2(dy, dx)
            steer = stanley_steer(e_lat, e_head, self.speed)
            mode = 'STANLEY'

        out = Float32()
        out.data = float(steer)
        self.steer_pub.publish(out)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            e_lat_s  = f'{e_lat:+0.2f}'  if not math.isnan(e_lat)  else 'n/a'
            e_head_s = f'{math.degrees(e_head):+0.1f}' if not math.isnan(e_head) else 'n/a'
            self.get_logger().info(
                f'[{mode:>7}] v={self.speed:5.2f} m/s  '
                f'e_lat={e_lat_s} m  e_head={e_head_s} deg  '
                f'steer={steer:+0.3f}'
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
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
