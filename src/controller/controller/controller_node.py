#!/usr/bin/env python3
"""
ACC Controller Node (ROS2 Humble)
=================================
Adaptive Cruise Control node that subscribes to ego vehicle speed and
lead vehicle distance (from the YOLO perception node) and publishes
throttle/brake commands via Twist messages.

Control modes:
    CRUISE    – No lead vehicle detected → maintain target speed.
    ACC       – Lead vehicle detected → maintain safe following distance.
    EMERGENCY – Lead vehicle critically close → full brake immediately.

Control law:
    a = k_p * (d_lead - d_desired) + k_d * closing_rate
    where d_desired = d0 + T_gap * v_ego

Subscribed topics:
    /Car_1/vehicle/speed              (Float64)  – ego velocity [m/s]
    /ACC/lead_vehicle_distance        (Float32)  – distance to lead [m]

Published topics:
    /Car_1/cmd_vel                    (Twist)    – linear.x = throttle,
                                                   linear.y = brake.
                                                   Steer is owned by
                                                   stanley_node and goes
                                                   straight to /Car_1/cmd_steer.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64 as StdFloat64
from example_interfaces.msg import Float64 as ExFloat64
from geometry_msgs.msg import Twist

# ========================
# CONFIG FLAGS
# ========================
ENABLE_LOGGING = False  # set True to enable terminal output


class ACCNode(Node):
    """Adaptive Cruise Control node with PD-based distance control."""

    def __init__(self):
        super().__init__('Controller_Node', namespace='ACC')

        # ── Simulator parameter (carla | morai) ──────────────────────────
        self.declare_parameter('simulator', 'carla')
        simulator = self.get_parameter('simulator').get_parameter_value().string_value
        self.get_logger().info(f"[INFO] Simulator: {simulator}")

        # ── Subscriptions ────────────────────────────────────────────────
        SpeedMsg = ExFloat64 if simulator == 'morai' else StdFloat64
        self.create_subscription(SpeedMsg, '/Car_1/vehicle/speed', self.ego_velocity_callback, 20)
        self.create_subscription(Float32, '/ACC/lead_vehicle_distance', self.lead_distance_callback, 20)
        self.create_subscription(Float32, '/ACC/target_speed', self.target_speed_callback, 10)

        # ── Publisher ────────────────────────────────────────────────────
        # Twist: linear.x = throttle, linear.y = brake  [0.0 … 1.0]
        self.control_pub = self.create_publisher(Twist, '/Car_1/cmd_vel', 20)

        # ── ACC parameters ───────────────────────────────────────────────
        self.target_speed      = 20 / 3.6  # [m/s]
        self.d0                = 5.0       # standstill gap [m]
        # d_desired = d0 + T_gap * v_ego. With T_gap=1.5 s the formula gave
        # 13.4 m at 20 km/h cruise, so ACC began braking the moment YOLO saw
        # a lead inside ~13 m — felt over-cautious for the demo. T_gap=0.5 s
        # gives ~7.8 m at cruise (settling to d0=5 m at standstill), which
        # matches the "follow at roughly 5 m" mental model.
        self.T_gap             = 0.3
        self.k_p               = 1.2       # proportional gain
        self.k_d               = 0.8       # derivative gain
        self.a_max             = 3.0       # [m/s²]
        self.a_min             = -6.0      # [m/s²]
        self.emergency_distance = 3.0      # full brake below this distance [m]
        self.throttle_scale    = 3.0       # a_max → full throttle
        self.brake_scale       = 3.0       # a_min → full brake
        self.prev_throttle     = 0.0
        self.THROTTLE_RATE_LIMIT = 0.05  # max throttle increase per step (20 Hz → 1.0 in ~1 second)

        # ── Distance filter ──────────────────────────────────────────────
        # Low-pass on /ACC/lead_vehicle_distance. ALPHA=0.01 effectively
        # froze the filter (~100-sample memory at variable YOLO Hz), so
        # the controller saw a stale "lots of room" reading and accelerated
        # into the back of a closing lead. 0.4 gives ~3-sample (≈300 ms)
        # response while still smoothing single-frame YOLO jitter.
        self.ALPHA             = 0.4
        self.d_lead_filtered   = None

        # ── Internal state ───────────────────────────────────────────────
        self.v_ego             = 0.0
        self.d_lead            = None
        self.prev_d_lead       = None
        self.prev_time         = None
        self.last_log_time     = 0.0

        # ── Control loop @ 20 Hz ─────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"ACC Node initialized | target={self.target_speed:.1f} m/s | "
            f"d0={self.d0} m | T_gap={self.T_gap} s | "
            f"k_p={self.k_p} | k_d={self.k_d}"
        )

    # ====================================================================
    # CALLBACKS
    # ====================================================================

    def ego_velocity_callback(self, msg):
        """Store the latest ego vehicle speed."""
        try:
            self.v_ego = abs(msg.data)
        except Exception as e:
            self.get_logger().error(f"Ego velocity callback error: {e}")

    def lead_distance_callback(self, msg: Float32):
        """Store the latest lead vehicle distance, filtered to reduce noise."""
        try:
            d = msg.data

            if d == float('inf') or d <= 0.0:
                # No detection — reset filter and fall back to cruise mode
                self.d_lead = None
                self.d_lead_filtered = None
                return

            # Low-pass filter: smooth out noisy distance measurements
            # d_filtered = ALPHA * d_new + (1 - ALPHA) * d_prev
            if self.d_lead_filtered is None:
                self.d_lead_filtered = d  # initialise on first valid detection
            else:
                self.d_lead_filtered = self.ALPHA * d + (1 - self.ALPHA) * self.d_lead_filtered

            self.d_lead = self.d_lead_filtered

        except Exception as e:
            self.get_logger().error(f"Lead distance callback error: {e}")

    def target_speed_callback(self, msg):
        self.target_speed = msg.data / 3.6
        self.get_logger().info(f"Target speed updated: {self.target_speed:.1f} m/s")

    # ====================================================================
    # ACC CONTROL LAW
    # ====================================================================

    def acc_control(self, v_ego: float, d_lead: float) -> float:
        """
        PD-based ACC control law.

        Computes a desired longitudinal acceleration based on:
          - The distance error:  d_lead − d_desired
          - The closing rate:    d(d_lead)/dt  (estimated numerically)

        Parameters
        ----------
        v_ego  : float – ego vehicle speed [m/s]
        d_lead : float – distance to lead vehicle [m]

        Returns
        -------
        float – desired acceleration [m/s²], clipped to [a_min, a_max]
        """
        # Desired following distance: standstill gap + speed-dependent gap.
        d_desired = self.d0 + self.T_gap * v_ego

        # -- Estimate closing rate from consecutive distance measurements --
        # closing_rate > 0  → gap is increasing  (lead pulls away)
        # closing_rate < 0  → gap is shrinking    (ego approaches lead)
        # closing_rate ≈ 0  → gap is stable
        now = self.get_clock().now().nanoseconds / 1e9
        closing_rate = 0.0

        if self.prev_d_lead is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0.01:  # guard against division by zero / tiny dt
                closing_rate = (d_lead - self.prev_d_lead) / dt

        # Store current values for next iteration.
        self.prev_d_lead = d_lead
        self.prev_time = now

        # -- PD control --
        # P-term: positive when too far (accelerate), negative when too
        #         close (decelerate).
        # D-term: positive when gap grows (can accelerate), negative when
        #         gap shrinks (should brake earlier).
        distance_error = d_lead - d_desired
        a = self.k_p * distance_error + self.k_d * closing_rate

        # Clip to physical limits.
        a = max(min(a, self.a_max), self.a_min)

        # Deadband: if distance error is small and speed is very low,
        # suppress tiny throttle commands to prevent creeping.
        if abs(distance_error) < 1.0 and v_ego < 0.5 and a > 0:
            a = 0.0

        return a

    # ====================================================================
    # CRUISE CONTROL (fallback when no lead vehicle detected)
    # ====================================================================

    def cruise_control(self) -> tuple:
        """
        Simple proportional speed controller for cruising.

        Returns (throttle, brake) tuple.
        When no lead vehicle is visible the ego vehicle gently
        accelerates/maintains toward the target speed.
        """
        speed_error = self.target_speed - self.v_ego

        if speed_error > 0:
            # Proportional gain: 0.3 per 1 m/s error, capped at full throttle.
            # At 10 m/s error → throttle = 1.0 (full power to reach target)
            # At  2 m/s error → throttle = 0.6 (gentle approach)
            throttle = min(speed_error * 0.3, 1.0)
            brake = 0.0
        elif speed_error < -1.0:
            # More than 1 m/s over target → light braking.
            throttle = 0.0
            brake = min(-speed_error * 0.1, 0.3)
        else:
            # Within tolerance → coast.
            throttle = 0.0
            brake = 0.0

        return throttle, brake

    # ====================================================================
    # MAIN CONTROL LOOP  (called at 20 Hz)
    # ====================================================================

    def control_loop(self):
        """
        Main control loop – runs at 20 Hz.

        Decision hierarchy:
        1. Standstill hold  → already stopped and within range
        2. No lead vehicle  → CRUISE mode (maintain target speed)
        3. Lead vehicle critically close → EMERGENCY full brake
        4. Lead vehicle in range → ACC mode (PD distance control)
        """
        control_msg = Twist()

        # ---- MODE 1: STANDSTILL HOLD ----
        # Suppress control when stopped and within acceptable distance range.
        # Prevents the derivative term from reacting to sensor noise at rest.
        if self.v_ego < 0.5 and self.d_lead is not None and self.d_lead < self.d0 + 2.0:
            control_msg.linear.y = 0.05  # light hold brake
            self.control_pub.publish(control_msg)
            self.prev_throttle = 0.0
            if ENABLE_LOGGING:
                self._log_throttled("STANDSTILL", 0.0, 0.05)
            return

        # ---- MODE 2: CRUISE (no lead vehicle detected) ----
        if self.d_lead is None:
            throttle, brake = self.cruise_control()

            # Rate limit throttle
            throttle = min(throttle, self.prev_throttle + self.THROTTLE_RATE_LIMIT)
            self.prev_throttle = throttle

            control_msg.linear.x = throttle
            control_msg.linear.y = brake
            self.control_pub.publish(control_msg)

            # Reset closing-rate state since there is no lead vehicle
            self.prev_d_lead = None
            self.prev_time = None

            if ENABLE_LOGGING:
                self._log_throttled("CRUISE", throttle, brake)
            return

        # ---- MODE 3: EMERGENCY BRAKE (critically close) ----
        if self.d_lead < self.emergency_distance:
            control_msg.linear.x = 0.0
            control_msg.linear.y = 1.0  # full brake — no rate limit on braking
            self.control_pub.publish(control_msg)
            self.prev_throttle = 0.0
            if ENABLE_LOGGING:
                self._log_throttled("EMERGENCY", 0.0, 1.0)
            return

        # ---- MODE 4: ACC (adaptive distance control) ----
        a = self.acc_control(self.v_ego, self.d_lead)

        if a >= 0:
            throttle = min(a / self.throttle_scale, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-a / self.brake_scale, 1.0)

        # Rate limit throttle — braking is always instant
        throttle = min(throttle, self.prev_throttle + self.THROTTLE_RATE_LIMIT)
        self.prev_throttle = throttle

        control_msg.linear.x = throttle
        control_msg.linear.y = brake

        self.control_pub.publish(control_msg)

        if ENABLE_LOGGING:
            self._log_throttled("ACC", throttle, brake)

    # ====================================================================
    # LOGGING HELPER
    # ====================================================================

    def _log_throttled(self, mode: str, throttle: float, brake: float):
        """Overwrite a single terminal line at most once per second."""
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_log_time > 1.0:
            d_text = f"{self.d_lead:.2f} m" if self.d_lead is not None else "None"
            print(
                f"\r[{mode:>9}] "
                f"v={self.v_ego:.2f} m/s  "
                f"d={d_text}  "
                f"thr={throttle:.2f}  "
                f"brk={brake:.2f}          ",
                end='', flush=True
            )
            self.last_log_time = now


# ========================================================================
# ENTRY POINT
# ========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ACCNode()
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