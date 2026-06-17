#!/usr/bin/env python3
"""AGV base driver: /cmd_vel → command, V3 status → VehicleData + odom."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from jetson_can_msgs.msg import V3Command, V3ExtStatus, V3Status
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scr_sensor.msg import VehicleData
from tf2_ros import TransformBroadcaster

from ds_jetson_bridge.jetson_protocol import (
    CMD_TIMEOUT_MS,
    MODE_CAN,
    RECOVER_STABLE_MS,
    twist_to_motion,
)

# 兼容 nav2（volatile pub）与 ros2 topic pub 默认（transient_local pub）
CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class AgvBaseDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_base_driver")

        self.declare_parameter("link_type", "rs232")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_rate_hz", 50.0)
        self.declare_parameter("status_timeout_ms", 500)
        self.declare_parameter("cmd_timeout_ms", CMD_TIMEOUT_MS)
        self.declare_parameter("max_linear_m_s", 0.8)
        self.declare_parameter("max_angular_rad_s", 1.0)
        self.declare_parameter("cruise_scale", 1.0)
        self.declare_parameter("mode_req", MODE_CAN)
        self.declare_parameter("strafe_jl_from_angular", True)
        self.declare_parameter("strafe_speed_m_s", 0.3)
        self.declare_parameter("sideways_steer_millirad", 1571)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        link_type = self.get_parameter("link_type").get_parameter_value().string_value
        prefix = f"/jetson_{link_type}"

        self._cmd_rate_hz = self.get_parameter("cmd_rate_hz").get_parameter_value().double_value
        self._status_timeout_s = (
            self.get_parameter("status_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._cmd_timeout_s = (
            self.get_parameter("cmd_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._max_linear = (
            self.get_parameter("max_linear_m_s").get_parameter_value().double_value
        )
        self._max_angular = (
            self.get_parameter("max_angular_rad_s").get_parameter_value().double_value
        )
        self._cruise_scale = (
            self.get_parameter("cruise_scale").get_parameter_value().double_value
        )
        self._mode_req = (
            self.get_parameter("mode_req").get_parameter_value().integer_value
        )
        self._strafe_jl = (
            self.get_parameter("strafe_jl_from_angular")
            .get_parameter_value()
            .bool_value
        )
        self._strafe_speed_m_s = (
            self.get_parameter("strafe_speed_m_s").get_parameter_value().double_value
        )
        self._sideways_steer_millirad = (
            self.get_parameter("sideways_steer_millirad")
            .get_parameter_value()
            .integer_value
        )
        self._odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )

        self._latest_status: V3Status | None = None
        self._latest_ext: V3ExtStatus | None = None
        self._last_status_time = 0.0

        self._pending_v_mm_s = 0
        self._pending_omega = 0
        self._pending_steer = 0
        self._pending_motion_model = 0
        self._last_cmd_time = time.monotonic()
        self._last_motion_v_mm_s = 0
        self._last_motion_omega = 0
        self._last_motion_steer = 0
        self._recover_until = 0.0

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._last_odom_time: float | None = None

        cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )

        self._sub_cmd = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_cb, CMD_VEL_QOS
        )
        self._sub_status = self.create_subscription(
            V3Status, f"{prefix}/v3_status", self._status_cb, 10
        )
        self._sub_ext = self.create_subscription(
            V3ExtStatus, f"{prefix}/v3_ext_status", self._ext_cb, 10
        )

        self._pub_command = self.create_publisher(V3Command, f"{prefix}/command", 10)
        self._pub_vehicle = self.create_publisher(
            VehicleData, "/vehicle/vehicle_data", 10
        )
        self._pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / self._cmd_rate_hz if self._cmd_rate_hz > 0 else 0.02
        self._cmd_timer = self.create_timer(period, self._publish_command)
        self._state_timer = self.create_timer(period, self._publish_state)

        self.get_logger().info(
            f"agv_base_driver link={link_type} cmd={cmd_vel_topic} "
            f"→ {prefix}/command, /vehicle/vehicle_data, /odom"
        )

    def _status_cb(self, msg: V3Status) -> None:
        self._latest_status = msg
        self._last_status_time = time.monotonic()

    def _ext_cb(self, msg: V3ExtStatus) -> None:
        self._latest_ext = msg

    def _cmd_vel_cb(self, msg: Twist) -> None:
        lx = msg.linear.x * self._cruise_scale
        ly = msg.linear.y * self._cruise_scale
        az = msg.angular.z * self._cruise_scale
        lx = max(-self._max_linear, min(self._max_linear, lx))
        ly = max(-self._max_linear, min(self._max_linear, ly))
        az = max(-self._max_angular, min(self._max_angular, az))

        motion = twist_to_motion(
            lx,
            ly,
            az,
            strafe_jl_from_angular=self._strafe_jl,
            strafe_speed_m_s=self._strafe_speed_m_s,
            sideways_steer_millirad=self._sideways_steer_millirad,
        )
        self._pending_v_mm_s = motion.v_mm_s
        self._pending_omega = motion.omega_millirad_s
        self._pending_steer = motion.steer_millirad
        self._pending_motion_model = motion.motion_model
        self._last_cmd_time = time.monotonic()

        if (
            self._pending_v_mm_s != 0
            or self._pending_omega != 0
            or self._pending_steer != 0
        ):
            self._last_motion_v_mm_s = self._pending_v_mm_s
            self._last_motion_omega = self._pending_omega
            self._last_motion_steer = self._pending_steer
            self.get_logger().info(
                f"cmd_vel → v={self._pending_v_mm_s}mm/s "
                f"omega={self._pending_omega} steer={self._pending_steer}",
                throttle_duration_sec=2.0,
            )

    def _in_recovery(self) -> bool:
        return time.monotonic() < self._recover_until

    def _start_recovery(self) -> None:
        self._recover_until = time.monotonic() + RECOVER_STABLE_MS / 1000.0
        self._pending_v_mm_s = 0
        self._pending_omega = 0
        self._pending_steer = 0
        self._pending_motion_model = 0

    def _resolve_motion(self) -> tuple[int, int, int, int, str]:
        now = time.monotonic()
        cmd_stale = (now - self._last_cmd_time) > self._cmd_timeout_s
        had_recent_motion = (
            self._last_motion_v_mm_s != 0
            or self._last_motion_omega != 0
            or self._last_motion_steer != 0
        )
        if had_recent_motion and cmd_stale and not self._in_recovery():
            self._start_recovery()
            self.get_logger().warn(
                "cmd_vel 超时，进入 1s 零速恢复",
                throttle_duration_sec=2.0,
            )

        allow_motion = (
            not self._in_recovery()
            and not cmd_stale
            and (
                self._pending_v_mm_s != 0
                or self._pending_omega != 0
                or self._pending_steer != 0
            )
        )

        if self._in_recovery():
            if now >= self._recover_until:
                self._recover_until = 0.0
                if cmd_stale:
                    self._last_motion_v_mm_s = 0
                    self._last_motion_omega = 0
                    self._last_motion_steer = 0
                else:
                    self.get_logger().info("恢复完成，继续执行 cmd_vel")
            return 0, 0, 0, 0, "RECOVER"
        if not allow_motion:
            return 0, 0, 0, 0, "IDLE"
        return (
            self._pending_v_mm_s,
            self._pending_omega,
            self._pending_steer,
            self._pending_motion_model,
            "RUN",
        )

    def _publish_command(self) -> None:
        v, omega, steer, motion_model, mode = self._resolve_motion()
        msg = V3Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.seq = 0
        msg.mode_req = self._mode_req
        msg.v_mm_s = v
        msg.omega_millirad_s = omega
        msg.steer_millirad = steer
        msg.motion_model = motion_model
        msg.light_enable = 0
        msg.light_mode = 0
        msg.clear_error = 0
        self._pub_command.publish(msg)

        if mode == "RUN" and (v != 0 or omega != 0 or steer != 0):
            self.get_logger().debug(
                f"command RUN v={v} omega={omega} steer={steer} motion={motion_model}"
            )

    def _status_valid(self) -> bool:
        if self._latest_status is None:
            return False
        return (time.monotonic() - self._last_status_time) <= self._status_timeout_s

    def _build_vehicle_data(self) -> VehicleData | None:
        if self._latest_status is None:
            return None
        s = self._latest_status
        msg = VehicleData()
        msg.header = s.header
        msg.linear_velocity_mm_s = s.fb_v_mm_s
        msg.angular_velocity_millirad_s = s.fb_omega_millirad_s
        msg.steer_millirad = s.fb_steer_millirad
        msg.safety_state = s.safety_state
        msg.limit_factor = s.limit_factor
        msg.link_state = s.link_state
        msg.sonar_front_mm = s.sonar_front_mm
        msg.sonar_back_mm = s.sonar_back_mm
        msg.sonar_left_mm = s.sonar_left_mm
        msg.sonar_right_mm = s.sonar_right_mm
        msg.battery_voltage_0p1v = s.battery_voltage_0p1v
        msg.battery_soc_percent = s.battery_soc
        msg.v3_status_seq = s.seq
        msg.data_valid = self._status_valid()

        if self._latest_ext is not None:
            e = self._latest_ext
            msg.wheel_rf = e.wheel_rf
            msg.wheel_rr = e.wheel_rr
            msg.wheel_lr = e.wheel_lr
            msg.wheel_lf = e.wheel_lf
            msg.motor_temp_max_c = e.motor_temp_max_c
            msg.driver_state_or = e.driver_state_or
            msg.v3_ext_status_seq = e.seq
        return msg

    def _integrate_odom(self, v_mm_s: int, omega_millirad_s: int) -> None:
        now = time.monotonic()
        if self._last_odom_time is None:
            self._last_odom_time = now
            return
        dt = now - self._last_odom_time
        self._last_odom_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        v = v_mm_s / 1000.0
        omega = omega_millirad_s / 1000.0
        self._odom_theta += omega * dt
        self._odom_x += v * math.cos(self._odom_theta) * dt
        self._odom_y += v * math.sin(self._odom_theta) * dt

    def _publish_state(self) -> None:
        vehicle = self._build_vehicle_data()
        if vehicle is None:
            return

        self._pub_vehicle.publish(vehicle)

        v_mm_s = vehicle.linear_velocity_mm_s
        omega_millirad_s = vehicle.angular_velocity_millirad_s
        if vehicle.data_valid:
            self._integrate_odom(v_mm_s, omega_millirad_s)

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = math.sin(self._odom_theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._odom_theta / 2.0)
        odom.twist.twist.linear.x = v_mm_s / 1000.0
        odom.twist.twist.angular.z = omega_millirad_s / 1000.0
        self._pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = self._odom_x
        tf.transform.translation.y = self._odom_y
        tf.transform.rotation.z = math.sin(self._odom_theta / 2.0)
        tf.transform.rotation.w = math.cos(self._odom_theta / 2.0)
        self._tf_broadcaster.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = AgvBaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
