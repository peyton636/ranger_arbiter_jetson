#!/usr/bin/env python3
"""以太网底盘统一节点：UDP(BLOB) + 对外仅 4 个业务 Topic + /fix。"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from jetson_mcu_msgs.msg import V3Command, V3ExtStatus, V3Status
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scr_sensor.msg import AgvControl, AgvFeatureStatus, VehicleData
from tf2_ros import TransformBroadcaster

from eth_gateway.chassis_fusion import build_feature_status, build_vehicle_data
from eth_gateway.gps_fix import GpsFixHelper
from eth_gateway.jetson_protocol import twist_to_motion
from eth_gateway.mcu_eth_bridge import McuEthBridge

CMD_TIMEOUT_S = 0.5
RECOVER_STABLE_S = 1.0

AGV_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class AgvBaseEthBringeNode(Node):
    def __init__(self) -> None:
        super().__init__("agv_base_bringe")

        self.declare_parameter("agv_control_topic", "/agv_control")
        self.declare_parameter("vehicle_topic", "/Vehicle/VehicleData")
        self.declare_parameter("feature_status_topic", "/Function/FeatureStatusInfo")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gps_enable", True)
        self.declare_parameter("state_rate_hz", 50.0)
        self.declare_parameter("status_timeout_ms", 500)
        self.declare_parameter("cmd_timeout_ms", 500)
        self.declare_parameter("max_linear_m_s", 0.8)
        self.declare_parameter("max_angular_rad_s", 1.0)
        self.declare_parameter("cruise_scale", 1.0)
        self.declare_parameter("mode_req", 1)
        self.declare_parameter("strafe_jl_from_angular", True)
        self.declare_parameter("strafe_speed_m_s", 0.3)
        self.declare_parameter("sideways_steer_millirad", 1571)

        McuEthBridge.declare_parameters(self)
        pub_flags = McuEthBridge.read_ros_publish_flags(self)
        # 生产模式：不对外发布 /jetson_eth/*
        pub_flags["enable_ros_uplink_publish"] = False
        pub_flags["enable_ros_link_publish"] = False
        pub_flags["enable_ros_time_sync_publish"] = False
        pub_flags["enable_sensor_cfg_sub"] = False

        self._status_timeout_s = (
            self.get_parameter("status_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        self._cmd_timeout_s = (
            self.get_parameter("cmd_timeout_ms").get_parameter_value().integer_value / 1000.0
        )
        self._max_linear = self.get_parameter("max_linear_m_s").get_parameter_value().double_value
        self._max_angular = (
            self.get_parameter("max_angular_rad_s").get_parameter_value().double_value
        )
        self._cruise_scale = self.get_parameter("cruise_scale").get_parameter_value().double_value
        self._mode_req = self.get_parameter("mode_req").get_parameter_value().integer_value
        self._strafe_jl = (
            self.get_parameter("strafe_jl_from_angular").get_parameter_value().bool_value
        )
        self._strafe_speed = (
            self.get_parameter("strafe_speed_m_s").get_parameter_value().double_value
        )
        self._sideways_steer = (
            self.get_parameter("sideways_steer_millirad").get_parameter_value().integer_value
        )
        self._odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._gps_enable = self.get_parameter("gps_enable").get_parameter_value().bool_value

        self._latest_status: V3Status | None = None
        self._latest_ext: V3ExtStatus | None = None
        self._last_status_time = 0.0
        self._jetson_link_up = False
        self._offset_ms = 0.0
        self._offset_valid = False

        self._pending_v = 0
        self._pending_omega = 0
        self._pending_steer = 0
        self._pending_model = 0
        self._pending_mode_req = self._mode_req
        self._pending_clear_error = 0
        self._light_enable = 1
        self._light_mode = 0
        self._last_cmd_time = 0.0
        self._recover_until = 0.0
        self._last_motion_v = 0
        self._last_motion_omega = 0
        self._last_motion_steer = 0

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._last_odom_time: float | None = None

        agv_topic = self.get_parameter("agv_control_topic").get_parameter_value().string_value
        vehicle_topic = self.get_parameter("vehicle_topic").get_parameter_value().string_value
        feature_topic = (
            self.get_parameter("feature_status_topic").get_parameter_value().string_value
        )
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        fix_topic = self.get_parameter("fix_topic").get_parameter_value().string_value

        self._pub_vehicle = self.create_publisher(VehicleData, vehicle_topic, 10)
        self._pub_feature = self.create_publisher(AgvFeatureStatus, feature_topic, 10)
        self._pub_odom = self.create_publisher(Odometry, odom_topic, 10)
        self._tf = TransformBroadcaster(self)
        self._gps: GpsFixHelper | None = None
        if self._gps_enable:
            self._gps = GpsFixHelper(self, fix_topic=fix_topic, use_time_sync_stamp=True)

        self._bridge = McuEthBridge(
            self,
            "/jetson_eth",
            **pub_flags,
            on_v3_status=self._on_v3_status,
            on_v3_ext=self._on_v3_ext,
            on_gps_frames=self._on_gps_frames if self._gps else None,
            on_time_sync=self._on_time_sync,
        )

        self.create_subscription(AgvControl, agv_topic, self._on_agv_control, AGV_QOS)

        hz = self.get_parameter("state_rate_hz").get_parameter_value().double_value
        period = 1.0 / hz if hz > 0 else 0.02
        self.create_timer(period, self._publish_state)

        self.get_logger().info(
            f"agv_base_eth_bringe: 对外 {agv_topic} → UDP | "
            f"{vehicle_topic}, {feature_topic}, {odom_topic}"
            + (f", {fix_topic}" if self._gps else "")
        )

    def _on_v3_status(self, msg: V3Status) -> None:
        self._latest_status = msg
        self._last_status_time = time.monotonic()

    def _on_v3_ext(self, msg: V3ExtStatus) -> None:
        self._latest_ext = msg

    def _on_gps_frames(self, a, b, c) -> None:
        if self._gps is not None:
            self._gps.on_gps_frames(a, b, c)

    def _on_time_sync(self, msg) -> None:
        if msg.offset_valid:
            self._offset_ms = msg.offset_ms
            self._offset_valid = True
        if self._gps is not None:
            self._gps.on_time_sync(msg)

    def _on_agv_control(self, msg: AgvControl) -> None:
        self._pending_mode_req = msg.mode_req
        self._light_enable = msg.light_enable if msg.light_enable else 1
        self._light_mode = msg.light_mode
        if msg.clear_error:
            self._pending_clear_error = msg.clear_error

        if msg.use_twist_input:
            lx = max(-self._max_linear, min(self._max_linear, msg.linear_x * self._cruise_scale))
            ly = max(-self._max_linear, min(self._max_linear, msg.linear_y * self._cruise_scale))
            az = max(
                -self._max_angular,
                min(self._max_angular, msg.angular_z * self._cruise_scale),
            )
            if self._strafe_jl and abs(az) > 1e-6 and abs(lx) < 1e-6 and abs(ly) < 1e-6:
                sign = 1 if az > 0 else -1
                ly = sign * self._strafe_speed
                motion = twist_to_motion(0.0, ly, 0.0, sideways_steer_millirad=self._sideways_steer)
            else:
                motion = twist_to_motion(lx, ly, az, sideways_steer_millirad=self._sideways_steer)
            self._pending_v = motion.v_mm_s
            self._pending_omega = motion.omega_millirad_s
            self._pending_steer = motion.steer_millirad
            self._pending_model = motion.motion_model
        else:
            self._pending_v = msg.v_mm_s
            self._pending_omega = msg.omega_millirad_s
            self._pending_steer = msg.steer_millirad
            self._pending_model = msg.motion_model

        self._last_cmd_time = time.monotonic()
        if self._pending_v or self._pending_omega or self._pending_steer:
            self._last_motion_v = self._pending_v
            self._last_motion_omega = self._pending_omega
            self._last_motion_steer = self._pending_steer

    def _resolve_motion(self) -> tuple[int, int, int, int, bool]:
        now = time.monotonic()
        cmd_stale = (now - self._last_cmd_time) > self._cmd_timeout_s
        had_motion = (
            self._last_motion_v != 0
            or self._last_motion_omega != 0
            or self._last_motion_steer != 0
        )
        if had_motion and cmd_stale and now >= self._recover_until:
            self._recover_until = now + RECOVER_STABLE_S
            self._pending_v = 0
            self._pending_omega = 0
            self._pending_steer = 0
            self._pending_model = 0

        if now < self._recover_until:
            return 0, 0, 0, 0, False

        if cmd_stale or (
            self._pending_v == 0 and self._pending_omega == 0 and self._pending_steer == 0
        ):
            return 0, 0, 0, 0, False

        return (
            self._pending_v,
            self._pending_omega,
            self._pending_steer,
            self._pending_model,
            True,
        )

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
        self._jetson_link_up = self._bridge.link_connected

        v, omega, steer, model, allow = self._resolve_motion()
        cmd = V3Command()
        cmd.mode_req = self._pending_mode_req
        cmd.v_mm_s = v
        cmd.omega_millirad_s = omega
        cmd.steer_millirad = steer
        cmd.motion_model = model
        cmd.light_enable = self._light_enable
        cmd.light_mode = self._light_mode
        cmd.clear_error = self._pending_clear_error
        self._pending_clear_error = 0
        self._bridge.set_command(cmd)

        cache = self._bridge.blob_pub.cache
        vehicle = build_vehicle_data(
            cache,
            self._latest_status,
            self._latest_ext,
            last_status_time=self._last_status_time,
            status_timeout_s=self._status_timeout_s,
        )
        if vehicle is None:
            return

        feature = build_feature_status(
            cache,
            self._latest_status,
            jetson_link_up=self._jetson_link_up,
            last_status_time=self._last_status_time,
            status_timeout_s=self._status_timeout_s,
            time_sync_valid=self._offset_valid,
            time_sync_offset_ms=self._offset_ms,
        )
        if feature is None:
            return

        self._pub_vehicle.publish(vehicle)
        self._pub_feature.publish(feature)

        if vehicle.data_valid:
            self._integrate_odom(vehicle.linear_velocity_mm_s, vehicle.angular_velocity_millirad_s)

        stamp = vehicle.header.stamp
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = math.sin(self._odom_theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._odom_theta / 2.0)
        odom.twist.twist.linear.x = vehicle.linear_velocity_mm_s / 1000.0
        odom.twist.twist.angular.z = vehicle.angular_velocity_millirad_s / 1000.0
        self._pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = self._odom_x
        tf.transform.translation.y = self._odom_y
        tf.transform.rotation.z = math.sin(self._odom_theta / 2.0)
        tf.transform.rotation.w = math.cos(self._odom_theta / 2.0)
        self._tf.sendTransform(tf)

    def destroy_node(self) -> bool:
        self._bridge.shutdown()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AgvBaseEthBringeNode()
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
