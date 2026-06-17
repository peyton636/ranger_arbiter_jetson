#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np

from ds_imu_gps_localization.geo_utils import rot_to_quat
from ds_imu_gps_localization.imu_gps_localizer import ImuGpsLocalizer


class ImuGpsLocalizationNode(Node):
    def __init__(self):
        super().__init__("imu_gps_localization")
        self.declare_parameter("acc_noise", 1e-2)
        self.declare_parameter("gyro_noise", 1e-4)
        self.declare_parameter("acc_bias_noise", 1e-6)
        self.declare_parameter("gyro_bias_noise", 1e-8)
        self.declare_parameter("I_p_Gps_x", 0.0)
        self.declare_parameter("I_p_Gps_y", 0.0)
        self.declare_parameter("I_p_Gps_z", 0.0)
        self.declare_parameter("path_topic", "fused_path")
        self.declare_parameter("frame_id", "world")

        i_p_gps = np.array([
            self.get_parameter("I_p_Gps_x").value,
            self.get_parameter("I_p_Gps_y").value,
            self.get_parameter("I_p_Gps_z").value,
        ])
        self._localizer = ImuGpsLocalizer(
            acc_noise=self.get_parameter("acc_noise").value,
            gyro_noise=self.get_parameter("gyro_noise").value,
            acc_bias_noise=self.get_parameter("acc_bias_noise").value,
            gyro_bias_noise=self.get_parameter("gyro_bias_noise").value,
            i_p_gps=i_p_gps,
        )
        self._frame_id = self.get_parameter("frame_id").value
        path_topic = self.get_parameter("path_topic").value
        self._path = Path()
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._init_logged = False
        self._imu_count = 0
        self._gps_fix_count = 0
        self.create_subscription(Imu, "imu/data", self._imu_cb, 50)
        self.create_subscription(NavSatFix, "fix", self._gps_cb, 10)
        self.get_logger().info("IMU/GPS 融合节点已启动，等待 /imu/data 与 /fix")

    def _stamp_to_sec(self, stamp):
        return Time.from_msg(stamp).nanoseconds * 1e-9

    def _imu_cb(self, msg: Imu):
        imu_data = {
            "timestamp": self._stamp_to_sec(msg.header.stamp),
            "acc": np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]),
            "gyro": np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ]),
        }
        fused = self._localizer.add_imu_data(imu_data)
        if fused is None:
            if not self._localizer.initialized:
                self._imu_count += 1
                buf = len(self._localizer.imu_buffer)
                self.get_logger().warn(
                    f"融合未初始化：IMU 缓冲 {buf}/100，"
                    f"有效 GPS fix 收到 {self._gps_fix_count} 次 "
                    f"（需 status≥0 的 /fix，室内常无 fix）",
                    throttle_duration_sec=5.0,
                )
            return
        self._publish_path(fused)

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().info(
                f"收到 /fix 但 status={msg.status.status}（无定位），"
                "融合需室外有效 GPS fix",
                throttle_duration_sec=10.0,
            )
            return
        self._gps_fix_count += 1
        cov = np.diag([
            msg.position_covariance[0] if msg.position_covariance[0] > 0 else 1.0,
            msg.position_covariance[4] if msg.position_covariance[4] > 0 else 1.0,
            msg.position_covariance[8] if msg.position_covariance[8] > 0 else 1.0,
        ])
        gps_data = {
            "timestamp": self._stamp_to_sec(msg.header.stamp),
            "lla": np.array([msg.latitude, msg.longitude, msg.altitude]),
            "cov": cov,
        }
        ok = self._localizer.add_gps_data(gps_data)
        if ok and self._localizer.initialized and not self._init_logged:
            self._init_logged = True
            self.get_logger().info("IMU/GPS 融合系统已初始化")

    def _publish_path(self, state):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = float(state["G_p_I"][0])
        pose.pose.position.y = float(state["G_p_I"][1])
        pose.pose.position.z = float(state["G_p_I"][2])
        q = rot_to_quat(state["G_R_I"])
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])
        self._path.header = pose.header
        self._path.poses.append(pose)
        self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = ImuGpsLocalizationNode()
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
