#!/usr/bin/env python3
"""ROS2 port of gps_goal: convert lat/lon to map-frame navigation goal."""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

try:
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False


def dms_to_decimal(text):
    text = str(text).strip()
    if "," not in text:
        return float(text)
    parts = [float(x) for x in text.split(",")]
    sign = -1.0 if text.startswith("-") else 1.0
    deg, minutes, seconds = parts[0], parts[1], parts[2]
    if sign < 0:
        minutes = -abs(minutes)
        seconds = -abs(seconds)
    return deg + minutes / 60.0 + seconds / 3600.0


def calc_goal_xy(origin_lat, origin_lon, goal_lat, goal_lon):
    g = Geodesic.WGS84.Inverse(origin_lat, origin_lon, goal_lat, goal_lon)
    distance = g["s12"]
    azimuth = math.radians(g["azi1"])
    x = math.cos(azimuth) * distance
    y = math.sin(azimuth) * distance
    return x, y, distance, math.degrees(g["azi1"])


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class GpsGoalNode(Node):
    def __init__(self):
        super().__init__("gps_goal")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("use_nav2", True)
        self.declare_parameter("nav2_action", "navigate_to_pose")
        self.declare_parameter("goal_pose_topic", "goal_pose")
        self.declare_parameter("wait_for_origin", True)

        self._frame_id = self.get_parameter("frame_id").value
        use_nav2 = self.get_parameter("use_nav2").value
        if isinstance(use_nav2, str):
            use_nav2 = use_nav2.lower() in ("true", "1", "yes")
        self._use_nav2 = bool(use_nav2) and HAS_NAV2
        self._origin_lat = None
        self._origin_lon = None

        self.create_subscription(PoseStamped, "local_xy_origin", self._origin_cb, 10)
        self.create_subscription(PoseStamped, "gps_goal_pose", self._goal_pose_cb, 10)
        self.create_subscription(NavSatFix, "gps_goal_fix", self._goal_fix_cb, 10)

        if self._use_nav2:
            action_name = self.get_parameter("nav2_action").value
            self._nav_client = ActionClient(self, NavigateToPose, action_name)
            self.get_logger().info(f"等待 Nav2 action: {action_name}")
        else:
            topic = self.get_parameter("goal_pose_topic").value
            self._goal_pub = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Nav2 不可用，发布目标到 /{topic}")

        if self.get_parameter("wait_for_origin").value:
            self.get_logger().info("等待 /local_xy_origin 设置地图原点经纬度...")

    def _origin_cb(self, msg: PoseStamped):
        self._origin_lat = msg.pose.position.x
        self._origin_lon = msg.pose.position.y
        self.get_logger().info(
            f"收到原点: lat={self._origin_lat:.6f}, lon={self._origin_lon:.6f}"
        )

    def _goal_pose_cb(self, msg: PoseStamped):
        lat = msg.pose.position.y
        lon = msg.pose.position.x
        z = msg.pose.position.z
        q = msg.pose.orientation
        roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
        self.do_gps_goal(lat, lon, z=z, roll=roll, pitch=pitch, yaw=yaw)

    def _goal_fix_cb(self, msg: NavSatFix):
        self.do_gps_goal(msg.latitude, msg.longitude)

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def do_gps_goal(self, lat, lon, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        if self._origin_lat is None or self._origin_lon is None:
            self.get_logger().error("尚未收到 /local_xy_origin，无法计算目标点")
            return
        lat = float(lat)
        lon = float(lon)
        x, y, dist, az = calc_goal_xy(self._origin_lat, self._origin_lon, lat, lon)
        self.get_logger().info(
            f"GPS 目标 lat={lat:.6f} lon={lon:.6f} -> map(x,y)=({x:.3f},{y:.3f}) "
            f"距离={dist:.3f}m 方位角={az:.1f}°"
        )
        self._send_goal(x, y, z, roll, pitch, yaw)

    def _send_goal(self, x, y, z, roll, pitch, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        if self._use_nav2:
            if not self._nav_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().warn("Nav2 navigate_to_pose 未就绪")
                return
            goal = NavigateToPose.Goal()
            goal.pose = pose
            send_future = self._nav_client.send_goal_async(goal)
            send_future.add_done_callback(self._nav_goal_response_cb)
        else:
            self._goal_pub.publish(pose)
            self.get_logger().info(f"已发布 PoseStamped 到 goal_pose")

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝了导航目标")
            return
        self.get_logger().info("Nav2 已接受目标，导航中...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Nav2 导航结束，结果码: {result}")


def main():
    rclpy.init()
    node = GpsGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
