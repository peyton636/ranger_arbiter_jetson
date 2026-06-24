#!/usr/bin/env python3
"""���� GPS��/fix�������� eth_gateway ���Խڵ㣩��"""

from __future__ import annotations

import rclpy
from jetson_mcu_msgs.msg import GpsFrameA, GpsFrameB, GpsFrameC, TimeSyncResponse
from rclpy.node import Node

from gps_rs232_to_fix.gps_fix import GpsFixHelper


class GpsToFixNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_to_fix")

        self.declare_parameter("link_type", "eth")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("use_time_sync_stamp", True)

        link_type = self.get_parameter("link_type").get_parameter_value().string_value
        prefix = f"/jetson_{link_type}"
        fix_topic = self.get_parameter("fix_topic").get_parameter_value().string_value

        self._helper = GpsFixHelper(
            self,
            fix_topic=fix_topic,
            frame_id=self.get_parameter("frame_id").get_parameter_value().string_value,
            use_time_sync_stamp=self.get_parameter("use_time_sync_stamp")
            .get_parameter_value()
            .bool_value,
        )

        self.create_subscription(
            GpsFrameA, f"{prefix}/gps/a", self._helper.on_gps_frame_a, 10
        )
        self.create_subscription(
            GpsFrameB, f"{prefix}/gps/b", self._helper.on_gps_frame_b, 10
        )
        self.create_subscription(
            GpsFrameC, f"{prefix}/gps/c", self._helper.on_gps_frame_c, 10
        )
        if self._helper._use_time_sync_stamp:
            self.create_subscription(
                TimeSyncResponse, f"{prefix}/time_sync", self._helper.on_time_sync, 10
            )
        self.create_timer(10.0, self._helper.check_stale)
        self.create_timer(0.1, self._helper.try_publish)

        self.get_logger().info(f"gps_to_fix ����ģʽ: {prefix}/gps/* �� {fix_topic}")


def main() -> None:
    rclpy.init()
    node = GpsToFixNode()
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
