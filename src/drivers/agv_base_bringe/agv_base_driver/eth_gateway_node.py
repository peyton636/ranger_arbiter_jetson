#!/usr/bin/env python3
"""仅 MCU 以太网链路（调试）；生产请用 agv_base_driver 单节点。"""

from __future__ import annotations

import rclpy
from jetson_mcu_msgs.msg import V3Command
from rclpy.node import Node

from agv_base_driver.mcu_eth_bridge import McuEthBridge

_TOPIC_PREFIX = "/jetson_eth"


class EthGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("eth_gateway")
        McuEthBridge.declare_parameters(self)
        flags = McuEthBridge.read_ros_publish_flags(self)
        self._bridge = McuEthBridge(self, _TOPIC_PREFIX, **flags)
        self.create_subscription(
            V3Command, f"{_TOPIC_PREFIX}/command", self._command_cb, 10
        )
        self.get_logger().info(
            f"eth_gateway 调试模式：订阅 {_TOPIC_PREFIX}/command"
        )

    def _command_cb(self, msg: V3Command) -> None:
        self._bridge.set_command(msg)

    def destroy_node(self) -> bool:
        self._bridge.shutdown()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = EthGatewayNode()
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
