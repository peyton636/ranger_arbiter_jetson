#!/usr/bin/env python3
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial


def _checksum(data):
    return (sum(data[:10]) & 0xFF) == data[10]


def _hex_to_short(raw):
    return struct.unpack("hhhh", bytearray(raw))


def _euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class WitImuNode(Node):
    def __init__(self):
        super().__init__("wit_imu")
        self.declare_parameter("port", "/dev/imu_usb")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("mag_topic", "imu/mag")

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        mag_topic = self.get_parameter("mag_topic").get_parameter_value().string_value

        self._imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self._mag_pub = self.create_publisher(MagneticField, mag_topic, 10)

        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)
            self._ser.reset_input_buffer()
        except serial.SerialException as exc:
            self.get_logger().fatal(f"无法打开 IMU 串口 {port}: {exc}")
            raise

        self._rx_buf = bytearray()
        self._angular_velocity = [0.0, 0.0, 0.0]
        self._acceleration = [0.0, 0.0, 0.0]
        self._magnetometer = [0, 0, 0]
        self._angle_degree = [0.0, 0.0, 0.0]

        self.get_logger().info(f"IMU 串口已打开: {port} @ {baud}, 发布 /{imu_topic}")
        self.create_timer(0.01, self._poll_serial)

    def _poll_serial(self):
        try:
            waiting = self._ser.in_waiting
        except serial.SerialException as exc:
            self.get_logger().error(f"IMU 串口读取失败: {exc}")
            return
        if waiting <= 0:
            return
        self._rx_buf += self._ser.read(waiting)
        while len(self._rx_buf) >= 11:
            if self._rx_buf[0] != 0x55:
                del self._rx_buf[0]
                continue
            frame = self._rx_buf[:11]
            if not _checksum(frame):
                del self._rx_buf[0]
                continue
            self._handle_frame(list(frame))
            del self._rx_buf[:11]

    def _handle_frame(self, data):
        if data[1] == 0x51:
            self._acceleration = [
                _hex_to_short(data[2:10])[i] / 32768.0 * 16.0 * 9.8 for i in range(3)
            ]
        elif data[1] == 0x52:
            self._angular_velocity = [
                _hex_to_short(data[2:10])[i] / 32768.0 * 2000.0 * math.pi / 180.0
                for i in range(3)
            ]
        elif data[1] == 0x53:
            self._angle_degree = [
                _hex_to_short(data[2:10])[i] / 32768.0 * 180.0 for i in range(3)
            ]
            self._publish_imu()
        elif data[1] == 0x54:
            self._magnetometer = list(_hex_to_short(data[2:10]))

    def _publish_imu(self):
        stamp = self.get_clock().now().to_msg()
        roll, pitch, yaw = [d * math.pi / 180.0 for d in self._angle_degree]
        qx, qy, qz, qw = _euler_to_quaternion(roll, pitch, yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self._frame_id
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.angular_velocity.x = self._angular_velocity[0]
        imu_msg.angular_velocity.y = self._angular_velocity[1]
        imu_msg.angular_velocity.z = self._angular_velocity[2]
        imu_msg.linear_acceleration.x = self._acceleration[0]
        imu_msg.linear_acceleration.y = self._acceleration[1]
        imu_msg.linear_acceleration.z = self._acceleration[2]
        self._imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self._frame_id
        mag_msg.magnetic_field.x = float(self._magnetometer[0])
        mag_msg.magnetic_field.y = float(self._magnetometer[1])
        mag_msg.magnetic_field.z = float(self._magnetometer[2])
        self._mag_pub.publish(mag_msg)

    def destroy_node(self):
        if hasattr(self, "_ser") and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = WitImuNode()
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
