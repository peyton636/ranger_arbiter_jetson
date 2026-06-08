#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
import serial

from ds_gps_driver.nmea_parser import check_nmea_checksum, parse_nmea_sentence

GPS_QUALITIES = {
    0: (9999.0, NavSatStatus.STATUS_NO_FIX, NavSatFix.COVARIANCE_TYPE_UNKNOWN),
    1: (4.0, NavSatStatus.STATUS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED),
    2: (2.0, NavSatStatus.STATUS_SBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED),
    4: (4.0, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED),
    5: (4.0, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED),
}


class GpsSerialNode(Node):
    def __init__(self):
        super().__init__("gps_serial_driver")
        self.declare_parameter("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("fix_topic", "fix")
        self.declare_parameter("vel_topic", "vel")

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        fix_topic = self.get_parameter("fix_topic").get_parameter_value().string_value
        vel_topic = self.get_parameter("vel_topic").get_parameter_value().string_value

        self._frame_id = frame_id
        self._fix_pub = self.create_publisher(NavSatFix, fix_topic, 10)
        self._vel_pub = self.create_publisher(TwistStamped, vel_topic, 10)

        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)
        except serial.SerialException as exc:
            self.get_logger().fatal(f"无法打开 GPS 串口 {port}: {exc}")
            raise

        self._rx_buf = b""
        self._last_status_log = self.get_clock().now()
        self.get_logger().info(f"GPS 串口已打开: {port} @ {baud}, 发布 /{fix_topic}")
        self.create_timer(0.02, self._poll_serial)

    def _poll_serial(self):
        try:
            waiting = self._ser.in_waiting
        except serial.SerialException as exc:
            self.get_logger().error(f"GPS 串口读取失败: {exc}")
            return
        if waiting <= 0:
            return
        self._rx_buf += self._ser.read(waiting)
        while True:
            idx = self._rx_buf.find(b"\n")
            if idx < 0:
                break
            raw_line = self._rx_buf[:idx].rstrip(b"\r")
            self._rx_buf = self._rx_buf[idx + 1 :]
            if raw_line:
                self._handle_sentence(raw_line)

    def _handle_sentence(self, line: bytes):
        if not check_nmea_checksum(line):
            return
        parsed = parse_nmea_sentence(line)
        if not parsed:
            return
        stamp = self.get_clock().now().to_msg()
        if "GGA" in parsed:
            self._publish_fix(parsed["GGA"], stamp)
        if "RMC" in parsed:
            self._publish_vel_rmc(parsed["RMC"], stamp)
        elif "VTG" in parsed:
            self._publish_vel_vtg(parsed["VTG"], stamp)

    def _publish_fix(self, data, stamp):
        fix_type = data.get("fix_type", 0)
        if fix_type not in GPS_QUALITIES:
            fix_type = 0
        default_epe, status, cov_type = GPS_QUALITIES[fix_type]

        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.status.status = status
        msg.position_covariance_type = cov_type

        lat = data.get("latitude", float("nan"))
        if data.get("latitude_direction") == "S":
            lat = -lat
        lon = data.get("longitude", float("nan"))
        if data.get("longitude_direction") == "W":
            lon = -lon
        alt = data.get("altitude", float("nan"))
        msl = data.get("mean_sea_level", float("nan"))
        if not math.isnan(alt) and not math.isnan(msl):
            alt = alt + msl

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        hdop = data.get("hdop", float("nan"))
        if math.isnan(hdop):
            hdop = default_epe
        msg.position_covariance[0] = hdop ** 2
        msg.position_covariance[4] = hdop ** 2
        msg.position_covariance[8] = (2.0 * hdop) ** 2
        self._fix_pub.publish(msg)
        self._log_fix_status(fix_type, data.get("num_satellites", 0), lat, lon)

    def _log_fix_status(self, fix_type, num_sats, lat, lon):
        now = self.get_clock().now()
        if (now - self._last_status_log).nanoseconds < 10_000_000_000:
            return
        self._last_status_log = now
        if fix_type == 0:
            self.get_logger().info(
                f"无定位（室内/无卫星）: fix_type=0, 卫星数={num_sats}, "
                "仍发布 /fix (status=NO_FIX, 经纬度=nan)"
            )
        else:
            self.get_logger().info(
                f"已定位: lat={lat:.7f}, lon={lon:.7f}, "
                f"fix_type={fix_type}, 卫星数={num_sats}"
            )

    def _publish_vel_rmc(self, data, stamp):
        if not data.get("fix_valid"):
            return
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x = data.get("speed", 0.0)
        msg.twist.angular.z = data.get("true_course", 0.0)
        self._vel_pub.publish(msg)

    def _publish_vel_vtg(self, data, stamp):
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x = data.get("speed", 0.0)
        course_deg = data.get("true_course", float("nan"))
        if not math.isnan(course_deg):
            msg.twist.angular.z = math.radians(course_deg)
        self._vel_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, "_ser") and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = GpsSerialNode()
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
