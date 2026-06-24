#!/usr/bin/env python3
# Publish /cmd_vel from a Tk slider GUI; query GPS/sensor/motion/light snapshots.

from __future__ import annotations

import os
import sys
import tkinter as tk
from tkinter import scrolledtext, ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scr_sensor.msg import AgvControl, AgvFeatureStatus, VehicleData
from sensor_msgs.msg import NavSatFix, NavSatStatus

from agv_base_driver.jetson_protocol import twist_to_motion

CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

SAFETY_NAMES = {
    1: "NORMAL",
    2: "SPEED_LIMIT",
    3: "DEGRADED",
    4: "EMERGENCY",
}

SAFETY_COLORS = {
    1: "#1a7f37",
    2: "#bf8700",
    3: "#bc4c00",
    4: "#cf222e",
}

MOTION_MODEL_NAMES = {
    0: "阿克曼",
    1: "斜移",
    2: "自旋",
    3: "驻车",
}

_MOTION_EPS_V_MM_S = 15
_MOTION_EPS_OMEGA_MRAD_S = 25
_MOTION_EPS_STEER_MRAD = 800


def _wheel_angle_list(wheel_angle) -> list[int]:
    if wheel_angle is None:
        return [0, 0, 0, 0]
    try:
        vals = [int(x) for x in wheel_angle]
    except TypeError:
        return [0, 0, 0, 0]
    return vals if vals else [0, 0, 0, 0]


def _motion_direction_label(
    v_mm_s: int,
    omega_millirad_s: int,
    steer_millirad: int,
    motion_model: int | None = None,
) -> str:
    """Human-readable motion: 停止/前进/后退/左移/右移/原地左转/原地右转."""
    if motion_model == 1 or (
        motion_model is None and abs(steer_millirad) >= _MOTION_EPS_STEER_MRAD
    ):
        if steer_millirad > 0:
            return "左移"
        if steer_millirad < 0:
            return "右移"
        if abs(v_mm_s) >= _MOTION_EPS_V_MM_S:
            return "斜移"
        return "斜移(待命)"
    if motion_model == 2 or (
        motion_model is None
        and abs(omega_millirad_s) >= _MOTION_EPS_OMEGA_MRAD_S
        and abs(v_mm_s) < _MOTION_EPS_V_MM_S
    ):
        if omega_millirad_s > 0:
            return "原地左转"
        if omega_millirad_s < 0:
            return "原地右转"
        return "自旋(待命)"
    if v_mm_s > _MOTION_EPS_V_MM_S:
        return "前进"
    if v_mm_s < -_MOTION_EPS_V_MM_S:
        return "后退"
    return "停止"

GPS_STATUS = {
    NavSatStatus.STATUS_NO_FIX: "NO_FIX",
    NavSatStatus.STATUS_FIX: "FIX",
    NavSatStatus.STATUS_SBAS_FIX: "SBAS_FIX",
    NavSatStatus.STATUS_GBAS_FIX: "GBAS_FIX",
}


def _decode_light_pack(pack: int) -> tuple[int, int]:
    return pack & 0x01, (pack >> 1) & 0x01


def _fmt_sonar_short(v: int | None, invalid: int = 65535) -> str:
    if v is None or v == invalid or v <= 0:
        return "---"
    return f"{v}"


def _link_state_note(link_state: int) -> str:
    bits = []
    if link_state & 0x01:
        bits.append("Jetson链路丢失")
    if link_state & 0x02:
        bits.append("CAN丢失")
    return "  ".join(bits) if bits else "链路正常"


def _fault_note(code: int) -> str:
    if code == 0:
        return "无"
    return f"0x{code:08X}"


SONAR_SOURCE_NAMES = {
    0: "MCU状态帧",
    1: "传感器BLOB",
}


class CmdVelGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_gui")

        self.declare_parameter("agv_control_topic", "/agv_control")
        self.declare_parameter("feature_status_topic", "/Function/FeatureStatusInfo")
        self.declare_parameter("vehicle_data_topic", "/Vehicle/VehicleData")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("max_linear_m_s", 0.8)
        self.declare_parameter("max_angular_rad_s", 1.0)
        self.declare_parameter("publish_hz", 20.0)

        agv_topic = (
            self.get_parameter("agv_control_topic").get_parameter_value().string_value
        )
        feature_topic = (
            self.get_parameter("feature_status_topic").get_parameter_value().string_value
        )
        vehicle_topic = (
            self.get_parameter("vehicle_data_topic").get_parameter_value().string_value
        )
        self._fix_topic = (
            self.get_parameter("fix_topic").get_parameter_value().string_value
        )
        self._max_linear = (
            self.get_parameter("max_linear_m_s").get_parameter_value().double_value
        )
        self._max_angular = (
            self.get_parameter("max_angular_rad_s").get_parameter_value().double_value
        )
        hz = self.get_parameter("publish_hz").get_parameter_value().double_value

        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0
        self._feature: AgvFeatureStatus | None = None
        self._vehicle: VehicleData | None = None
        self._fix: NavSatFix | None = None
        self._light_on = False

        self._pub = self.create_publisher(AgvControl, agv_topic, CMD_VEL_QOS)
        self.create_subscription(AgvFeatureStatus, feature_topic, self._feature_cb, 10)
        self.create_subscription(VehicleData, vehicle_topic, self._vehicle_cb, 10)
        self.create_subscription(NavSatFix, self._fix_topic, self._fix_cb, 10)
        period = 1.0 / hz if hz > 0 else 0.05
        self.create_timer(period, self._publish_control)
        # Sync explicit light-off at startup (MCU ignores light_info=0).
        self._light_boot_timer = self.create_timer(0.5, self._publish_light_off_once)

        self.get_logger().info(
            f"cmd_vel_gui -> {agv_topic}; status: {feature_topic}, {vehicle_topic}"
        )

    def _feature_cb(self, msg: AgvFeatureStatus) -> None:
        self._feature = msg

    def _vehicle_cb(self, msg: VehicleData) -> None:
        self._vehicle = msg

    def _fix_cb(self, msg: NavSatFix) -> None:
        self._fix = msg

    def set_cmd(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._linear_x = max(-self._max_linear, min(self._max_linear, linear_x))
        self._linear_y = max(-self._max_linear, min(self._max_linear, linear_y))
        self._angular_z = max(-self._max_angular, min(self._max_angular, angular_z))

    def stop(self) -> None:
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0
        self._publish_control()

    def set_light(self, on: bool) -> None:
        self._light_on = on
        self._publish_control()
        self.get_logger().info(f"light command: {'ON' if on else 'OFF'}")

    def _publish_light_off_once(self) -> None:
        self.set_light(False)
        self.destroy_timer(self._light_boot_timer)

    def light_status_text(self) -> str:
        cmd = "开" if self._light_on else "关"
        mcu = "未知"
        if self._feature is not None:
            mcu = "开" if self._feature.light_mode_feedback else "关"
        return f"指令 {cmd}  |  MCU {mcu}"

    def _publish_control(self) -> None:
        if not rclpy.ok():
            return
        msg = AgvControl()
        msg.mode_req = AgvControl.MODE_CAN
        msg.use_twist_input = True
        msg.linear_x = self._linear_x
        msg.linear_y = self._linear_y
        msg.angular_z = self._angular_z
        msg.light_enable = 1
        msg.light_mode = 1 if self._light_on else 0
        msg.clear_error = 0
        self._pub.publish(msg)

    def status_text(self) -> str:
        if self._vehicle is not None:
            v = self._vehicle
            return (
                f"电池 {v.battery_voltage_0p1v * 0.1:.1f} V  |  "
                f"电量 {v.battery_soc_percent}%"
            )
        return "电池：等待 VehicleData ..."

    def safety_status(self) -> tuple[str, str, str]:
        """Return (label, detail, tk foreground color)."""
        if self._feature is not None:
            f = self._feature
            name = SAFETY_NAMES.get(f.safety_state, f"UNK({f.safety_state})")
            color = SAFETY_COLORS.get(f.safety_state, "#57606a")
            link_note = _link_state_note(f.link_state)
            detail = f"限速 {f.limit_factor}%  |  {link_note}"
            return name, detail, color
        return "WAIT", "等待 FeatureStatusInfo ...", "#57606a"

    def command_motion_state(self) -> str:
        """Downlink intent from slider-derived motion."""
        motion = twist_to_motion(self._linear_x, self._linear_y, self._angular_z)
        mode_name = MOTION_MODEL_NAMES.get(motion.motion_model, str(motion.motion_model))
        direction = _motion_direction_label(
            motion.v_mm_s,
            motion.omega_millirad_s,
            motion.steer_millirad,
            motion.motion_model,
        )
        return (
            f"{direction}（{mode_name}）  "
            f"滑条 X={self._linear_x:.2f} Y={self._linear_y:.2f} Z={self._angular_z:.2f}"
        )

    def feedback_motion_state(self) -> str:
        """Chassis actual motion from VehicleData."""
        if self._vehicle is None:
            return "等待底盘反馈 ..."
        v = self._vehicle
        mode = self._feature.motion_model if self._feature else None
        direction = _motion_direction_label(
            v.linear_velocity_mm_s,
            v.angular_velocity_millirad_s,
            v.steer_millirad,
            mode,
        )
        v_m_s = v.linear_velocity_mm_s / 1000.0
        w_rad_s = v.angular_velocity_millirad_s / 1000.0
        mode_name = MOTION_MODEL_NAMES.get(mode, str(mode)) if mode is not None else "?"
        return f"{direction}（{mode_name}）  线速 {v_m_s:.2f} m/s  角速 {w_rad_s:.2f} rad/s"

    def sonar_distances(self) -> dict[str, str]:
        """Front / back / left / right sonar text for cross panel."""
        inv = VehicleData.SONAR_INVALID
        if self._vehicle is not None:
            v = self._vehicle
            return {
                "front": _fmt_sonar_short(v.sonar_front_mm, inv),
                "back": _fmt_sonar_short(v.sonar_back_mm, inv),
                "left": _fmt_sonar_short(v.sonar_left_mm, inv),
                "right": _fmt_sonar_short(v.sonar_right_mm, inv),
            }
        return {"front": "---", "back": "---", "left": "---", "right": "---"}

    def gps_snapshot(self) -> str:
        if self._fix is None:
            return "【GPS】\n暂无数据，请检查 GPS 节点与 /fix 话题。"
        f = self._fix
        st = GPS_STATUS.get(f.status.status, "未知")
        st_cn = {
            "NO_FIX": "未定位",
            "FIX": "已定位",
            "SBAS_FIX": "差分定位",
            "GBAS_FIX": "精密定位",
        }.get(st, st)
        return "\n".join(
            [
                "【GPS 定位】",
                f"状态：{st_cn}",
                f"经度：{f.longitude:.6f}°",
                f"纬度：{f.latitude:.6f}°",
                f"海拔：{f.altitude:.1f} m",
            ]
        )

    def sensor_snapshot(self) -> str:
        lines = [
            "【传感器 / 电源】",
            "四向超声见上方状态栏右侧，此处不重复显示。",
            "",
        ]
        if self._vehicle is not None:
            v = self._vehicle
            lines.append(
                f"电池：{v.battery_voltage_0p1v * 0.1:.1f} V  |  "
                f"电量 {v.battery_soc_percent}%  SOH {v.bms_soh_percent}%"
            )
        else:
            lines.append("电池：暂无 VehicleData")

        if self._vehicle is not None and self._vehicle.sonar_time_sync_valid:
            lines.extend(["", "时间同步：已对齐", f"超声来源：{self._vehicle.sonar_source}"])
        else:
            lines.extend(["", "时间同步：暂无"])

        return "\n".join(lines)

    def motion_snapshot(self) -> str:
        lines = [
            "【底盘运动详情】",
            "安全状态与运动方向见上方状态栏。",
            "",
        ]
        if self._feature is not None:
            f = self._feature
            lines.append(f"故障码：{_fault_note(f.fault_code)}")
            lines.append(f"Jetson seq：{f.jetson_seq}  链路：{'UP' if f.jetson_link_up else 'DOWN'}")
        if self._vehicle is not None:
            v = self._vehicle
            lines.append(
                f"轮速 RF/RR/LR/LF：{v.wheel_rf}/{v.wheel_rr}/{v.wheel_lr}/{v.wheel_lf}"
            )
        return "\n".join(lines)

    def light_snapshot(self) -> str:
        gui = "开" if self._light_on else "关"
        mcu = "未知"
        if self._feature is not None:
            mcu = "开" if self._feature.light_mode_feedback else "关"
        return "\n".join(
            [
                "【灯光】",
                f"界面按钮：{gui}",
                f"MCU 回馈：{mcu}",
            ]
        )


class CmdVelGuiApp:
    def __init__(self, node: CmdVelGuiNode) -> None:
        self._node = node
        self._root = tk.Tk()
        self._root.title("AGV cmd_vel teleop + data query")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        pad = {"padx": 8, "pady": 6}
        frm = ttk.Frame(self._root, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")

        status_frm = ttk.LabelFrame(frm, text="底盘状态（实时）", padding=8)
        status_frm.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 8))
        status_frm.columnconfigure(0, weight=1)

        left_frm = ttk.Frame(status_frm)
        left_frm.grid(row=0, column=0, sticky="nw")

        ttk.Label(left_frm, text="Safety:").grid(row=0, column=0, sticky="w", padx=4)
        self._safety_label = tk.Label(
            left_frm,
            text="WAIT",
            font=("", 14, "bold"),
            fg="#57606a",
            anchor="w",
        )
        self._safety_label.grid(row=0, column=1, sticky="w", padx=4)
        self._safety_detail = ttk.Label(left_frm, text="等待 v3_status ...")
        self._safety_detail.grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=(2, 6))

        ttk.Label(left_frm, text="指令运动:").grid(row=2, column=0, sticky="nw", padx=4)
        self._cmd_motion_label = ttk.Label(
            left_frm, text=node.command_motion_state(), wraplength=320
        )
        self._cmd_motion_label.grid(row=2, column=1, sticky="w", padx=4)

        ttk.Label(left_frm, text="实际运动:").grid(
            row=3, column=0, sticky="nw", padx=4, pady=(4, 0)
        )
        self._fb_motion_label = ttk.Label(
            left_frm, text=node.feedback_motion_state(), wraplength=320
        )
        self._fb_motion_label.grid(row=3, column=1, sticky="w", padx=4, pady=(4, 0))

        sonar_frm = ttk.LabelFrame(status_frm, text="超声 mm", padding=6)
        sonar_frm.grid(row=0, column=1, sticky="ne", padx=(20, 4))
        sonar_frm.columnconfigure(0, weight=1)
        sonar_frm.columnconfigure(1, weight=1)
        sonar_frm.columnconfigure(2, weight=1)

        def _sonar_lbl(parent, row: int, col: int, title: str) -> tk.Label:
            box = ttk.Frame(parent, padding=4)
            box.grid(row=row, column=col, padx=6, pady=2)
            ttk.Label(box, text=title, font=("", 9)).pack()
            val = tk.Label(box, text="---", font=("TkFixedFont", 13, "bold"))
            val.pack()
            return val

        self._sonar_front = _sonar_lbl(sonar_frm, 0, 1, "前 ↑")
        self._sonar_left = _sonar_lbl(sonar_frm, 1, 0, "左 ←")
        self._sonar_right = _sonar_lbl(sonar_frm, 1, 2, "右 →")
        self._sonar_back = _sonar_lbl(sonar_frm, 2, 1, "后 ↓")

        ttk.Label(frm, text="前后 X (阿克曼 0x141=0, v± steer0) m/s").grid(
            row=1, column=0, sticky="w", **pad
        )
        self._scale_x = self._make_scale(frm, 2, node._max_linear)

        ttk.Label(frm, text="左右 Y (斜移 model=1, 左 steer+1570 右 steer-1570) m/s").grid(
            row=3, column=0, sticky="w", **pad
        )
        self._scale_y = self._make_scale(frm, 4, node._max_linear)

        ttk.Label(frm, text="原地转 Z (自旋 0x141=2, omega±) rad/s").grid(
            row=5, column=0, sticky="w", **pad
        )
        self._scale_w = self._make_scale(frm, 6, node._max_angular)

        btn_frm = ttk.Frame(frm)
        btn_frm.grid(row=7, column=0, columnspan=2, **pad)
        ttk.Button(btn_frm, text="STOP / zero", command=self._emergency_stop).pack(
            side=tk.LEFT, padx=4
        )

        light_frm = ttk.Frame(frm)
        light_frm.grid(row=8, column=0, columnspan=2, padx=8, pady=4, sticky="w")
        ttk.Label(light_frm, text="灯光:").pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(light_frm, text="ON", command=self._light_on).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(light_frm, text="OFF", command=self._light_off).pack(
            side=tk.LEFT, padx=3
        )
        self._light_label = ttk.Label(light_frm, text=node.light_status_text())
        self._light_label.pack(side=tk.LEFT, padx=8)

        self._cmd_label = ttk.Label(frm, text="cmd: v=(0,0) w=0")
        self._cmd_label.grid(row=9, column=0, columnspan=2, sticky="w", **pad)

        self._fb_label = ttk.Label(frm, text=node.status_text(), wraplength=520)
        self._fb_label.grid(row=10, column=0, columnspan=2, sticky="w", **pad)

        ttk.Separator(frm, orient=tk.HORIZONTAL).grid(
            row=11, column=0, columnspan=2, sticky="ew", pady=8
        )
        ttk.Label(frm, text="数据查询（点击查看详情，与上方状态不重复）:").grid(
            row=12, column=0, columnspan=2, sticky="w", padx=8
        )

        query_frm = ttk.Frame(frm)
        query_frm.grid(row=13, column=0, columnspan=2, padx=8, pady=4)
        ttk.Button(query_frm, text="GPS", command=self._show_gps).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(query_frm, text="Sensor", command=self._show_sensor).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(query_frm, text="Motion", command=self._show_motion).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(query_frm, text="Light", command=self._show_light).pack(
            side=tk.LEFT, padx=3
        )

        self._data_text = scrolledtext.ScrolledText(
            frm, width=72, height=14, font=("TkFixedFont", 10), wrap=tk.WORD
        )
        self._data_text.grid(row=14, column=0, columnspan=2, padx=8, pady=6)
        self._data_text.config(state=tk.DISABLED)
        self._set_data_text("点击 GPS / Sensor / Motion / Light 查看对应详情。")

        ttk.Label(
            frm,
            text="Y 斜移：model=1 v>0 左steer+1570 右-1570；400ms 后 RUN；轮角0 查 MCU CMDOUT",
            font=("", 9),
        ).grid(row=15, column=0, columnspan=2, sticky="w", **pad)

    def _set_data_text(self, text: str) -> None:
        self._data_text.config(state=tk.NORMAL)
        self._data_text.delete("1.0", tk.END)
        self._data_text.insert(tk.END, text)
        self._data_text.config(state=tk.DISABLED)

    def _show_gps(self) -> None:
        self._set_data_text(self._node.gps_snapshot())

    def _show_sensor(self) -> None:
        self._set_data_text(self._node.sensor_snapshot())

    def _show_motion(self) -> None:
        self._set_data_text(self._node.motion_snapshot())

    def _show_light(self) -> None:
        self._set_data_text(self._node.light_snapshot())

    def _light_on(self) -> None:
        self._node.set_light(True)
        self._light_label.config(text=self._node.light_status_text())

    def _light_off(self) -> None:
        self._node.set_light(False)
        self._light_label.config(text=self._node.light_status_text())

    def _make_scale(self, parent: ttk.Frame, row: int, limit: float) -> tk.Scale:
        scale = tk.Scale(
            parent,
            from_=-limit,
            to=limit,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=360,
            command=lambda _v: self._on_slider(),
        )
        scale.set(0.0)
        scale.grid(row=row, column=0, **{"padx": 8, "pady": 4})
        return scale

    def _refresh_status_panel(self) -> None:
        name, detail, color = self._node.safety_status()
        self._safety_label.config(text=name, fg=color)
        self._safety_detail.config(text=detail)
        self._cmd_motion_label.config(text=self._node.command_motion_state())
        self._fb_motion_label.config(text=self._node.feedback_motion_state())
        d = self._node.sonar_distances()
        self._sonar_front.config(text=d["front"])
        self._sonar_back.config(text=d["back"])
        self._sonar_left.config(text=d["left"])
        self._sonar_right.config(text=d["right"])

    def _on_slider(self) -> None:
        lx = float(self._scale_x.get())
        ly = float(self._scale_y.get())
        az = float(self._scale_w.get())
        eps = 0.02
        # 单轴优先：避免 X 未归零时误以为在横移
        if abs(ly) > eps:
            if lx != 0.0:
                lx = 0.0
                self._scale_x.set(0.0)
            if az != 0.0:
                az = 0.0
                self._scale_w.set(0.0)
        elif abs(az) > eps:
            if lx != 0.0:
                lx = 0.0
                self._scale_x.set(0.0)
            if ly != 0.0:
                ly = 0.0
                self._scale_y.set(0.0)
        elif abs(lx) > eps:
            if ly != 0.0:
                ly = 0.0
                self._scale_y.set(0.0)
            if az != 0.0:
                az = 0.0
                self._scale_w.set(0.0)
        self._node.set_cmd(lx, ly, az)
        self._cmd_label.config(
            text=f"cmd: v=({lx:.2f}, {ly:.2f}) m/s  w={az:.2f} rad/s"
        )
        self._refresh_status_panel()

    def _emergency_stop(self) -> None:
        self._scale_x.set(0.0)
        self._scale_y.set(0.0)
        self._scale_w.set(0.0)
        self._node.stop()
        self._cmd_label.config(text="cmd: v=(0,0) w=0 [STOP]")
        self._refresh_status_panel()

    def _tick(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0)
            self._refresh_status_panel()
            self._fb_label.config(text=self._node.status_text())
            self._light_label.config(text=self._node.light_status_text())
            self._root.after(50, self._tick)

    def _on_close(self) -> None:
        self._emergency_stop()
        self._root.destroy()

    def run(self) -> None:
        self._tick()
        self._root.mainloop()


def _ensure_display() -> bool:
    if os.environ.get("DISPLAY"):
        return True
    # Local desktop session often uses :0 but SSH/launch may omit DISPLAY.
    if os.path.isdir("/tmp/.X11-unix"):
        for name in sorted(os.listdir("/tmp/.X11-unix")):
            if name.startswith("X") and name[1:].isdigit():
                os.environ["DISPLAY"] = f":{name[1:]}"
                return True
    return False


def main() -> None:
    if not _ensure_display():
        print(
            "cmd_vel_gui requires DISPLAY (graphical desktop).\n"
            "  On Jetson monitor: export DISPLAY=:0\n"
            "  Remote SSH: ssh -X user@jetson\n"
            "  Or run bringup without GUI:\n"
            "    ros2 launch agv_base_driver jetson_eth_bringup.launch.py ...",
            file=sys.stderr,
        )
        sys.exit(1)

    rclpy.init()
    node = CmdVelGuiNode()
    try:
        app = CmdVelGuiApp(node)
    except tk.TclError as exc:
        print(
            f"cmd_vel_gui failed to open window (DISPLAY={os.environ.get('DISPLAY')}): {exc}\n"
            "  Try: export DISPLAY=:0\n"
            "  If SSH: ssh -X user@jetson",
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
