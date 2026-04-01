import struct
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import SPI

"""Tkinter ROS2 monitor for SPI packets published on /spi_receive.

Address map:
- 0/1: status packets
- 2/3: sensor packets
"""


STATUS_ADDRESS_1 = 0
STATUS_ADDRESS_2 = 1
SENSOR_ADDRESS_1 = 2
SENSOR_ADDRESS_2 = 3

STATUS_NAMES = (
    "i2c_bus_status_1",
    "spi_bus_status_1",
    "pwm_status",
    "imu1_status",
    "imu2_status",
    "bar30_status",
    "i2c_bus_status_2",
    "spi_bus_status_2",
    "ADC_status",
    "leak1_status",
    "leak2_status",
    "batt1_status",
    "batt2_status",
    "temp1_status",
    "temp2_status",
)


ERROR_LABELS = {
    0: "UNINITIALIZED",
    1: "OK",
    2: "ERROR",
    3: "LEAK_DETECTED",
    4: "CRC_FAILED",
    5: "CONFIG_ERR",
    6: "TIMEOUT",
    99: "UNKNOWN",
}


class SpiMonitor(Node):
    """ROS2 node that renders incoming SPI packet data in a small Tk UI."""

    def __init__(self) -> None:
        """Create the ROS2 node, initialize UI state, and start polling loop."""
        super().__init__("spi_monitor")

        self._root = tk.Tk()
        self._root.title("SPI Sensor Monitor")
        self._root.geometry("520x460")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._closed = False

        self._meta_var = tk.StringVar(value="Waiting for /spi_receive...")
        self._status_vars = {name: tk.StringVar(value="--") for name in STATUS_NAMES}
        # Sensor section variables (text shown in the UI).
        self._sensor_vars = {
            "imu1_accel": tk.StringVar(value="--"),
            "imu1_gyro": tk.StringVar(value="--"),
            "imu1_mag": tk.StringVar(value="--"),
            "imu1_temp": tk.StringVar(value="--"),
            "imu2_accel": tk.StringVar(value="--"),
            "imu2_gyro": tk.StringVar(value="--"),
            "imu2_mag": tk.StringVar(value="--"),
            "imu2_temp": tk.StringVar(value="--"),
            "bar30_pressure": tk.StringVar(value="--"),
            "bar30_temp": tk.StringVar(value="--"),
            "leak1_status": tk.StringVar(value="--"),
            "leak2_status": tk.StringVar(value="--"),
            "batt1_status": tk.StringVar(value="--"),
            "batt2_status": tk.StringVar(value="--"),
            "batt_temp1_status": tk.StringVar(value="--"),
            "batt_temp2_status": tk.StringVar(value="--"),
        }

        self._build_ui()

        self.create_subscription(SPI, "/spi_receive", self._handle_spi, 10)
        self._root.after(20, self._pump)

    def _build_ui(self) -> None:
        """Build and place all Tk widgets for status + sensor display."""
        frame = ttk.Frame(self._root, padding=12)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="SPI Receive Monitor", font=("TkDefaultFont", 14, "bold")).pack(anchor=tk.W)
        ttk.Label(frame, textvariable=self._meta_var).pack(anchor=tk.W, pady=(4, 10))

        status_box = ttk.LabelFrame(frame, text="System Status", padding=10)
        status_box.pack(fill=tk.X, pady=(0, 10))
        for row, name in enumerate(STATUS_NAMES):
            ttk.Label(status_box, text=name).grid(row=row, column=0, sticky=tk.W, padx=(0, 16), pady=2)
            ttk.Label(status_box, textvariable=self._status_vars[name]).grid(row=row, column=1, sticky=tk.W, pady=2)

        sensor_box = ttk.LabelFrame(frame, text="Sensor Data", padding=10)
        sensor_box.pack(fill=tk.BOTH, expand=True)
        labels = (
            ("IMU1 accel", "imu1_accel"),
            ("IMU1 gyro", "imu1_gyro"),
            ("IMU1 mag", "imu1_mag"),
            ("IMU1 temp", "imu1_temp"),
            ("IMU2 accel", "imu2_accel"),
            ("IMU2 gyro", "imu2_gyro"),
            ("IMU2 mag", "imu2_mag"),
            ("IMU2 temp", "imu2_temp"),
            ("BAR30 pressure", "bar30_pressure"),
            ("BAR30 temp", "bar30_temp"),
            ("Leak Detector 1", "leak1_status"),
            ("Leak Detector 2", "leak2_status"),
            ("Battery 1 Voltage", "batt1_status"),
            ("Battery 2 Voltage", "batt2_status"),
            ("Battery Temp 1", "batt_temp1_status"),
            ("Battery Temp 2", "batt_temp2_status"),
        )
        for row, (label, key) in enumerate(labels):
            ttk.Label(sensor_box, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 16), pady=2)
            ttk.Label(sensor_box, textvariable=self._sensor_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=2)

    def _handle_spi(self, msg: SPI) -> None:
        """Route each SPI packet to the decoder that matches its address."""
        self._meta_var.set(
            f"Last packet: address={msg.address} size={msg.size} crc=0x{msg.crc:04X}"
        )

        payload = bytes(msg.data[:msg.size])

        if msg.address == STATUS_ADDRESS_1:
            self._update_status_1(payload)
        elif msg.address == SENSOR_ADDRESS_1:
            self._update_sensor_1(payload)
        elif msg.address == STATUS_ADDRESS_2:
            self._update_status_2(payload)
        elif msg.address == SENSOR_ADDRESS_2:
            self._update_sensor_2(payload)
        else:
            self.get_logger().warning(
                f"Unhandled SPI address {msg.address} with payload size {msg.size}"
            )

    def _update_status_1(self, payload: bytes) -> None:
        """Decode status packet from address 0.

        Layout: <6i (six 32-bit signed integers).
        """
        values = struct.unpack("<6i", payload)
        for name, value in zip(STATUS_NAMES[0:6], values):
            label = ERROR_LABELS.get(value, str(value))
            self._status_vars[name].set(f"{value} ({label})")

    def _update_status_2(self, payload: bytes) -> None:
            """Decode status packet from address 1.

            Layout: <9i (same layout as status_1).
            """
            values = struct.unpack("<9i", payload)
            for name, value in zip(STATUS_NAMES[6:15], values):
                label = ERROR_LABELS.get(value, str(value))
                self._status_vars[name].set(f"{value} ({label})")

    def _update_sensor_1(self, payload: bytes) -> None:
        """Decode primary sensor packet (IMU1 + IMU2 + BAR30).

        Layout: <20h2f
        - 20 int16 values: two IMUs, each [accel(3), gyro(3), mag(3), temp(1)]
        - 2 float32 values: [bar30_pressure, bar30_temp]
        """
        values = struct.unpack("<20h2f", payload)

        imu1 = values[0:10]
        imu2 = values[10:20]
        bar30_pressure, bar30_temp = values[20:22]

        self._sensor_vars["imu1_accel"].set(self._fmt_vec3(imu1[0:3]))
        self._sensor_vars["imu1_gyro"].set(self._fmt_vec3(imu1[3:6]))
        self._sensor_vars["imu1_mag"].set(self._fmt_vec3(imu1[6:9]))
        self._sensor_vars["imu1_temp"].set(str(imu1[9]))

        self._sensor_vars["imu2_accel"].set(self._fmt_vec3(imu2[0:3]))
        self._sensor_vars["imu2_gyro"].set(self._fmt_vec3(imu2[3:6]))
        self._sensor_vars["imu2_mag"].set(self._fmt_vec3(imu2[6:9]))
        self._sensor_vars["imu2_temp"].set(str(imu2[9]))

        self._sensor_vars["bar30_pressure"].set(f"{bar30_pressure:.3f}")
        self._sensor_vars["bar30_temp"].set(f"{bar30_temp:.3f}")

    def _update_sensor_2(self, payload: bytes) -> None:
        """Decode secondary sensor packet (leaks, battery voltages, battery temps)."""
        values = struct.unpack("<2f4i", payload)

        leak1, leak2, batt1, batt2 = values[2:6]
        batt_temp1, batt_temp2 = values[0:2]

        self._sensor_vars["batt_temp1_status"].set(f"{batt_temp1} °C")
        self._sensor_vars["batt_temp2_status"].set(f"{batt_temp2} °C")
        self._sensor_vars["leak1_status"].set(f"{leak1} mV")
        self._sensor_vars["leak2_status"].set(f"{leak2} mV")
        self._sensor_vars["batt1_status"].set(f"{batt1} mV")
        self._sensor_vars["batt2_status"].set(f"{batt2} mV")

    @staticmethod
    def _fmt_vec3(values: tuple[int, int, int] | list[int]) -> str:
        """Format 3-axis vector values for compact UI display."""
        return f"x={values[0]} y={values[1]} z={values[2]}"

    def _pump(self) -> None:
        """Run one non-blocking ROS spin cycle, then reschedule this method."""
        if self._closed:
            return

        rclpy.spin_once(self, timeout_sec=0.0)
        self._root.after(20, self._pump)

    def _on_close(self) -> None:
        """Handle window close event and stop further UI/ROS polling."""
        self._closed = True
        self._root.destroy()

    def run(self) -> None:
        """Start Tk event loop (blocking until window closes)."""
        self._root.mainloop()


def main(args=None) -> None:
    """Entry point for the SPI monitor node."""
    rclpy.init(args=args)
    node = SpiMonitor()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
