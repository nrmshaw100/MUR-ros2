import time
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import SPI

GET_STATUS = 0x00
GET_DATA = 0x01

class MonitorSystem(Node):
    """ROS2 node that periodically polls for new data from the SPI monitor and updates the UI."""

    def __init__(self) -> None:
        """Create the ROS2 node and start polling loop."""
        super().__init__("monitor_system")
        self.sys_controller_pub_ = self.create_publisher(SPI, "/spi_controller", 10)
        self.sys_monitor_pub_ = self.create_publisher(SPI, "/spi_monitor", 10)
        self.timer = self.create_timer(0.2, self._poll_spi_monitor)
        self.address_val = GET_DATA

    def _poll_spi_monitor(self) -> None:
        """Trigger an update of the SPI monitor UI."""
        self.get_logger().debug("Polling SPI monitor for new data...")
        msg = SPI()
        msg.synch = 0x55
        msg.syncl = 0x55
        msg.address = self.address_val  
        msg.size = 0
        msg.crc = 0
        self.sys_monitor_pub_.publish(msg)
        time.sleep(0.05) 
        self.sys_controller_pub_.publish(msg)

        if self.address_val == GET_STATUS:
            self.address_val = GET_DATA
        else:
            self.address_val = GET_STATUS

def main(args=None):
    rclpy.init(args=args)
    monitor_system = MonitorSystem()
    rclpy.spin(monitor_system)
    monitor_system.destroy_node()
    rclpy.shutdown()