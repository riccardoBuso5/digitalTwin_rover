#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import serial
import math

class WokwiBridge(Node):
    def __init__(self):
        super().__init__('wokwi_bridge')
        self.declare_parameter('wokwi_url', 'rfc2217://localhost:4000')
        self.declare_parameter('obstacle_threshold', 0.8)

        self.url = self.get_parameter('wokwi_url').get_parameter_value().string_value
        self.threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.ser = None
        self.front_obs = False
        self.rear_obs = False
        self.last_linear = 0.0
        self.last_angular = 0.0

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_front = self.create_subscription(LaserScan, '/front_sensor/scan', self.front_cb, 10)
        self.sub_rear = self.create_subscription(LaserScan, '/rear_sensor/scan', self.rear_cb, 10)

        # Invio seriale disaccoppiato a 10 Hz (evita saturazione socket Wokwi)
        self.send_timer = self.create_timer(0.1, self.send_to_wokwi)
        self.conn_timer = self.create_timer(2.0, self.ensure_connected)
        self.ensure_connected()

    def ensure_connected(self):
        if self.ser is not None and self.ser.is_open:
            return
        try:
            self.ser = serial.serial_for_url(
                self.url, baudrate=115200, timeout=0.01, write_timeout=0.01
            )
            self.get_logger().info(f'Connesso a Wokwi su {self.url}')
        except Exception:
            self.ser = None

    def _has_obstacle(self, scan: LaserScan) -> bool:
        valid = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        return min(valid) < self.threshold if valid else False

    def front_cb(self, msg: LaserScan):
        self.front_obs = self._has_obstacle(msg)

    def rear_cb(self, msg: LaserScan):
        self.rear_obs = self._has_obstacle(msg)

    def cmd_callback(self, msg: Twist):
        self.last_linear = msg.linear.x
        self.last_angular = msg.angular.z

    def send_to_wokwi(self):
        if self.ser is None or not self.ser.is_open:
            return
        obs_flag = 1 if (self.front_obs or self.rear_obs) else 0
        payload = f"{self.last_linear:.3f},{self.last_angular:.3f},{obs_flag}\n"
        try:
            self.ser.write(payload.encode('utf-8'))
        except Exception:
            self.ser = None

def main(args=None):
    rclpy.init(args=args)
    node = WokwiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()