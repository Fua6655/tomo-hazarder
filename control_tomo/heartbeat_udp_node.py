#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import time


class HeartbeatUDP(Node):

    def __init__(self):
        super().__init__('heartbeat_udp')

        # ---- PARAMETERS ----
        self.declare_parameter('esp_ip', '192.168.0.116')
        self.declare_parameter('esp_port', 8888)
        self.declare_parameter('rate_hz', 10.0)  # 10 Hz = every 100 ms

        self.esp_ip = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value
        self.rate_hz = self.get_parameter('rate_hz').value

        # ---- UDP SOCKET ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.send_heartbeat)

        self.get_logger().info(
            f"Heartbeat UDP -> {self.esp_ip}:{self.esp_port} @ {self.rate_hz} Hz"
        )

    def send_heartbeat(self):
        try:
            self.sock.sendto(b'HEARTBEAT', (self.esp_ip, self.esp_port))
        except Exception as e:
            self.get_logger().warn(f"Heartbeat send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatUDP()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
