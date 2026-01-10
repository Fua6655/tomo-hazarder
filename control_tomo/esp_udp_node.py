#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import socket


class EspUdpNode(Node):

    def __init__(self):
        super().__init__('esp_udp_node')

        # ---------- PARAMETERS ----------
        self.declare_parameter('esp_ip', '192.168.0.116')
        self.declare_parameter('esp_port', 8888)

        self.esp_ip = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value

        # ---------- UDP SOCKET ----------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ---------- SUBSCRIPTIONS ----------
        self.create_subscription(
            UInt8MultiArray, 'tomo/states', self.states_cb, 10
        )
        self.create_subscription(
            UInt8MultiArray, 'tomo/events', self.events_cb, 10
        )
        self.create_subscription(
            UInt8MultiArray, 'tomo/lights', self.lights_cb, 10
        )

        self.get_logger().info(
            f"ESP UDP node sending to {self.esp_ip}:{self.esp_port}"
        )

    # ---------------- STATES ----------------
    def states_cb(self, msg: UInt8MultiArray):
        if len(msg.data) != 3:
            return
        a, p, l = msg.data
        line = f"STATES,{a},{p},{l}\n"
        self.send(line)

    # ---------------- EVENTS ----------------
    def events_cb(self, msg: UInt8MultiArray):
        if len(msg.data) != 4:
            return
        e, c, h, a = msg.data
        line = f"EVENTS,{e},{c},{h},{a}\n"
        self.send(line)

    # ---------------- LIGHTS ----------------
    def lights_cb(self, msg: UInt8MultiArray):
        if len(msg.data) != 6:
            return
        fp, fs, fl, b, l, r = msg.data
        line = f"LIGHTS,{fp},{fs},{fl},{b},{l},{r}\n"
        self.send(line)

    # ---------------- SEND ----------------
    def send(self, line: str):
        self.sock.sendto(
            line.encode(),
            (self.esp_ip, self.esp_port)
        )


def main(args=None):
    rclpy.init(args=args)
    node = EspUdpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
