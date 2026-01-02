#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial


class ArduinoSerialNode(Node):

    def __init__(self):
        super().__init__('arduino_serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.ser = serial.Serial(port, baudrate, timeout=0.1)

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
            f"Arduino serial node running on {port} @ {baudrate}"
        )

    # ---------------- STATES ----------------
    def states_cb(self, msg: UInt8MultiArray):
        if len(msg.data) != 3:
            return
        a, p, l = msg.data
        line = f"STATES,{a},{p},{l}\n"
        self.ser.write(line.encode())

    # ---------------- EVENTS ----------------
    def events_cb(self, msg: UInt8MultiArray):
        if len(msg.data) != 2:
            return
        e, c = msg.data
        line = f"EVENTS,{e},{c}\n"
        self.ser.write(line.encode())

    # ---------------- LIGHTS ----------------
    def lights_cb(self, msg: UInt8MultiArray):
        if len(msg.data) != 6:
            return
        fp, fs, fl, b, l, r = msg.data
        line = f"LIGHTS,{fp},{fs},{fl},{b},{l},{r}\n"
        self.ser.write(line.encode())


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
