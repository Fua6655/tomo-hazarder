#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('engine_serial_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('blink_interval', 500)  # default 500ms

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.blink_interval = int(self.get_parameter('blink_interval').value)

        self.serial_conn = None
        self.prev_engine_state = None
        self.prev_light_state = [None] * 5  # front_short, front_long, back, left, right

        self.engine_sub = self.create_subscription(Bool, 'tomo/engine_start', self.engine_cb, 10)
        self.light_sub = self.create_subscription(UInt8MultiArray, 'tomo/lights', self.light_cb, 10)
        self.create_timer(1.0, self.check_connection)

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.serial_port_name, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to Arduino at {self.serial_port_name}")
            # pošalji blink interval parametar Arduinu
            self.send_command(f"SET_BLINK_INTERVAL {self.blink_interval}")
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to Arduino: {e}")
            self.serial_conn = None

    def check_connection(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.connect_serial()

    def engine_cb(self, msg: Bool):
        state = msg.data
        if state != self.prev_engine_state:
            self.prev_engine_state = state
            cmd = "ENGINE_ON" if state else "ENGINE_OFF"
            self.send_command(cmd)

    def light_cb(self, msg: UInt8MultiArray):
        light = msg.data
        pin_names = [
            ("FRONT_SHORT_LIGHT", light[0]),
            ("FRONT_LONG_LIGHT", light[1]),
            ("BACK_LIGHT", light[2]),
            ("LEFT_BLINK", light[3]),
            ("RIGHT_BLINK", light[4])
        ]

        for i, (name, val) in enumerate(pin_names):
            if val != self.prev_light_state[i]:
                self.prev_light_state[i] = val
                cmd = f"{name}_ON" if val else f"{name}_OFF"
                self.send_command(cmd)

    def send_command(self, cmd: str):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(f"{cmd}\n".encode())
                self.get_logger().info(f"Sent command to Arduino: {cmd}")
            except Exception as e:
                self.get_logger().warn(f"Failed to send command: {e}")
        else:
            self.get_logger().warn("Serial connection not open, cannot send command")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
