#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class EngineSerialNode(Node):
    def __init__(self):
        super().__init__('engine_serial_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.serial_conn = None
        self.prev_state = False

        self.sub = self.create_subscription(Bool, '/engine_start', self.engine_cb, 10)
        self.create_timer(1.0, self.check_connection)

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.serial_port_name, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to Arduino at {self.serial_port_name}")
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to Arduino: {e}")
            self.serial_conn = None

    def check_connection(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.connect_serial()

    def engine_cb(self, msg: Bool):
        state = msg.data
        if state != self.prev_state:
            self.prev_state = state
            if state:
                self.send_command("ON")
            # OFF ne šaljemo jer relej pali samo jednom

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
    node = EngineSerialNode()
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
