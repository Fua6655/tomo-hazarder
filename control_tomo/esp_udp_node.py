#!/usr/bin/env python3
import rclpy
import socket
import time
import threading
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String


class EspUdpNode(Node):

    def __init__(self):
        super().__init__('esp_udp_node')

        self.declare_parameter('esp_ip', '192.168.0.116')
        self.declare_parameter('esp_port', 8888)

        self.esp_ip   = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value

        # ---------- UDP ----------
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.bind(('', self.esp_port + 1))
        self.rx_sock.settimeout(0.1)

        self.seq = 0
        self.pending = {}

        # ---------- ROS ----------
        self.status_pub  = self.create_publisher(String, 'esp/status', 10)
        self.latency_pub = self.create_publisher(String, 'esp/latency', 10)

        self.create_subscription(UInt8MultiArray, 'tomo/states', self.states_cb, 10)
        self.create_subscription(UInt8MultiArray, 'tomo/events', self.events_cb, 10)
        self.create_subscription(UInt8MultiArray, 'tomo/lights', self.lights_cb, 10)

        self.create_timer(0.2, self.send_heartbeat)
        threading.Thread(target=self.rx_loop, daemon=True).start()

        self.get_logger().info(f"ESP UDP node → {self.esp_ip}:{self.esp_port}")

    # ---------- SEND ----------
    def send_cmd(self, payload: str):
        self.seq += 1
        msg = f"CMD,{self.seq},{payload}"
        self.pending[self.seq] = time.time()
        self.tx_sock.sendto(msg.encode(), (self.esp_ip, self.esp_port))

    # ---------- CALLBACKS ----------
    def states_cb(self, msg):
        if len(msg.data) == 3:
            self.send_cmd(f"STATES,{msg.data[0]},{msg.data[1]},{msg.data[2]}")

    def events_cb(self, msg):
        if len(msg.data) == 4:
            self.send_cmd(
                f"EVENTS,{msg.data[0]},{msg.data[1]},{msg.data[2]},{msg.data[3]}"
            )

    def lights_cb(self, msg):
        if len(msg.data) == 6:
            self.send_cmd(
                f"LIGHTS,{msg.data[0]},{msg.data[1]},{msg.data[2]},"
                f"{msg.data[3]},{msg.data[4]},{msg.data[5]}"
            )

    # ---------- HEARTBEAT ----------
    def send_heartbeat(self):
        self.seq += 1
        self.pending[self.seq] = time.time()
        self.tx_sock.sendto(
            f"HEARTBEAT,{self.seq}".encode(),
            (self.esp_ip, self.esp_port)
        )

    # ---------- RX ----------
    def rx_loop(self):
        while rclpy.ok():
            try:
                data, _ = self.rx_sock.recvfrom(256)
            except:
                continue

            msg = data.decode().strip()
            self.status_pub.publish(String(data=msg))

            if msg.startswith("ACK"):
                seq = int(msg.split(",")[1])
                if seq in self.pending:
                    latency = (time.time() - self.pending.pop(seq)) * 1000
                    self.latency_pub.publish(
                        String(data=f"{latency:.1f} ms")
                    )


def main():
    rclpy.init()
    node = EspUdpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
