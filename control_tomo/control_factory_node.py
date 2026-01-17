#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String, Bool


class ControlFactory(Node):

    def __init__(self):
        super().__init__('control_factory')

        # ---------- PARAMETERS ----------
        self.declare_parameter('joy_timeout', 0.2)
        self.declare_parameter('auto_timeout', 0.2)

        self.joy_timeout  = self.get_parameter('joy_timeout').value
        self.auto_timeout = self.get_parameter('auto_timeout').value

        self.get_logger().info(
            f"Timeouts: joy={self.joy_timeout}s auto={self.auto_timeout}s"
        )

        # ---------- STATE ----------
        self.last_joy  = 0.0
        self.last_auto = 0.0

        # ACTIVE CONTROL SOURCE
        self.active_source = "web"   # default nakon boota
        self.force_web = False       # <-- NOVO

        # ---------- OUTPUT ----------
        self.pub_events = self.create_publisher(UInt8MultiArray, 'tomo/events', 10)
        self.pub_states = self.create_publisher(UInt8MultiArray, 'tomo/states', 10)
        self.pub_lights = self.create_publisher(UInt8MultiArray, 'tomo/lights', 10)

        # ACTIVE SOURCE INFO (za web/debug)
        self.pub_source = self.create_publisher(String, 'factory/active_source', 10)

        # ---------- INPUTS ----------
        self.create_subscription(UInt8MultiArray, 'ps4/events',  self.ps4_events_cb, 10)
        self.create_subscription(UInt8MultiArray, 'ps4/states',  self.ps4_states_cb, 10)
        self.create_subscription(UInt8MultiArray, 'ps4/lights',  self.ps4_lights_cb, 10)

        self.create_subscription(UInt8MultiArray, 'auto/events', self.auto_events_cb, 10)

        self.create_subscription(UInt8MultiArray, 'web/events',  self.web_events_cb, 10)
        self.create_subscription(UInt8MultiArray, 'web/states',  self.web_states_cb, 10)
        self.create_subscription(UInt8MultiArray, 'web/lights',  self.web_lights_cb, 10)

        # ---------- FORCE WEB ----------
        self.create_subscription(Bool,'web/force_control',self.force_web_cb,10)

        # ---------- TIMER ----------
        self.create_timer(0.05, self.check_timeouts)

        self.get_logger().info("ControlFactory READY")

    # ==================================================
    # ================= SOURCE LOGIC ===================
    # ==================================================

    def set_active_source(self, source: str):
        if self.active_source != source:
            self.active_source = source
            self.pub_source.publish(String(data=source))
            self.get_logger().info(f"ACTIVE SOURCE → {source.upper()}")

    def force_web_cb(self, msg: Bool):
        if msg.data:
            self.force_web = True
            self.set_active_source("web")
            self.get_logger().warn("WEB FORCE CONTROL ENABLED")

    def check_timeouts(self):
        now = time.time()

        if self.active_source == "ps4":
            if now - self.last_joy > self.joy_timeout:
                self.set_active_source("web")

        elif self.active_source == "auto":
            if now - self.last_auto > self.auto_timeout:
                self.set_active_source("web")

    # ---------- PRIORITY ----------
    def allow_auto(self):
        return (not self.force_web) and (time.time() - self.last_joy > self.joy_timeout)

    def allow_web(self):
        if self.force_web:
            return True

        return (
            time.time() - self.last_joy  > self.joy_timeout and
            time.time() - self.last_auto > self.auto_timeout
        )

    # ==================================================
    # ================= PS4 (TOP) ======================
    # ==================================================
    def ps4_events_cb(self, msg):
        self.last_joy = time.time()

        if self.force_web:
            self.force_web = False
            self.get_logger().warn("PS4 TOOK OVER – WEB FORCE DISABLED")

        self.set_active_source("ps4")
        self.pub_events.publish(msg)

    def ps4_states_cb(self, msg):
        self.last_joy = time.time()

        if self.force_web:
            self.force_web = False

        self.set_active_source("ps4")
        self.pub_states.publish(msg)

    def ps4_lights_cb(self, msg):
        self.last_joy = time.time()

        if self.force_web:
            self.force_web = False

        self.set_active_source("ps4")
        self.pub_lights.publish(msg)

    # ==================================================
    # ================= AUTO ===========================
    # ==================================================
    def auto_events_cb(self, msg):
        self.last_auto = time.time()

        if self.allow_auto():
            self.set_active_source("auto")
            self.pub_events.publish(msg)

    # ==================================================
    # ================= WEB ============================
    # ==================================================
    def web_events_cb(self, msg):
        if self.allow_web():
            self.set_active_source("web")
            self.pub_events.publish(msg)

    def web_states_cb(self, msg):
        if self.allow_web():
            self.set_active_source("web")
            self.pub_states.publish(msg)

    def web_lights_cb(self, msg):
        if self.allow_web():
            self.set_active_source("web")
            self.pub_lights.publish(msg)


def main():
    rclpy.init()
    node = ControlFactory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
