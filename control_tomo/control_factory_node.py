#!/usr/bin/env python3

from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String, Bool
from geometry_msgs.msg import Twist


class ControlState(Enum):
    JOYSTICK = 1
    WEB = 2
    AUTO = 3
    EMERGENCY = 4


class ControlFactory(Node):

    def __init__(self):
        super().__init__('control_factory')

        self.state = ControlState.JOYSTICK
        self.last_state = None

        self.force_web = False
        self.force_auto = False
        self.emergency_active = False

        # ---------------- INPUT STORAGE ----------------
        self.ps4_events = [0]*4
        self.ps4_states = [0]*3
        self.ps4_lights = [0]*6
        self.ps4_cmd = Twist()

        self.web_events = [0]*4
        self.web_lights = [0]*6

        self.auto_cmd = Twist()

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(UInt8MultiArray, 'ps4/events', lambda m: setattr(self, "ps4_events", list(m.data)), 10)
        self.create_subscription(UInt8MultiArray, 'ps4/states', lambda m: setattr(self, "ps4_states", list(m.data)), 10)
        self.create_subscription(UInt8MultiArray, 'ps4/lights', lambda m: setattr(self, "ps4_lights", list(m.data)), 10)
        self.create_subscription(Twist, 'ps4/cmd_vel', lambda m: setattr(self, "ps4_cmd", m), 10)

        self.create_subscription(UInt8MultiArray, 'web/events', lambda m: setattr(self, "web_events", list(m.data)), 10)
        self.create_subscription(UInt8MultiArray, 'web/lights', lambda m: setattr(self, "web_lights", list(m.data)), 10)

        self.create_subscription(Twist, 'auto/cmd_vel', lambda m: setattr(self, "auto_cmd", m), 10)

        self.create_subscription(Bool, 'web/force_control', self.web_force_cb, 10)
        self.create_subscription(Bool, 'auto/force_control', self.auto_force_cb, 10)
        self.create_subscription(Bool, 'web/emergency', self.emergency_cb, 10)

        # ---------------- PUBLISHERS ----------------
        self.pub_events = self.create_publisher(UInt8MultiArray, 'tomo/events', 10)
        self.pub_states = self.create_publisher(UInt8MultiArray, 'tomo/states', 10)
        self.pub_lights = self.create_publisher(UInt8MultiArray, 'tomo/lights', 10)
        self.pub_cmd = self.create_publisher(Twist, 'tomo/cmd_vel', 10)

        self.pub_source = self.create_publisher(String, 'factory/active_source', 10)
        self.pub_emergency = self.create_publisher(Bool, 'factory/emergency_active', 10)

        self.create_timer(0.05, self.update)
        self.get_logger().info("✅ ControlFactory READY")

    # --------------------------------------------------
    def web_force_cb(self, msg: Bool):
        self.force_web = msg.data
        if msg.data:
            self.force_auto = False

    def auto_force_cb(self, msg: Bool):
        self.force_auto = msg.data
        if msg.data:
            self.force_web = False

    def emergency_cb(self, msg: Bool):
        self.emergency_active = msg.data

    # --------------------------------------------------
    def hard_zero(self):
        self.pub_events.publish(UInt8MultiArray(data=[0]*4))
        self.pub_states.publish(UInt8MultiArray(data=[0]*3))
        self.pub_lights.publish(UInt8MultiArray(data=[0]*6))
        self.pub_cmd.publish(Twist())

    # --------------------------------------------------
    def update(self):

        # ---------- STATE RESOLUTION ----------
        if self.emergency_active:
            new_state = ControlState.EMERGENCY
        elif self.force_web:
            new_state = ControlState.WEB
        elif self.force_auto:
            new_state = ControlState.AUTO
        else:
            new_state = ControlState.JOYSTICK

        # ---------- STATE CHANGE ----------
        if new_state != self.state:
            self.hard_zero()
            self.state = new_state
            self.pub_source.publish(String(data=self.state.name))
            self.get_logger().warn(f"SOURCE → {self.state.name}")

        # ---------- OUTPUT ----------
        if self.state == ControlState.EMERGENCY:
            self.pub_emergency.publish(Bool(data=True))
            self.pub_cmd.publish(Twist())
            self.pub_events.publish(UInt8MultiArray(data=[0] * 4))
            self.pub_states.publish(UInt8MultiArray(data=[0] * 3))
            self.pub_lights.publish(UInt8MultiArray(data=self.ps4_lights))
            return

        self.pub_emergency.publish(Bool(data=False))

        if self.state == ControlState.JOYSTICK:
            self.pub_events.publish(UInt8MultiArray(data=self.ps4_events))
            self.pub_states.publish(UInt8MultiArray(data=self.ps4_states))
            self.pub_lights.publish(UInt8MultiArray(data=self.ps4_lights))
            self.pub_cmd.publish(self.ps4_cmd)

        elif self.state == ControlState.WEB:
            self.pub_events.publish(UInt8MultiArray(data=self.web_events))
            self.pub_lights.publish(UInt8MultiArray(data=self.web_lights))

        elif self.state == ControlState.AUTO:
            self.pub_cmd.publish(self.auto_cmd)


def main():
    rclpy.init()
    rclpy.spin(ControlFactory())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
