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

        self.force_web = False
        self.force_auto = False
        self.emergency_active = False

        # -------- EMERGENCY BLINK --------
        self.emergency_blink_state = False

        # ==================================================
        # LAST TOMO OUTPUT (FOR HANDOVER)
        # ==================================================
        self.last_events = [0] * 4
        self.last_states = [0] * 3
        self.last_lights = [0] * 6
        self.last_cmd = Twist()

        # ==================================================
        # INPUT STORAGE
        # ==================================================
        self.ps4_events = [0]*4
        self.ps4_states = [0]*3
        self.ps4_lights = [0]*6
        self.ps4_cmd = Twist()

        self.web_events = [0]*4
        self.web_lights = [0]*6

        self.auto_cmd = Twist()

        # ==================================================
        # SUBSCRIBERS
        # ==================================================
        self.create_subscription(UInt8MultiArray, 'ps4/events',
                                 lambda m: setattr(self, "ps4_events", list(m.data)), 10)
        self.create_subscription(UInt8MultiArray, 'ps4/states',
                                 lambda m: setattr(self, "ps4_states", list(m.data)), 10)
        self.create_subscription(UInt8MultiArray, 'ps4/lights',
                                 lambda m: setattr(self, "ps4_lights", list(m.data)), 10)
        self.create_subscription(Twist, 'ps4/cmd_vel',
                                 lambda m: setattr(self, "ps4_cmd", m), 10)

        self.create_subscription(UInt8MultiArray, 'web/events',
                                 lambda m: setattr(self, "web_events", list(m.data)), 10)
        self.create_subscription(UInt8MultiArray, 'web/lights',
                                 lambda m: setattr(self, "web_lights", list(m.data)), 10)

        self.create_subscription(Twist, 'auto/cmd_vel',
                                 lambda m: setattr(self, "auto_cmd", m), 10)

        self.create_subscription(Bool, 'web/force_control', self.web_force_cb, 10)
        self.create_subscription(Bool, 'auto/force_control', self.auto_force_cb, 10)
        self.create_subscription(Bool, 'web/emergency', self.emergency_cb, 10)

        # ==================================================
        # PUBLISHERS
        # ==================================================
        self.pub_events = self.create_publisher(UInt8MultiArray, 'tomo/events', 10)
        self.pub_states = self.create_publisher(UInt8MultiArray, 'tomo/states', 10)
        self.pub_lights = self.create_publisher(UInt8MultiArray, 'tomo/lights', 10)
        self.pub_cmd = self.create_publisher(Twist, 'tomo/cmd_vel', 10)

        self.pub_source = self.create_publisher(String, 'factory/active_source', 10)
        self.pub_emergency = self.create_publisher(Bool, 'factory/emergency_active', 10)

        # ==================================================
        # TIMERS
        # ==================================================
        self.create_timer(0.05, self.update)
        self.create_timer(0.5, self._emergency_blink_timer)

        self.get_logger().info("✅ ControlFactory READY")

    # ==================================================
    # CALLBACKS
    # ==================================================

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

    # ==================================================
    # UTILS
    # ==================================================

    @staticmethod
    def clone_twist(t: Twist):
        c = Twist()
        c.linear.x = t.linear.x
        c.linear.y = t.linear.y
        c.linear.z = t.linear.z
        c.angular.x = t.angular.x
        c.angular.y = t.angular.y
        c.angular.z = t.angular.z
        return c

    def hard_zero(self):
        self.pub_events.publish(UInt8MultiArray(data=[0]*4))
        self.pub_states.publish(UInt8MultiArray(data=[0]*3))
        self.pub_lights.publish(UInt8MultiArray(data=[0]*6))
        self.pub_cmd.publish(Twist())

    def full_reset(self):
        self.last_events = [0]*4
        self.last_states = [0]*3
        self.last_lights = [0]*6
        self.last_cmd = Twist()

        self.ps4_events = [0]*4
        self.ps4_states = [0]*3
        self.ps4_lights = [0]*6
        self.ps4_cmd = Twist()

        self.web_events = [0]*4
        self.web_lights = [0]*6
        self.auto_cmd = Twist()

    # 🔑 SAVE CURRENT TOMO OUTPUT
    def snapshot_current_output(self):
        if self.state == ControlState.JOYSTICK:
            self.last_events = list(self.ps4_events)
            self.last_states = list(self.ps4_states)
            self.last_lights = list(self.ps4_lights)
            self.last_cmd = self.clone_twist(self.ps4_cmd)

        elif self.state == ControlState.WEB:
            self.last_events = list(self.web_events)
            self.last_lights = list(self.web_lights)

        elif self.state == ControlState.AUTO:
            self.last_cmd = self.clone_twist(self.auto_cmd)

    # ==================================================
    # EMERGENCY BLINK TIMER
    # ==================================================

    def _emergency_blink_timer(self):
        if self.state != ControlState.EMERGENCY:
            self.emergency_blink_state = False
            return

        self.emergency_blink_state = not self.emergency_blink_state
        self.pub_lights.publish(UInt8MultiArray(
            data=[0, 0, 0, 0,
                  int(self.emergency_blink_state),
                  int(self.emergency_blink_state)]
        ))

    # ==================================================
    # MAIN LOOP
    # ==================================================

    def update(self):

        if self.emergency_active:
            new_state = ControlState.EMERGENCY
        elif self.force_web:
            new_state = ControlState.WEB
        elif self.force_auto:
            new_state = ControlState.AUTO
        else:
            new_state = ControlState.JOYSTICK

        if new_state != self.state:
            self.get_logger().warn(f"SOURCE → {new_state.name}")

            # 1️⃣ SAVE CURRENT OUTPUT
            if self.state != ControlState.EMERGENCY:
                self.snapshot_current_output()

            # 2️⃣ EMERGENCY EDGES
            if new_state == ControlState.EMERGENCY:
                self.hard_zero()

            elif self.state == ControlState.EMERGENCY:
                self.hard_zero()
                self.full_reset()

            # 3️⃣ HANDOVER INTO NEW SOURCE
            if new_state == ControlState.JOYSTICK:
                self.ps4_events = list(self.last_events)
                self.ps4_states = list(self.last_states)
                self.ps4_lights = list(self.last_lights)
                self.ps4_cmd = self.clone_twist(self.last_cmd)

            elif new_state == ControlState.WEB:
                self.web_events = list(self.last_events)
                self.web_lights = list(self.last_lights)

            elif new_state == ControlState.AUTO:
                self.auto_cmd = self.clone_twist(self.last_cmd)

            self.state = new_state
            self.pub_source.publish(String(data=self.state.name))

        # ---------- OUTPUT ----------
        if self.state == ControlState.EMERGENCY:
            self.pub_emergency.publish(Bool(data=True))
            self.pub_cmd.publish(Twist())
            self.pub_events.publish(UInt8MultiArray(data=[0]*4))
            self.pub_states.publish(UInt8MultiArray(data=[0]*3))
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
