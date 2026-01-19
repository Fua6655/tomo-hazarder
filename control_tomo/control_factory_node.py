#!/usr/bin/env python3

import time
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray, String, Bool
from geometry_msgs.msg import Twist


class ControlState(Enum):
    IDLE = 0
    JOYSTICK = 1
    WEB = 2
    AUTO = 3
    EMERGENCY = 4


class ControlFactory(Node):

    def __init__(self):
        super().__init__('control_factory')

        # ==================================================
        # PARAMETERS
        # ==================================================
        self.declare_parameter('joy_timeout', 0.2)
        self.declare_parameter('web_timeout', 0.5)
        self.declare_parameter('auto_timeout', 0.2)

        self.joy_timeout = self.get_parameter('joy_timeout').value
        self.web_timeout = self.get_parameter('web_timeout').value
        self.auto_timeout = self.get_parameter('auto_timeout').value

        # ==================================================
        # STATE
        # ==================================================
        self.state = ControlState.IDLE
        self.last_state = None

        self.force_web = False
        self.emergency_active = False

        self.just_exited_emergency = False
        self.require_rearm = False

        self.last_joy = 0.0
        self.last_web = 0.0
        self.last_auto = 0.0

        # ARMED bit index
        self.ARMED_IDX = 0

        # ==============================
        # STORED INPUTS PER SOURCE
        # ==============================
        self.ps4_events = [0, 0, 0, 0]
        self.ps4_states = [0, 0, 0]
        self.ps4_lights = [0, 0, 0, 0, 0, 0]
        self.ps4_cmd = Twist()

        self.web_events = [0, 0, 0, 0]
        self.web_states = [0, 0, 0]
        self.web_lights = [0, 0, 0, 0, 0, 0]
        self.web_cmd = Twist()

        self.auto_events = [0, 0, 0, 0]
        self.auto_states = [0, 0, 0]
        self.auto_lights = [0, 0, 0, 0, 0, 0]
        self.auto_cmd = Twist()

        # ==================================================
        # SUBSCRIBERS
        # ==================================================
        self.create_subscription(UInt8MultiArray, 'ps4/events', self.ps4_events_cb, 10)
        self.create_subscription(UInt8MultiArray, 'ps4/states', self.ps4_states_cb, 10)
        self.create_subscription(UInt8MultiArray, 'ps4/lights', self.ps4_lights_cb, 10)
        self.create_subscription(Twist, 'ps4/cmd_vel', self.ps4_cmd_cb, 10)

        self.create_subscription(UInt8MultiArray, 'web/events', self.web_events_cb, 10)
        self.create_subscription(UInt8MultiArray, 'web/states', self.web_states_cb, 10)
        self.create_subscription(UInt8MultiArray, 'web/lights', self.web_lights_cb, 10)
        self.create_subscription(Twist, 'web/cmd_vel', self.web_cmd_cb, 10)

        self.create_subscription(UInt8MultiArray, 'auto/events', self.auto_events_cb, 10)
        self.create_subscription(UInt8MultiArray, 'auto/states', self.auto_states_cb, 10)
        self.create_subscription(UInt8MultiArray, 'auto/lights', self.auto_lights_cb, 10)
        self.create_subscription(Twist, 'auto/cmd_vel', self.auto_cmd_cb, 10)

        self.create_subscription(Bool, 'web/force_control', self.force_web_cb, 10)
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
        self.create_timer(0.05, self.update_state)
        self.create_timer(0.05, self.publish_output)

        self.get_logger().info('🛑 ControlFactory READY (EMERGENCY enabled)')

    # ==================================================
    # CALLBACKS — STORE INPUTS
    # ==================================================
    def ps4_events_cb(self, msg):
        self.ps4_events = list(msg.data)
        self.last_joy = time.time()
        self.force_web = False

    def ps4_states_cb(self, msg):
        states = list(msg.data)

        if self.require_rearm:
            if states[self.ARMED_IDX] == 1:
                self.require_rearm = False
                self.get_logger().info('🟢 Joystick re-armed after emergency')
            else:
                states[self.ARMED_IDX] = 0

        self.ps4_states = states
        self.last_joy = time.time()
        self.force_web = False

    def ps4_lights_cb(self, msg):
        self.ps4_lights = list(msg.data)
        self.last_joy = time.time()
        self.force_web = False

    def ps4_cmd_cb(self, msg):
        self.ps4_cmd = msg
        self.last_joy = time.time()
        self.force_web = False

    def web_events_cb(self, msg): self.web_events = list(msg.data); self.last_web = time.time()
    def web_states_cb(self, msg): self.web_states = list(msg.data); self.last_web = time.time()
    def web_lights_cb(self, msg): self.web_lights = list(msg.data); self.last_web = time.time()
    def web_cmd_cb(self, msg): self.web_cmd = msg; self.last_web = time.time()

    def auto_events_cb(self, msg): self.auto_events = list(msg.data); self.last_auto = time.time()
    def auto_states_cb(self, msg): self.auto_states = list(msg.data); self.last_auto = time.time()
    def auto_lights_cb(self, msg): self.auto_lights = list(msg.data); self.last_auto = time.time()
    def auto_cmd_cb(self, msg): self.auto_cmd = msg; self.last_auto = time.time()

    def force_web_cb(self, msg: Bool):
        self.force_web = msg.data

    def emergency_cb(self, msg: Bool):
        prev = self.emergency_active
        self.emergency_active = msg.data
        self.pub_emergency.publish(Bool(data=self.emergency_active))

        if self.emergency_active:
            self.get_logger().error('🛑 EMERGENCY STOP ACTIVATED')
        else:
            self.get_logger().warn('🟢 Emergency released')
            if prev:
                self.just_exited_emergency = True
                self.require_rearm = True

    # ==================================================
    # STATE MACHINE
    # ==================================================
    def update_state(self):

        # ---------- EMERGENCY EXIT GUARD ----------
        if self.just_exited_emergency:
            self.ps4_states[self.ARMED_IDX] = 0
            self.web_states[self.ARMED_IDX] = 0
            self.auto_states[self.ARMED_IDX] = 0

            self.ps4_cmd = Twist()
            self.web_cmd = Twist()
            self.auto_cmd = Twist()

            self.just_exited_emergency = False

        if self.emergency_active:
            self.state = ControlState.EMERGENCY
        else:
            now = time.time()
            joy_alive = (now - self.last_joy) < self.joy_timeout
            web_alive = (now - self.last_web) < self.web_timeout
            auto_alive = (now - self.last_auto) < self.auto_timeout

            if self.force_web and web_alive:
                self.state = ControlState.WEB
            elif joy_alive:
                self.state = ControlState.JOYSTICK
            elif auto_alive:
                self.state = ControlState.AUTO
            else:
                self.state = ControlState.IDLE

        if self.state != self.last_state:
            self.pub_source.publish(String(data=self.state.name))
            self.get_logger().info(f'ACTIVE SOURCE → {self.state.name}')
            self.last_state = self.state

    # ==================================================
    # OUTPUT MULTIPLEXER
    # ==================================================
    def publish_output(self):
        if self.state == ControlState.EMERGENCY:
            self.pub_cmd.publish(Twist())
            self.pub_events.publish(UInt8MultiArray(data=[0, 0, 0, 0]))
            self.pub_states.publish(UInt8MultiArray(data=[0, 0, 0]))
            self.pub_lights.publish(UInt8MultiArray(data=[0, 0, 0, 0, 0, 0]))
            return

        if self.state == ControlState.JOYSTICK:
            e, s, l, c = self.ps4_events, self.ps4_states, self.ps4_lights, self.ps4_cmd
        elif self.state == ControlState.WEB:
            e, s, l, c = self.web_events, self.web_states, self.web_lights, self.web_cmd
        elif self.state == ControlState.AUTO:
            e, s, l, c = self.auto_events, self.auto_states, self.auto_lights, self.auto_cmd
        else:
            return

        self.pub_events.publish(UInt8MultiArray(data=e))
        self.pub_states.publish(UInt8MultiArray(data=s))
        self.pub_lights.publish(UInt8MultiArray(data=l))
        self.pub_cmd.publish(c)


def main():
    rclpy.init()
    node = ControlFactory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
