#!/usr/bin/env python3
import time
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray

try:
    from .ps4_controller import PS4Controller
except Exception:
    from ps4_controller import PS4Controller


class PS4TeleopNode(Node):
    """
    PS4 Teleop Node
    """

    def __init__(self):
        super().__init__('ps4_teleop')

        # ---------------- Parameters ----------------
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('linear_scale_high', 1.0)
        self.declare_parameter('linear_scale_low', 0.4)
        self.declare_parameter('angular_scale_high', 2.0)
        self.declare_parameter('angular_scale_low', 1.0)
        self.declare_parameter('arm_hold_time', 3.0)
        self.declare_parameter('power_hold_time', 3.0)
        self.declare_parameter('light_hold_time', 3.0)
        self.declare_parameter('cmd_topic', '/tomo/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')

        self.linear_axis = int(self.get_parameter('linear_axis').value)
        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.linear_scale_high = float(self.get_parameter('linear_scale_high').value)
        self.linear_scale_low = float(self.get_parameter('linear_scale_low').value)
        self.angular_scale_high = float(self.get_parameter('angular_scale_high').value)
        self.angular_scale_low = float(self.get_parameter('angular_scale_low').value)
        self.arm_hold_time = float(self.get_parameter('arm_hold_time').value)
        self.power_hold_time = float(self.get_parameter('power_hold_time').value)
        self.light_hold_time = float(self.get_parameter('light_hold_time').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)

        # ---------------- Controller ----------------
        self.ps = PS4Controller()

        # ---------------- States ----------------
        self.armed = False
        self.power_mode = False
        self.light_mode = False

        # ---------------- Events ----------------
        self.engine_start = False
        self.clutch_down = False

        # ---------------- Lights ----------------
        self.front_position_light = False
        self.front_short_light = False
        self.front_long_light = False
        self.back_light = False
        self.left_blinker_light = False
        self.right_blinker_light = False

        self.left_blinker_active = False
        self.right_blinker_active = False
        self.left_blink_state = False
        self.right_blink_state = False

        # ---------------- Edge / timers ----------------
        self._prev_armed = False
        self.prev_up = False
        self.prev_down = False
        self.prev_left = False
        self.prev_right = False
        self.prev_x = False
        self.prev_O = False
        self.prev_T = False
        self.prev_S = False
        self.prev_R1 = False

        self.x_press_time = None
        self.O_press_time = None
        self.S_press_time = None

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.states_pub = self.create_publisher(UInt8MultiArray, 'tomo/states', 10)
        self.events_pub = self.create_publisher(UInt8MultiArray, 'tomo/events', 10)
        self.light_pub = self.create_publisher(UInt8MultiArray, 'tomo/lights', 10)

        # ---------------- Subscriber ----------------
        self.sub = self.create_subscription(Joy, self.joy_topic, self.joy_cb, 10)

        # ---------------- Blink timers ----------------
        self.left_blink_timer = self.create_timer(0.5, self.left_blink_cb)
        self.right_blink_timer = self.create_timer(0.5, self.right_blink_cb)

        self.get_logger().info("PS4 teleop started")

    # ------------------------------------------------

    def apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def left_blink_cb(self):
        if self.left_blinker_active:
            self.left_blink_state = not self.left_blink_state
            self.publish_lights()

    def right_blink_cb(self):
        if self.right_blinker_active:
            self.right_blink_state = not self.right_blink_state
            self.publish_lights()

    def publish_lights(self):
        self.light_pub.publish(UInt8MultiArray(
            data=[
                int(self.front_position_light),
                int(self.front_short_light),
                int(self.front_long_light),
                int(self.back_light),
                int(self.left_blink_state),
                int(self.right_blink_state),
            ]
        ))

    # ------------------------------------------------

    def joy_cb(self, msg: Joy):
        axes: List[float] = list(msg.axes)
        buttons: List[int] = list(msg.buttons)
        now = time.monotonic()

        # ---------------- Process controller ----------------
        self.ps.process_joy(axes, buttons)
        self.ps.check_timeout()

        # ================= ARM =================
        x = bool(self.ps.X_btn)
        if x and not self.prev_x:
            self.x_press_time = now
        elif x and self.prev_x and self.x_press_time and now - self.x_press_time >= self.arm_hold_time:
            self.armed = not self.armed
            self.get_logger().info(f"{'ARMED' if self.armed else 'DISARMED'}")
            self.x_press_time = None
        elif not x:
            self.x_press_time = None
        self.prev_x = x

        if self.armed and not self.front_position_light:
            self.front_position_light = True
            self.get_logger().info("FRONT POSITION LIGHT ON")
            self.publish_lights()

        # ================= DISARM =================
        elif self._prev_armed and not self.armed:
            self.front_position_light = False
            self.front_short_light = False
            self.front_long_light = False
            self.back_light = False
            self.left_blinker_active = False
            self.right_blinker_active = False
            self.left_blink_state = False
            self.right_blink_state = False

            self.get_logger().info("LIGHTS OFF (disarmed)")
            self.publish_lights()

        self._prev_armed = self.armed

        # ================= POWER MODE =================
        if not self.armed and self.power_mode:
            self.power_mode = False

        O_btn = bool(self.ps.O_btn)
        if O_btn and not self.prev_O:
            self.O_press_time = now
        elif O_btn and self.prev_O and self.O_press_time and self.armed and now - self.O_press_time >= self.power_hold_time:
            self.power_mode = not self.power_mode
            self.get_logger().info(f"POWER MODE {'ON' if self.power_mode else 'OFF'}")
            self.O_press_time = None
        elif not O_btn:
            self.O_press_time = None
        self.prev_O = O_btn

        # ================= LIGHT MODE =================
        if not self.armed and self.light_mode:
            self.light_mode = False

        S_btn = bool(self.ps.Square_btn)
        if S_btn and not self.prev_S:
            self.S_press_time = now
        elif S_btn and self.prev_S and self.S_press_time and self.armed and now - self.S_press_time >= self.light_hold_time:
            self.light_mode = not self.light_mode
            self.get_logger().info(f"LIGHT MODE {'ON' if self.light_mode else 'OFF'}")
            self.S_press_time = None
        elif not S_btn:
            self.S_press_time = None
        self.prev_S = S_btn

        # ================= D-PAD LIGHT CONTROL =================
        if self.light_mode:
            up = bool(self.ps.up_btn)
            down = bool(self.ps.down_btn)
            left = bool(self.ps.left_btn)
            right = bool(self.ps.right_btn)

            # UP: front lights cycle
            if up and not self.prev_up:
                if not self.front_short_light and not self.front_long_light:
                    self.front_short_light = True
                    self.get_logger().info("FRONT SHORT ON")
                elif self.front_short_light:
                    self.front_short_light = False
                    self.front_long_light = True
                    self.get_logger().info("FRONT LONG ON")
                else:
                    self.front_long_light = False
                    self.get_logger().info("FRONT LIGHT OFF")

            # DOWN: back light toggle
            if down and not self.prev_down:
                self.back_light = not self.back_light
                self.get_logger().info(
                    f"BACK LIGHT {'ON' if self.back_light else 'OFF'}"
                )

            # LEFT: left blinker toggle
            if left and not self.prev_left:
                self.left_blinker_active = not self.left_blinker_active
                if not self.left_blinker_active:
                    self.left_blink_state = False
                self.get_logger().info(
                    f"LEFT BLINKER {'ON' if self.left_blinker_active else 'OFF'}"
                )

            # RIGHT: right blinker toggle
            if right and not self.prev_right:
                self.right_blinker_active = not self.right_blinker_active
                if not self.right_blinker_active:
                    self.right_blink_state = False
                self.get_logger().info(
                    f"RIGHT BLINKER {'ON' if self.right_blinker_active else 'OFF'}"
                )

            # update previous
            self.prev_up = up
            self.prev_down = down
            self.prev_left = left
            self.prev_right = right

            # IMPORTANT: publish lights immediately
            self.publish_lights()

        # ================= EVENTS =================

        engine_now = self.armed and self.power_mode and bool(self.ps.Triangle_btn)
        if engine_now != self.engine_start:
            self.engine_start = engine_now
            if self.engine_start:
                self.get_logger().info(
                    f"ENGINE STARTING (armed={self.armed}, power_mode={self.power_mode})"
                )
            else:
                self.get_logger().info("ENGINE START ABORT")

        clutch_now = self.armed and bool(self.ps.R1_btn)
        if clutch_now != self.clutch_down:
            self.clutch_down = clutch_now
            if self.clutch_down:
                self.get_logger().info(
                    f"CLUTCH ACTIVE (armed={self.armed})"
                )
            else:
                self.get_logger().info("CLUTCH INACTIVE")

        self.events_pub.publish(
            UInt8MultiArray(
                data=[int(self.engine_start), int(self.clutch_down)]
            )
        )

        # ================= STATES =================
        self.states_pub.publish(UInt8MultiArray(
            data=[int(self.armed), int(self.power_mode), int(self.light_mode)]
        ))

        # ================= MOVEMENT =================
        l1 = bool(self.ps.L1_btn)
        allowed = self.armed and l1 and not self.ps.joystick_lost

        def a(i, default=0.0):
            return axes[i] if 0 <= i < len(axes) else default

        lin = self.apply_deadzone(a(self.linear_axis))
        ang = self.apply_deadzone(a(self.angular_axis))

        twist = Twist()
        twist.linear.x = lin * self.linear_scale_high if allowed else 0.0
        twist.angular.z = ang * self.angular_scale_high if allowed else 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
