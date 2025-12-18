#!/usr/bin/env python3
import time
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8MultiArray

try:
    from .ps4_controller import PS4Controller
except Exception:
    from ps4_controller import PS4Controller

class PS4TeleopNode(Node):
    """
    PS4 Teleop Node:
    - LX (axis 0) => angular.z
    - LY (axis 1) => linear.x (inverted)
    - L1 => deadman
    - D-Pad up/down/left/right => lights control
    - X hold -> arm/disarm
    - O hold -> power mode ON/OFF
    - Triangle hold in power mode -> Engine start
    - Square hold -> Light mode ON/OFF
    """
    def __init__(self):
        super().__init__('ps4_teleop')
        # Parameters
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

        # PS4 controller
        self.ps = PS4Controller()

        # States
        self.armed = False
        self.power_mode = False
        self.light_mode = False
        self.engine_start = False

        # Lights
        self.front_short_light = False
        self.front_long_light = False
        self.back_light = False
        self.left_blinker_light = False
        self.right_blinker_light = False

        # Blinkers
        self.left_blinker_active = False
        self.right_blinker_active = False
        self.left_blink_state = False
        self.right_blink_state = False

        # Edge detection
        self.prev_up = False
        self.prev_down = False
        self.prev_left = False
        self.prev_right = False
        self.prev_x = False
        self.prev_O = False
        self.prev_T = False
        self.prev_S = False
        self.x_press_time = None
        self.O_press_time = None
        self.S_press_time = None

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.engine_pub = self.create_publisher(Bool, 'tomo/engine_start', 10)
        self.light_pub = self.create_publisher(UInt8MultiArray, 'tomo/lights', 10)
        self.sub = self.create_subscription(Joy, self.joy_topic, self.joy_cb, 10)

        # Blink timers
        self.left_blink_timer = self.create_timer(0.5, self.left_blink_cb)
        self.right_blink_timer = self.create_timer(0.5, self.right_blink_cb)

        self.get_logger().info(
            f"PS4 teleop started. X hold {self.arm_hold_time}s -> arm/disarm. "
            f"O hold {self.power_hold_time}s -> power mode. "
            f"Triangle hold in power mode -> Engine start."
        )

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
                self.front_short_light,
                self.front_long_light,
                self.back_light,
                self.left_blink_state,
                self.right_blink_state
            ]
        ))

    def joy_cb(self, msg: Joy):
        axes: List[float] = list(getattr(msg, 'axes', []))
        buttons: List[int] = list(getattr(msg, 'buttons', []))
        now = time.monotonic()

        try:
            self.ps.process_joy(axes, buttons)
            self.ps.check_timeout()
        except Exception as e:
            self.get_logger().warn(f"PS4Controller.process_joy exception: {e}")

        # --- X button (arm/disarm) ---
        x = bool(self.ps.X_btn)
        if x and not self.prev_x:
            self.x_press_time = now
        elif x and self.prev_x and self.x_press_time is not None:
            if now - self.x_press_time >= self.arm_hold_time:
                self.armed = not self.armed
                state = "ARMED" if self.armed else "DISARMED"
                self.get_logger().info(f"X hold {self.arm_hold_time}s -> {state}")
                self.x_press_time = None
        elif not x:
            self.x_press_time = None
        self.prev_x = x

        # auto exit from power mode
        if not self.armed and self.power_mode:
            self.power_mode = False
            self.get_logger().info("Not armed -> automatically exiting POWER_MODE")

        # --- O button (power mode) ---
        O_btn = bool(self.ps.O_btn)
        if O_btn and not self.prev_O:
            self.O_press_time = now
        elif O_btn and self.prev_O and self.O_press_time is not None and self.armed:
            if now - self.O_press_time >= self.power_hold_time:
                self.power_mode = not self.power_mode
                state = "POWER_MODE_ON" if self.power_mode else "POWER_MODE_OFF"
                self.get_logger().info(f"O hold {self.power_hold_time}s -> {state}")
                self.O_press_time = None
        elif not O_btn:
            self.O_press_time = None
        self.prev_O = O_btn

        # --- Triangle button (engine start) ---
        T_btn = bool(self.ps.Triangle_btn)
        engine_now = self.armed and self.power_mode and T_btn
        if engine_now != self.engine_start:
            self.engine_start = engine_now
            state_str = "STARTING ENGINE" if self.engine_start else "ABORT START"
            self.get_logger().info(f"Engine state changed: {state_str}")
            self.engine_pub.publish(Bool(data=self.engine_start))

        # auto exit from light mode
        if not self.armed and self.light_mode:
            self.light_mode = False
            self.get_logger().info("Not armed -> automatically exiting LIGHT_MODE")

        # --- Square button (light mode) ---
        S_btn = bool(self.ps.Square_btn)
        if S_btn and not self.prev_S:
            self.S_press_time = now
        elif S_btn and self.prev_S and self.S_press_time is not None and self.armed:
            if now - self.S_press_time >= self.light_hold_time:
                self.light_mode = not self.light_mode
                state = "LIGHT_MODE_ON" if self.light_mode else "LIGHT_MODE_OFF"
                self.get_logger().info(f"Square hold {self.light_hold_time}s -> {state}")
                self.S_press_time = None
        elif not S_btn:
            self.S_press_time = None
        self.prev_S = S_btn

        # --- D-Pad (Lights mode) ---
        if self.light_mode:
            up = bool(self.ps.up_btn)
            down = bool(self.ps.down_btn)
            left = bool(self.ps.left_btn)
            right = bool(self.ps.right_btn)

            # UP: front lights cycle
            if up and not self.prev_up:
                if not self.front_short_light and not self.front_long_light:
                    self.front_short_light = True
                    self.get_logger().info("Front SHORT: ON")
                elif self.front_short_light:
                    self.front_short_light = False
                    self.front_long_light = True
                    self.get_logger().info("Front LONG: ON")
                else:
                    self.front_long_light = False
                    self.get_logger().info("Front lights: OFF")

            # DOWN: back light toggle
            if down and not self.prev_down:
                self.back_light = not self.back_light
                self.get_logger().info(f"Back light: {'ON' if self.back_light else 'OFF'}")

            # LEFT: left blinker toggle
            if left and not self.prev_left:
                self.left_blinker_active = not self.left_blinker_active
                if not self.left_blinker_active:
                    self.left_blink_state = False
                self.get_logger().info(
                    f"Left blinker: {'ON' if self.left_blinker_active else 'OFF'}"
                )

            # RIGHT: right blinker toggle
            if right and not self.prev_right:
                self.right_blinker_active = not self.right_blinker_active
                if not self.right_blinker_active:
                    self.right_blink_state = False
                self.get_logger().info(
                    f"Right blinker: {'ON' if self.right_blinker_active else 'OFF'}"
                )

            # update previous
            self.prev_up = up
            self.prev_down = down
            self.prev_left = left
            self.prev_right = right

            # publish static lights (blinkers publish themselves)
            self.publish_lights()

        # --- Movement ---
        l1 = bool(self.ps.L1_btn or self.ps.L1_btn == 1)
        allowed_to_move = self.armed and l1 and not self.ps.joystick_lost

        def a(i, default=0.0):
            return axes[i] if 0 <= i < len(axes) else default

        raw_lin = a(self.linear_axis)
        raw_ang = a(self.angular_axis)
        raw_lin = self.apply_deadzone(raw_lin)
        raw_ang = self.apply_deadzone(raw_ang)

        lin_scale = self.linear_scale_high #if self.speed_mode_high else self.linear_scale_low
        ang_scale = self.angular_scale_high #if self.speed_mode_high else self.angular_scale_low

        lin = float(raw_lin * lin_scale) if allowed_to_move else 0.0
        ang = float(raw_ang * ang_scale) if allowed_to_move else 0.0

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.pub.publish(twist)


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
