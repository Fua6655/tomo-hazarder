#!/usr/bin/env python3
# ps4_teleop_node.py
import time
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

try:
    from .ps4_controller import PS4Controller
except Exception:
    from ps4_controller import PS4Controller

class PS4TeleopNode(Node):
    """
    Teleop node koji koristi PS4Controller:
    - LX (axis 0) => angular.z
    - LY (axis 1) => linear.x (inverted)
    - L1 (button) => deadman (mora biti držan)
    - D-Pad up/down => set speed mode high/low
    - X button hold 3s => arm / disarm (toggle)
    """
    def __init__(self):
        super().__init__('ps4_teleop')
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('linear_scale_high', 1.0)
        self.declare_parameter('linear_scale_low', 0.4)
        self.declare_parameter('angular_scale_high', 2.0)
        self.declare_parameter('angular_scale_low', 1.0)
        self.declare_parameter('arm_hold_time', 3.0)
        self.declare_parameter('cmd_topic', '/turtle1/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_rate', 20.0)

        self.linear_axis = int(self.get_parameter('linear_axis').value)
        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.linear_scale_high = float(self.get_parameter('linear_scale_high').value)
        self.linear_scale_low = float(self.get_parameter('linear_scale_low').value)
        self.angular_scale_high = float(self.get_parameter('angular_scale_high').value)
        self.angular_scale_low = float(self.get_parameter('angular_scale_low').value)
        self.arm_hold_time = float(self.get_parameter('arm_hold_time').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)

        self.ps = PS4Controller()
        self.armed = False
        self.speed_mode_high = False
        self.prev_up = False
        self.prev_down = False
        self.prev_x = False
        self.x_press_time = None

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub = self.create_subscription(Joy, self.joy_topic, self.joy_cb, 10)

        self.get_logger().info(
            f"PS4 teleop started. cmd_topic='{self.cmd_topic}', joy_topic='{self.joy_topic}'. "
            f"Hold X for {self.arm_hold_time}s to arm/disarm. Hold L1 to move."
        )

    def apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def joy_cb(self, msg: Joy):
        axes: List[float] = list(getattr(msg, 'axes', []))
        buttons = list(getattr(msg, 'buttons', []))

        try:
            self.ps.process_joy(axes, buttons)
            self.ps.check_timeout()
        except Exception as e:
            self.get_logger().warn(f"PS4Controller.process_joy exception: {e}")

        up = bool(self.ps.up_btn)
        down = bool(self.ps.down_btn)
        if up and not self.prev_up:
            self.speed_mode_high = True
            self.get_logger().info("Speed mode: HIGH")
        if down and not self.prev_down:
            self.speed_mode_high = False
            self.get_logger().info("Speed mode: LOW")
        self.prev_up = up
        self.prev_down = down

        x = bool(self.ps.X_btn)
        now = time.monotonic()
        if x and not self.prev_x:
            self.x_press_time = now
        elif not x and self.prev_x:
            self.x_press_time = None
        elif x and self.prev_x and self.x_press_time is not None:
            if now - self.x_press_time >= self.arm_hold_time:
                self.armed = not self.armed
                state = "ARMED" if self.armed else "DISARMED"
                self.get_logger().info(f"X hold {self.arm_hold_time}s -> {state}")
                self.x_press_time = None
        self.prev_x = x

        l1 = bool(self.ps.L1_btn or self.ps.L1_btn == 1)
        allowed_to_move = self.armed and l1 and not self.ps.joystick_lost

        def a(i, default=0.0):
            return axes[i] if 0 <= i < len(axes) else default

        raw_lin = a(self.linear_axis)
        raw_ang = a(self.angular_axis)

        raw_lin = self.apply_deadzone(raw_lin)
        raw_ang = self.apply_deadzone(raw_ang)

        lin_scale = self.linear_scale_high if self.speed_mode_high else self.linear_scale_low
        ang_scale = self.angular_scale_high if self.speed_mode_high else self.angular_scale_low

        lin = float(raw_lin * lin_scale) if allowed_to_move else 0.0
        ang = float(raw_ang * ang_scale) if allowed_to_move else 0.0

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang

        self.pub.publish(twist)

        if allowed_to_move:
            self.get_logger().debug(f"Publishing cmd_vel: lin={lin:.3f}, ang={ang:.3f}")
        else:
            self.get_logger().debug("Joystick lost or not allowed to move - publishing zero twist")


def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down ps4_teleop node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
