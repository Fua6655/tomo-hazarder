import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

try:
    from .ps4_controller import PS4Controller
except Exception:
    from ps4_controller import PS4Controller

RELAY_PIN = 17

class TomoPowerNode(PS4Controller, Node):
    def __init__(self):
        Node.__init__(self, 'tomo_power_node')
        PS4Controller.__init__(self)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.LOW)

        self.tomo_on = False
        self.timer = self.create_timer(0.05, self.check_buttons)

        self.get_logger().info("Tomo Power Node started (inherits PS4Controller)")

    def check_buttons(self):
        self.check_timeout()
        if self.joystick_lost:
            GPIO.output(RELAY_PIN, GPIO.LOW)
            return
        if self.X_btn:
            if not self.tomo_on:
                GPIO.output(RELAY_PIN, GPIO.HIGH)
                self.tomo_on = True
        else:
            pass

    def joy_callback(self, msg):
        self.process_joy(list(msg.axes), list(msg.buttons))


def main(args=None):
    rclpy.init(args=args)
    node = TomoPowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
