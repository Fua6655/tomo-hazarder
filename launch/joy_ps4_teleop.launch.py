# joy_ps4_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # PS4 joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        ),

        # PS4 teleop node for Tomo Vinković
        Node(
            package='control_tomo',
            executable='ps4_teleop',
            name='ps4_teleop',
            output='screen',
            parameters=[{
                'linear_axis': 1,
                'angular_axis': 0,
                'deadzone': 0.08,
                'linear_scale_high': 1.0,
                'linear_scale_low': 0.4,
                'angular_scale_high': 2.0,
                'angular_scale_low': 1.0,
                'arm_hold_time': 3.0,
                'power_hold_time': 2.0,
                'light_hold_time': 2.0,
                'cmd_topic': '/tomo/cmd_vel',
                'joy_topic': '/joy'
            }]
        ),
        # Arduino Serial Node
        Node(
            package='control_tomo',
            executable='arduino_serial',
            name='arduino_serial',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baudrate': 115200
            }]
        )
    ])
