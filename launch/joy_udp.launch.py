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
                'cmd_topic': '/ps4/cmd_vel',
                'joy_topic': '/joy'
            }]
        ),
        # ESP32 UDP Node
        Node(
            package='control_tomo',
            executable='esp_udp',
            name='esp_udp',
            output='screen',
            parameters=[{
                'esp_ip': '192.168.0.187',
                'esp_port': 8888
            }]
        ),
        # Control factory UDP Node
        Node(
            package='control_tomo',
            executable='control_factory',
            name='control_factory',
            output='screen',
            parameters=[{
                'joy_timeout': 0.2,
                'auto_timeout': 0.2
            }]
        ),
        Node(
            package='control_tomo',
            executable='web_server',
            name='web_server',
            output='screen'
        ),
    ])
