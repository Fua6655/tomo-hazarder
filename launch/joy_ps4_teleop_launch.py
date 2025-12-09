# joy_ps4_teleop_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # joy driver
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

        # turtlesim GUI
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # our PS4 teleop node
        Node(
            package='control_tomo',
            executable='ps4_teleop',   # ovo ime dodajemo u setup.py entry_points
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
                'cmd_topic': '/turtle1/cmd_vel',
                'joy_topic': '/joy'
            }]
        )
    ])
