# turtlesim_only.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
            remappings=[
                ('/turtle1/cmd_vel', '/tomo/cmd_vel'),
            ]
        )
    ])
