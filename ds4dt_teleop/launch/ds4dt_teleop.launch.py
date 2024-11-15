from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ds4dt_teleop',
            executable='ds4dt_teleop_node',
            name='ds4dt_teleop_node'
        ),
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node'
        )
    ])
