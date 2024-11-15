import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('ds4dt_teleop'),
        'config',
        'params.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='ds4dt_teleop',
            executable='ds4dt_teleop_node',
            name='ds4dt_teleop_node',
            parameters=[config],
            namespace='tb4'
        ),
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node'
        )
    ])
