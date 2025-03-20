import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('vff_avoidance'),
        'config',
        'AvoidanceNodeConfig.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='vff_avoidance',
            executable='avoidance_vff_main',
            name='avoidance_node',
            output='screen',
            parameters=[config_file]
        )
    ])
