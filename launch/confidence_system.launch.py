from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'robot_confidence'
    pkg_share = get_package_share_directory(pkg_name)
    params = os.path.join(pkg_share, 'config', 'confidence_params.yaml')

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='confidence_node',
            name='confidence_estimator',
            output='screen',
            parameters=[params]
        ),
        Node(
            package=pkg_name,
            executable='navigation_behavior_node',
            name='navigation_behavior',
            output='screen',
            parameters=[params]
        )
    ])
