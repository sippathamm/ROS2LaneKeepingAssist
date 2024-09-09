from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('lane_keeping_assist'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', 'realsense_camera/image_compressed'),
                ('out', 'realsense_camera/image_uncompressed')]
        ),
        Node(
            package='lane_keeping_assist',
            executable='lane_detector_node',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='lane_keeping_assist',
            executable='steering_predictor_node',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='lane_keeping_assist',
            executable='core_node',
            parameters=[params],
            output='screen'
        )
    ])
