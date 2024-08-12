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
            package='stream_publisher',
            executable='from_camera',
            parameters=[{
                'stream_source_index': 0,
                'silence': True
            }]
        ),
        Node(
            package='lane_keeping_assist',
            executable='steering_predictor',
            parameters=[params],
            output='screen',
        )
    ])
