from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():    
    test_video_path = os.path.join(
        get_package_share_directory('lane_keeping_assist'),
        'share',
        'test_videos',
        'recorded_timestamp.mp4'
    )
    
    params = os.path.join(
        get_package_share_directory('lane_keeping_assist'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='stream_publisher',
            executable='from_video',
            parameters=[{
                'stream_source_path': test_video_path,
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
