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

    lane_detector_node = Node(
        package='lane_keeping_assist',
        executable='lane_detector',
        parameters=[params,
                    {'image_topic': 'video_raw'}],
        output='screen',
    )

    steering_predictor_node = Node(
        package='lane_keeping_assist',
        executable='steering_predictor',
        parameters=[params,
                    {'image_topic': 'video_raw'}],
        output='screen',
    )

    core_node = Node(
        package='lane_keeping_assist',
        executable='core',
        parameters=[params],
        output='screen'
    )

    test_video_publisher_node = Node(
        package='lane_keeping_assist',
        executable='test_video_publisher',
        parameters=[params]
    )

    return LaunchDescription([
        lane_detector_node,
        steering_predictor_node,
        core_node,
        test_video_publisher_node
    ])
