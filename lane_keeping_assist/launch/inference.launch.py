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

    image_transpot_node = Node(
        package='image_transport',
        executable='republish',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', 'realsense_camera/image_compressed'),
            ('out', 'realsense_camera/image_uncompressed')]
    )

    lane_detector_node = Node(
        package='lane_keeping_assist',
        executable='lane_detector',
        parameters=[params],
        output='screen',
    )

    steering_predictor_node = Node(
        package='lane_keeping_assist',
        executable='steering_predictor',
        parameters=[params],
        output='screen',
    )

    core_node = Node(
        package='lane_keeping_assist',
        executable='core',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([
        image_transpot_node,
        lane_detector_node,
        steering_predictor_node,
        core_node
    ])
