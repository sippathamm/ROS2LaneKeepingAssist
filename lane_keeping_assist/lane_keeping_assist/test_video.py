"""
Author: Sippawit Thammawiset
Date: 06.08.2024
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import os


class TestVideoPublisher(Node):
    def __init__(self):
        super().__init__('test_video_publisher')

        self.declare_parameter('video_filepath',
                               os.path.join(get_package_share_directory('lane_keeping_assist'),
                                            'share', 'test_videos', 'recorded_timestamp.mp4')
                               )

        self.VIDEO_FILEPATH = self.get_parameter('video_filepath').get_parameter_value().string_value

        self.capture = cv2.VideoCapture(self.VIDEO_FILEPATH)

        if not self.capture.isOpened():
            raise RuntimeError('[ERROR] Could not open video file')

        self.video_publisher = self.create_publisher(Image, 'video_raw', 10)
        timer_period = 1 / self.capture.get(cv2.CAP_PROP_FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        _, frame = self.capture.read()

        if not _:
            raise RuntimeError('[ERROR] Could not read frame')

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output_frame = self.bridge.cv2_to_imgmsg(rgb_frame, 'rgb8')
        self.video_publisher.publish(output_frame)


def main(args=None):
    rclpy.init(args=args)
    node = TestVideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
