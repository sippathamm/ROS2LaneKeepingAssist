"""
Author: Sippawit Thammawiset
Date: August 6, 2024.
File: test_video_publisher.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .utils import colors
from tqdm import tqdm
import cv2
import os


class TestVideoPublisherNode(Node):
    def __init__(self):
        super().__init__('test_video_publisher')

        self.declare_parameter('video_filepath',
                               os.path.join(get_package_share_directory('lane_keeping_assist'),
                                            'share', 'test_videos', 'normal_drive.mp4')
                               )

        self.VIDEO_FILEPATH = self.get_parameter('video_filepath').get_parameter_value().string_value

        self.capture = cv2.VideoCapture(self.VIDEO_FILEPATH)

        if not self.capture.isOpened():
            raise RuntimeError(
                f'{colors.ERROR}'
                f'[ERROR] Could not open video file.'
                f'{colors.ENDC}'
            )

        self.total_frames = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.pbar = tqdm(total=self.total_frames, desc='Playing video', unit='frame')

        self.video_publisher = self.create_publisher(Image, 'video_raw', 10)
        timer_period = 1 / self.capture.get(cv2.CAP_PROP_FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        _, frame = self.capture.read()

        if not _:
            self.pbar.close()
            self.capture.release()

            raise RuntimeError(
                f'{colors.ERROR}'
                f'[ERROR] Could not read frame.'
                f'{colors.ENDC}'
            )

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output_frame = self.bridge.cv2_to_imgmsg(rgb_frame, 'rgb8')
        self.video_publisher.publish(output_frame)

        self.pbar.update(1)


def main(args=None):
    rclpy.init(args=args)
    node = TestVideoPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
