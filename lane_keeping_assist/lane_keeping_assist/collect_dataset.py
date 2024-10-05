"""
Author: Sippawit Thammawiset
Date: September 20, 2024.
File: collect_dataset.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
from .utils import colors
import cv2
import csv


class CollectDatasetNode(Node):
    def __init__(self) -> None:
        super().__init__('collect_dataset')

        # Parameters
        self.declare_parameter('steering_topic', 'feedback/steering')
        self.declare_parameter('velocity_topic', 'feedback/velocity')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('width', 512)
        self.declare_parameter('height', 256)
        self.declare_parameter('frequency', 30)

        self.STEERING_TOPIC = self.get_parameter('steering_topic').get_parameter_value().string_value
        self.VELOCITY_TOPIC = self.get_parameter('velocity_topic').get_parameter_value().string_value
        self.IMAGE_TOPIC = self.get_parameter('image_topic').get_parameter_value().string_value
        self.WIDTH = self.get_parameter('width').get_parameter_value().integer_value
        self.HEIGHT = self.get_parameter('height').get_parameter_value().integer_value
        self.FREQUENCY = self.get_parameter('frequency').get_parameter_value().integer_value

        # Subscribers
        self.fbk_steering_subscriber = self.create_subscription(Float32, self.STEERING_TOPIC,
                                                                self.__fbk_steering_callback, 10)
        self.fbk_velocity_subscriber = self.create_subscription(Float32, self.VELOCITY_TOPIC,
                                                                self.__fbk_velocity_callback, 10)
        self.image_subscriber = self.create_subscription(Image, self.IMAGE_TOPIC,
                                                         self.__image_callback, 10)

        # Attributes
        self.bridge = CvBridge()
        self.fbk_steering_angle = Float32()
        self.fbk_velocity = Float32()
        self.image = Image()

        self.csv_file = open('data_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'steering_angle', 'velocity'])

        self.video_writer = cv2.VideoWriter('recorded_raw.mp4', cv2.VideoWriter_fourcc(*"MP4V"),
                                            self.FREQUENCY, (self.WIDTH, self.HEIGHT))
        self.video_writer_timestamp = cv2.VideoWriter('recorded_timestamp.mp4', cv2.VideoWriter_fourcc(*"MP4V"),
                                                      self.FREQUENCY, (self.WIDTH, self.HEIGHT))

        self.get_logger().info(
            f'{colors.OKGREEN}'
            f'> Waiting for image...'
            f'{colors.ENDC}'
        )

    def __fbk_steering_callback(self,
                                msg: Float32) -> None:
        self.fbk_steering_angle = msg

    def __fbk_velocity_callback(self,
                                msg: Float32) -> None:
        self.fbk_velocity = msg

    def __image_callback(self,
                         msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        steering_angle = float(self.fbk_steering_angle.data)
        velocity = float(self.fbk_velocity.data)

        resized_frame = cv2.resize(frame, (self.WIDTH, self.HEIGHT))
        timestamp_frame = resized_frame.copy()

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Timestamp with milliseconds

        cv2.putText(timestamp_frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1,
                    cv2.LINE_AA)

        self.video_writer.write(resized_frame)
        self.video_writer_timestamp.write(timestamp_frame)
        self.csv_writer.writerow([timestamp, steering_angle, velocity])

        self.get_logger().info(
            f'\n'
            f'> Timestamp: {timestamp}\n'
            f'> Steering angle: {steering_angle}\n'
            f'> Velocity: {velocity}'
        )

    def destroy_node(self) -> None:
        self.csv_file.close()
        self.video_writer.release()
        self.video_writer_timestamp.release()

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollectDatasetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
