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
import cv2
import csv


class CollectDatasetNode(Node):
    def __init__(self) -> None:
        super().__init__('collect_dataset')

        # Subscribers
        self.fbk_steering_angle_subscriber = self.create_subscription(Float32, 'feedback/steering_angle', self.__fbk_steering_angle_callback, 10)
        self.fbk_speed_subscriber = self.create_subscription(Float32, 'feedback/speed', self.__fbk_speed_callback, 10)
        self.image_subscriber = self.create_subscription(Image, 'realsense_camera/image_raw', self.__image_callback, 10)

        # Messages
        self.bridge = CvBridge()
        self.fbk_steering_angle = Float32()
        self.fbk_speed = Float32()
        self.image = Image()

        self.csv_file = open('data_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'steering_angle', 'speed'])

        self.video_writer = cv2.VideoWriter('recorded_raw.mp4', cv2.VideoWriter_fourcc(*"MP4V"), 60, (424, 240))
        self.video_writer_timestamp = cv2.VideoWriter('recorded_timestamp.mp4', cv2.VideoWriter_fourcc(*"MP4V"), 60, (424, 240))

    def __fbk_steering_angle_callback(self, msg: Image) -> None:
        self.fbk_steering_angle = msg

    def __fbk_speed_callback(self, msg: Float32) -> None:
        self.fbk_speed = msg

    def __image_callback(self, msg: Float32) -> None:
        image = msg
        steering_angle = float(self.fbk_steering_angle.data)
        speed = float(self.fbk_speed.data)

        frame = self.bridge.imgmsg_to_cv2(image, 'rgb8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        timestamp_frame = rgb_frame.copy()

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Timestamp with milliseconds
        data1 = round(steering_angle, 3)
        data2 = round(speed, 3)

        cv2.putText(timestamp_frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1,
                    cv2.LINE_AA)

        self.video_writer.write(rgb_frame)
        self.video_writer_timestamp.write(timestamp_frame)
        self.csv_writer.writerow([timestamp, data1, data2])

        self.get_logger().info(
            f'\n'
            f'{timestamp} > {data1} > {data2}'
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
