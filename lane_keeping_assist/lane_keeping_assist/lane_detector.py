"""
Author: Sippawit Thammawiset
Date: September 5, 2024.
File: lane_detector.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from coefficient_msg.msg import Coefficients
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from onnx_inference import ONNXInference
from typing import Tuple
from .utils import colors
import numpy as np
import cv2
import os


class LaneDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('lane_detector')

        # ROS Parameters
        self.declare_parameter('model_filepath',
                               os.path.join(get_package_share_directory('lane_keeping_assist'), 'share', 'models',
                                            'onnx-lane_detector-240x424-rgb8.onnx'))
        self.declare_parameter('compressed_image_topic', 'image_raw/compressed')
        self.declare_parameter('degree', 2)
        self.declare_parameter('silence', True)

        self.MODEL_FILEPATH = self.get_parameter('model_filepath').get_parameter_value().string_value
        self.IMAGE_TOPIC = self.get_parameter('compressed_image_topic').get_parameter_value().string_value
        self.DEGREE = self.get_parameter('degree').get_parameter_value().integer_value
        self.SILENCE = self.get_parameter('silence').get_parameter_value().bool_value

        # Publishers
        self.lane_coeffs_publisher = {
            'left': self.create_publisher(Coefficients, 'lane_detector/left_lane/coefficients', 10),
            'right': self.create_publisher(Coefficients, 'lane_detector/right_lane/coefficients', 10)
        }
        self.debug_image_publisher = self.create_publisher(Image, 'lane_detector/debug_image', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(CompressedImage, self.IMAGE_TOPIC, self.__image_callback, 10)

        # Attributes
        self.lane_coeffs = {
            'left': Coefficients(),
            'right': Coefficients()
        }
        self.debug_image = Image()

        self.bridge = CvBridge()

        model_name = self.MODEL_FILEPATH.split('/')[-1]
        self.get_logger().info(
            f'> Using model: {model_name}'
        )

        try:
            self.model = ONNXInference(self.MODEL_FILEPATH,
                                       'input_1',
                                       ['bin', 'inst'],
                                       ['CUDAExecutionProvider', 'CPUExecutionProvider'],
                                       'LaneDetector')

            self.get_logger().info(
                f'{colors.OKGREEN}'
                f'> Initialized {self.__class__.__name__} without any errors. Waiting for image...'
                f'{colors.ENDC}'
            )
        except Exception as e:
            self.get_logger().info(
                f'{colors.ERROR}'
                f'> Failed to initialize {self.__class__.__name__}. '
                f'Reason: "{e}"'
                f'{colors.ENDC}'
            )
            self.destroy_node()

    def __image_callback(self,
                         msg: CompressedImage) -> None:
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        resized_frame = cv2.resize(frame, self.model.input_shape[-2:-4:-1])
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

        X = np.expand_dims(rgb_frame, axis=0)

        bin_seg, inst_seg = self.model.predict(X)  # Expensive!

        # bin_seg_thresh = (bin_seg >= 0.5).astype(np.uint8)
        inst_seg_thresh = (inst_seg >= 0.5).astype(np.uint8)

        left_lane_mask = inst_seg_thresh[0, :, :, 1]
        right_lane_mask = inst_seg_thresh[0, :, :, 2]

        left_lane_u, left_lane_v = self.find_mask_pixels(left_lane_mask)
        right_lane_u, right_lane_v = self.find_mask_pixels(right_lane_mask)

        left_lane_v_sample = self.get_v_sample(left_lane_v)
        right_lane_v_sample = self.get_v_sample(right_lane_v)

        try:
            left_lane_coeffs = np.polyfit(left_lane_v, left_lane_u, self.DEGREE)
            left_lane_fit_u = np.poly1d(left_lane_coeffs)(left_lane_v_sample)

            for u, v in zip(left_lane_fit_u, left_lane_v_sample):
                center_coordinates = (int(u), int(v))
                radius = 3
                color = (255, 0, 0)
                thickness = -1
                cv2.circle(rgb_frame, center_coordinates, radius, color, thickness)

            cv2.putText(rgb_frame,
                        'Left lane coeffs: {}.'.format(left_lane_coeffs),
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
        except TypeError:
            left_lane_coeffs = np.zeros((self.DEGREE,))

            cv2.putText(rgb_frame,
                        'Left lane coeffs: Failed to fit',
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 0), 1)

        try:
            right_lane_coeffs = np.polyfit(right_lane_v, right_lane_u, self.DEGREE)
            right_lane_fit_u = np.poly1d(right_lane_coeffs)(right_lane_v_sample)

            for u, v in zip(right_lane_fit_u, right_lane_v_sample):
                center_coordinates = (int(u), int(v))
                radius = 3
                color = (0, 0, 255)
                thickness = -1
                cv2.circle(rgb_frame, center_coordinates, radius, color, thickness)

            cv2.putText(rgb_frame,
                        'Right lane coeffs: {}.'.format(right_lane_coeffs),
                        (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
        except TypeError:
            right_lane_coeffs = np.zeros((self.DEGREE,))

            cv2.putText(rgb_frame,
                        'Right lane coeffs: Failed to fit',
                        (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 0), 1)

        if not self.SILENCE:
            self.get_logger().info('\n'
                                   '       > Left lane coeffs: %s\n'
                                   '       > Right lane coeffs: %s\n' %
                                   (left_lane_coeffs.tolist(),
                                    right_lane_coeffs.tolist())
                                   )

        self.publish_lane_coeffs('left', left_lane_coeffs)
        self.publish_lane_coeffs('right', right_lane_coeffs)
        self.publish_debug_image(rgb_frame)

    @staticmethod
    def find_mask_pixels(image: np.ndarray,
                         n_windows: int = 9,
                         margin: int = 100,
                         min_px: int = 50) -> Tuple[np.ndarray, np.ndarray]:
        """
        Legacy version
        """

        height, width = image.shape

        histogram = np.sum(image[height // 2:, :], axis=0)
        peak = np.argmax(histogram)  # Find the peak the histogram

        window_height = np.int32(height // n_windows)
        current = peak

        nonzero = image.nonzero()
        nonzero_v = np.array(nonzero[0])
        nonzero_u = np.array(nonzero[1])

        indices = []

        for window in range(n_windows):
            window_v_min = height - (window + 1) * window_height
            window_v_max = height - window * window_height
            window_u_min = current - margin
            window_u_max = current + margin

            index = ((nonzero_v >= window_v_min) & (nonzero_v < window_v_max) &
                     (nonzero_u >= window_u_min) & (nonzero_u < window_u_max)).nonzero()[0]

            indices.append(index)

            if len(indices) > min_px:
                current = np.int32(np.mean(nonzero_u[indices]))

        try:
            indices = np.concatenate(indices)
        except ValueError:
            pass

        u, v = nonzero_u[indices], nonzero_v[indices]

        return u, v

    @staticmethod
    def get_v_sample(v: np.ndarray,
                     scaling_factor: float = 0.005) -> np.ndarray:
        try:
            v_sample = np.linspace(v.min(), v.max(), int(scaling_factor * len(v)))
        except ValueError:
            v_sample = []

        return v_sample

    def publish_lane_coeffs(self,
                            lane: str,
                            coeffs: np.ndarray) -> None:
        self.lane_coeffs[lane].data = [coeff for coeff in coeffs]
        self.lane_coeffs_publisher[lane].publish(self.lane_coeffs[lane])

    def publish_debug_image(self,
                            debug_image: np.ndarray) -> None:
        self.debug_image = self.bridge.cv2_to_imgmsg(debug_image, 'rgb8')
        self.debug_image_publisher.publish(self.debug_image)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
