"""
Author: Sippawit Thammawiset
Date: August 6, 2024.
File: steering_predictor.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from onnx_inference import ONNXInference
from .utils.add_transparent_image import add_transparent_image
from .utils import colors
import numpy as np
import cv2
import os
import collections


class SteeringPredictorNode(Node):
    def __init__(self) -> None:
        super().__init__('steering_predictor')

        # ROS Parameters
        self.declare_parameter('model_filepath',
                               os.path.join(get_package_share_directory('lane_keeping_assist'), 'share', 'models',
                                            'onnx-SteeringPredictor-18-240x424-rgb8.onnx'))
        self.declare_parameter('compressed_image_topic', 'image_raw/compressed')
        self.declare_parameter('max_steering_angle', 40.0)
        self.declare_parameter('min_steering_angle', -40.0)
        self.declare_parameter('silence', True)

        self.MODEL_FILEPATH = self.get_parameter('model_filepath').get_parameter_value().string_value
        self.IMAGE_TOPIC = self.get_parameter('compressed_image_topic').get_parameter_value().string_value
        self.MAX_STEERING_ANGLE = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.MIN_STEERING_ANGLE = self.get_parameter('min_steering_angle').get_parameter_value().double_value
        self.SILENCE = self.get_parameter('silence').get_parameter_value().bool_value

        # Publishers
        self.steering_angle_publisher = self.create_publisher(Float32, 'steering_predictor/steering_angle', 10)
        self.debug_image_publisher = self.create_publisher(Image, 'steering_predictor/debug_image', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(CompressedImage, self.IMAGE_TOPIC, self.__image_callback, 10)

        # Attributes
        self.steering_angle = Float32()
        self.debug_image = Image()

        self.recent_steering_angles = collections.deque([], maxlen=10)
        self.bridge = CvBridge()
        self.steering_wheel_image = cv2.imread(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                                            'share', 'images', 'steering-wheel-14-256.png'),
                                               cv2.IMREAD_UNCHANGED)
        self.steering_wheel_image = cv2.resize(self.steering_wheel_image, (80, 80))
        self.steering_wheel_image_h, self.steering_wheel_image_w = self.steering_wheel_image.shape[:2]
        self.steering_wheel_image_center = (self.steering_wheel_image_w // 2, self.steering_wheel_image_h // 2)

        model_name = self.MODEL_FILEPATH.split('/')[-1]
        self.get_logger().info(
            f'> Using model: {model_name}'
        )

        try:
            self.model = ONNXInference(self.MODEL_FILEPATH,
                                       'input_1',
                                       ['predictions'],
                                       ['CUDAExecutionProvider', 'CPUExecutionProvider'],
                                       'SteeringPredictor')

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
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        background = rgb_frame.copy()
        roi_frame = rgb_frame[100:-1, 0:-1]
        resized_frame = cv2.resize(roi_frame, self.model.input_shape[-2:-4:-1])

        X = np.expand_dims(resized_frame, axis=0)

        y_pred = self.model.predict(X)
        y_pred = float(np.squeeze(y_pred, axis=0)[0])
        y_pred = np.round(y_pred, 3)
        y_pred = np.clip(y_pred, self.MIN_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        self.recent_steering_angles.append(y_pred)
        mean_y_pred = np.mean(self.recent_steering_angles, axis=0)

        if not self.SILENCE:
            self.get_logger().info('\n'
                                   '       > Predicted steering angle: %s\n' %
                                   y_pred
                                   )

        self.publish_steering_angle(y_pred)
        self.update_debug_image(background, mean_y_pred)
        self.publish_debug_image(background)

    def publish_steering_angle(self,
                               steering_angle: float) -> None:
        self.steering_angle.data = steering_angle
        self.steering_angle_publisher.publish(self.steering_angle)

    def update_debug_image(self,
                           background: np.ndarray,
                           steering_angle: float,
                           sensitivity: float = 0.5) -> None:
        rotation_matrix = cv2.getRotationMatrix2D(self.steering_wheel_image_center, -steering_angle * sensitivity, 1.0)
        rotated_steering_wheel = cv2.warpAffine(self.steering_wheel_image, rotation_matrix,
                                                (self.steering_wheel_image_w, self.steering_wheel_image_h),
                                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        cv2.putText(background, 'Predicted steering angle: {:.3f}'.format(steering_angle), (30, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        add_transparent_image(background, rotated_steering_wheel, 30, 40)

    def publish_debug_image(self,
                            debug_image: np.ndarray) -> None:
        self.debug_image = self.bridge.cv2_to_imgmsg(debug_image, 'rgb8')
        self.debug_image_publisher.publish(self.debug_image)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
