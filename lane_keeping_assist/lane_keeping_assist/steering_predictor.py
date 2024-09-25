"""
Author: Sippawit Thammawiset
Date: August 6, 2024.
File: steering_predictor.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
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

        self.__declare_parameters()
        self.__get_parameters()

        # Publishers
        self.steering_angle_publisher = self.create_publisher(Float32, 'steering_predictor/steering_angle', 10)
        self.debug_image_publisher = self.create_publisher(Image, 'steering_predictor/debug_image', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(Image, self.IMAGE_TOPIC, self.__image_callback, 10)

        # Messages
        self.steering_angle = Float32()
        self.debug_image = Image()

        # Attributes
        self.bridge = CvBridge()
        self.steering_wheel_image = cv2.imread(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                                            'share', 'images', 'steering-wheel-14-256.png'),
                                               cv2.IMREAD_UNCHANGED)
        self.steering_wheel_image = cv2.resize(self.steering_wheel_image, (80, 80))
        self.steering_wheel_image_h, self.steering_wheel_image_w = self.steering_wheel_image.shape[:2]
        self.steering_wheel_image_center = (self.steering_wheel_image_w // 2, self.steering_wheel_image_h // 2)
        model_name = 'fine_tuned-model-42-onnx.onnx'
        self.model = ONNXInference(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                                'share', 'models', model_name),
                                   'input_1',
                                   ['predictions'],
                                   ['CUDAExecutionProvider', 'CPUExecutionProvider'],
                                   'SteeringPredictor')
        self.recent_steering_angles = collections.deque([], maxlen=10)

        self.get_logger().info(
            f'> Using model: {model_name}'
        )
        self.get_logger().info(
            f'{colors.OKGREEN}'
            f'> Initialized {self.model} without any errors. Waiting for image...'
            f'{colors.ENDC}'
        )

    def __declare_parameters(self) -> None:
        self.declare_parameter('image_topic', 'image_raw')

    def __get_parameters(self) -> None:
        self.IMAGE_TOPIC = self.get_parameter('image_topic').get_parameter_value().string_value

    def __image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        background = frame.copy()
        roi_frame = frame[100:-1, 0:-1]
        resized_frame = cv2.resize(roi_frame, self.model.input_shape[-2:-4:-1])

        X = np.expand_dims(resized_frame, axis=0)

        # start_time = time.perf_counter_ns()  # Start time in nanoseconds
        y_pred = self.model.predict(X)
        # time_taken = (time.perf_counter_ns() - start_time) / 1_000_000  # Convert nanoseconds to milliseconds

        y_pred = float(np.squeeze(y_pred, axis=0)[0])
        y_pred = np.round(y_pred, 3)
        y_pred = np.clip(y_pred, -40.0, 40.0)  # Degree

        self.recent_steering_angles.append(y_pred)
        mean_y_pred = np.mean(self.recent_steering_angles, axis=0)

        self.publish_steering_angle(y_pred)
        self.update_debug_image(background, mean_y_pred)
        self.publish_debug_image(background)

    def publish_steering_angle(self, steering_angle: float) -> None:
        self.steering_angle.data = steering_angle
        self.steering_angle_publisher.publish(self.steering_angle)

    def update_debug_image(self, background: np.ndarray, steering_angle: float, sensitivity: float = 0.5) -> None:
        rotation_matrix = cv2.getRotationMatrix2D(self.steering_wheel_image_center, -steering_angle * sensitivity, 1.0)
        rotated_steering_wheel = cv2.warpAffine(self.steering_wheel_image, rotation_matrix,
                                                (self.steering_wheel_image_w, self.steering_wheel_image_h),
                                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        cv2.putText(background, 'Predicted steering angle: {:.3f}'.format(steering_angle), (30, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        add_transparent_image(background, rotated_steering_wheel, 30, 40)

    def publish_debug_image(self, debug_image: np.ndarray) -> None:
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
