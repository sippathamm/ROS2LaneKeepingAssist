"""
Author: Sippawit Thammawiset
Date: 06.08.2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .utils.add_transparent_image import add_transparent_image
import numpy as np
import onnxruntime
import cv2
import os
import time


class SteeringPredictor(Node):
    def __init__(self):
        super().__init__('steering_predictor')

        self.declare_parameter('cmd_steering_topic', 'cmd_servo')
        self.declare_parameter('stream_topic', 'raw_stream')
        self.declare_parameter('turn_right_steering_angle_rad', -0.27)
        self.declare_parameter('turn_left_steering_angle_rad', 0.3)
        self.declare_parameter('turn_right_cmd_steering', -1000)
        self.declare_parameter('turn_left_cmd_steering', 1000)
        self.declare_parameter('gain', 1.0)
        
        self.CMD_STEERING_TOPIC = self.get_parameter('cmd_steering_topic').get_parameter_value().string_value
        self.STREAM_TOPIC = self.get_parameter('stream_topic').get_parameter_value().string_value
        self.TURN_RIGHT_STEERING_ANGLE_RAD = self.get_parameter('turn_right_steering_angle_rad').get_parameter_value().double_value
        self.TURN_LEFT_STEERING_ANGLE_RAD = self.get_parameter('turn_left_steering_angle_rad').get_parameter_value().double_value
        self.TURN_RIGHT_CMD_STEERING = self.get_parameter('turn_right_cmd_steering').get_parameter_value().integer_value
        self.TURN_LEFT_CMD_STEERING = self.get_parameter('turn_left_cmd_steering').get_parameter_value().integer_value
        self.GAIN = self.get_parameter('gain').get_parameter_value().double_value

        self.cmd_steering_publisher = self.create_publisher(Int16, self.CMD_STEERING_TOPIC, 10)
        self.steering_debug_publisher = self.create_publisher(Image, 'steering_debug', 10)
        self.stream_subscriber = self.create_subscription(Image, self.STREAM_TOPIC, self.image_callback, 10)
        self.bridge = CvBridge()
        self.steering_wheel_image = cv2.imread(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                                            'share', 'images', 'steering-wheel-14-256.png'),
                                               cv2.IMREAD_UNCHANGED)
        self.steering_wheel_image = cv2.resize(self.steering_wheel_image, (100, 100))

        self.session = onnxruntime.InferenceSession(
            os.path.join(get_package_share_directory('lane_keeping_assist'),
                         'share', 'models', 'model_onnx_rgb8.onnx'),
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )

        self.get_logger().info('> Initialized without any errors. Looking for stream...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.resize(frame, (512, 256))
        background = frame.copy()
        cutoff_frame = frame[136:, :]
        yuv_frame = cv2.cvtColor(cutoff_frame, cv2.COLOR_BGR2RGB)
        resized_frame = cv2.resize(yuv_frame, (128, 64))

        X = resized_frame.reshape(1, 64, 128, 3)
        ortvalue = onnxruntime.OrtValue.ortvalue_from_numpy(X.astype('float32'))

        start_time = time.perf_counter_ns()  # Start time in nanoseconds
        y_pred = self.session.run(['steering_angle_rad'], {'input_1': ortvalue})[0]
        time_taken = (time.perf_counter_ns() - start_time) / 1_000_000  # Convert nanoseconds to milliseconds
        y_pred = y_pred.ravel()[0] * self.GAIN
        steering_angle_rad = min(max(y_pred, self.TURN_RIGHT_STEERING_ANGLE_RAD),
                                 self.TURN_LEFT_STEERING_ANGLE_RAD)

        cmd_steering_angle = Int16()
        cmd_steering_angle.data = int((steering_angle_rad - self.TURN_LEFT_STEERING_ANGLE_RAD) *
                                      (self.TURN_RIGHT_CMD_STEERING - self.TURN_LEFT_CMD_STEERING) /
                                      (self.TURN_RIGHT_STEERING_ANGLE_RAD - self.TURN_LEFT_STEERING_ANGLE_RAD) +
                                      self.TURN_LEFT_CMD_STEERING)
        self.cmd_steering_publisher.publish(cmd_steering_angle)

        self.get_logger().info('\n'
                               '       > Steering CMD: %d\n'
                               '       > Steering angle [rad]: %f\n'
                               '       > Inference time [ms]: %f' % (
                                   cmd_steering_angle.data,
                                   steering_angle_rad,
                                   time_taken)
                               )

        (h, w) = self.steering_wheel_image.shape[:2]
        center = (w // 2, h // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, np.rad2deg(steering_angle_rad) * 2, 1.0)
        rotated_steering_wheel = cv2.warpAffine(self.steering_wheel_image, rotation_matrix, (w, h),
                                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        add_transparent_image(background, rotated_steering_wheel, 30, 30)
        output_image = self.bridge.cv2_to_imgmsg(background, 'bgr8')
        self.steering_debug_publisher.publish(output_image)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
