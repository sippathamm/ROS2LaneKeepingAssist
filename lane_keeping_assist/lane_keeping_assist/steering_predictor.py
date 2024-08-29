"""
Author: Sippawit Thammawiset
Date: 06.08.2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Bool
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
        self.declare_parameter('cmd_speed_topic', 'cmd_throttle')
        self.declare_parameter('feedback_speed_topic', 'vel_rpt')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('min_speed', 80)
        self.declare_parameter('turn_right_steering_angle_rad', -1.0)
        self.declare_parameter('turn_left_steering_angle_rad', 1.0)
        self.declare_parameter('turn_right_cmd_steering', -1000)
        self.declare_parameter('turn_left_cmd_steering', 1000)
        self.declare_parameter('gain', 1.0)

        self.CMD_STEERING_TOPIC = self.get_parameter('cmd_steering_topic').get_parameter_value().string_value
        self.CMD_SPEED_TOPIC = self.get_parameter('cmd_speed_topic').get_parameter_value().string_value
        self.FEEDBACK_SPEED_TOPIC = self.get_parameter('feedback_speed_topic').get_parameter_value().string_value
        self.IMAGE_TOPIC = self.get_parameter('image_topic').get_parameter_value().string_value
        self.MAX_SPEED = self.get_parameter('max_speed').get_parameter_value().integer_value
        self.MIN_SPEED = self.get_parameter('min_speed').get_parameter_value().integer_value
        self.TURN_RIGHT_STEERING_ANGLE_RAD = self.get_parameter('turn_right_steering_angle_rad').get_parameter_value().double_value
        self.TURN_LEFT_STEERING_ANGLE_RAD = self.get_parameter('turn_left_steering_angle_rad').get_parameter_value().double_value
        self.TURN_RIGHT_CMD_STEERING = self.get_parameter('turn_right_cmd_steering').get_parameter_value().integer_value
        self.TURN_LEFT_CMD_STEERING = self.get_parameter('turn_left_cmd_steering').get_parameter_value().integer_value
        self.GAIN = self.get_parameter('gain').get_parameter_value().double_value

        self.cmd_steering_publisher = self.create_publisher(Int16, self.CMD_STEERING_TOPIC, 10)
        self.cmd_speed_publisher = self.create_publisher(Int16, self.CMD_SPEED_TOPIC, 10)
        self.steering_debug_publisher = self.create_publisher(Image, 'steering_debug', 10)
        self.image_subscriber = self.create_subscription(Image, self.IMAGE_TOPIC, self.image_callback, 10)
        self.feedback_speed_subscriber = self.create_subscription(Float32, self.FEEDBACK_SPEED_TOPIC, self.feedback_speed_callback, 10)

        self.bridge = CvBridge()
        self.steering_wheel_image = cv2.imread(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                                            'share', 'images', 'steering-wheel-14-256.png'),
                                               cv2.IMREAD_UNCHANGED)
        self.steering_wheel_image = cv2.resize(self.steering_wheel_image, (100, 100))
        self.feedback_speed_ms = 0

        self.session = onnxruntime.InferenceSession(
            os.path.join(get_package_share_directory('lane_keeping_assist'),
                         'share', 'models', 'steering_predictor_v1.1.17_dev_128x64_rgb8_onnx.onnx'),
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )

        self.get_logger().info('> Initialized without any errors. Waiting for image...')

    def feedback_speed_callback(self, msg):
        self.feedback_speed_ms = msg.data

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        background = frame.copy()
        roi_frame = frame[int(frame.shape[0] * 0.390625):, :]
        resized_frame = cv2.resize(roi_frame, (128, 64))

        X = np.expand_dims(resized_frame, axis=0)
        ortvalue = onnxruntime.OrtValue.ortvalue_from_numpy(X.astype('float32'))

        start_time = time.perf_counter_ns()  # Start time in nanoseconds
        y_pred = self.session.run(['steering_angle'], {'input_1': ortvalue})[0]
        time_taken = (time.perf_counter_ns() - start_time) / 1_000_000  # Convert nanoseconds to milliseconds

        y_pred = y_pred.ravel()[0] * self.GAIN
        normalized_steering_angle = round(y_pred, 3)
        normalized_steering_angle = min(max(normalized_steering_angle, self.TURN_RIGHT_STEERING_ANGLE_RAD),
                                        self.TURN_LEFT_STEERING_ANGLE_RAD)

        cmd_steering_angle = Int16()
        cmd_steering_angle.data = int((normalized_steering_angle - self.TURN_LEFT_STEERING_ANGLE_RAD) *
                                      (self.TURN_RIGHT_CMD_STEERING - self.TURN_LEFT_CMD_STEERING) /
                                      (self.TURN_RIGHT_STEERING_ANGLE_RAD - self.TURN_LEFT_STEERING_ANGLE_RAD) +
                                      self.TURN_LEFT_CMD_STEERING)
        self.cmd_steering_publisher.publish(cmd_steering_angle)

        cmd_speed = Int16()
        k = 1 if normalized_steering_angle < 0 else -1
        cmd_speed.data = int(k * (self.MAX_SPEED - self.MIN_SPEED) * normalized_steering_angle / 1.0 + self.MAX_SPEED)
        self.cmd_speed_publisher.publish(cmd_speed)

        self.get_logger().info('\n'
                               '       > Steering CMD: %d\n'
                               '       > Speed CMD: %d\n'
                               '       > Steering angle: %f\n'
                               '       > Inference time [ms]: %f' % (
                                   cmd_steering_angle.data,
                                   cmd_speed.data,
                                   normalized_steering_angle,
                                   time_taken)
                               )

        (h, w) = self.steering_wheel_image.shape[:2]
        center = (w // 2, h // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, np.rad2deg(normalized_steering_angle) * 0.5, 1.0)
        rotated_steering_wheel = cv2.warpAffine(self.steering_wheel_image, rotation_matrix, (w, h),
                                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        cv2.putText(background, 'Normalized steering angle: ' + str(normalized_steering_angle), (30, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(background, 'Current speed [m/s]: ' + str('{:.3f}'.format(self.feedback_speed_ms)), (30, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        add_transparent_image(background, rotated_steering_wheel, 30, 30)

        output_image = self.bridge.cv2_to_imgmsg(background, 'rgb8')
        self.steering_debug_publisher.publish(output_image)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
