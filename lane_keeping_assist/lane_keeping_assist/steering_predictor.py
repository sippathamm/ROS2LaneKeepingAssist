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
from tensorflow.keras.models import Model, load_model
import numpy as np
import cv2
import os


class SteeringPredictor(Node):
    def __init__(self):
        super().__init__('steering_predictor')

        self.declare_parameter('cmd_steering_topic', 'cmd_servo')
        self.declare_parameter('stream_topic', 'raw_stream')
        self.declare_parameter('turn_right_steering_angle_rad', -0.27)
        self.declare_parameter('turn_left_steering_angle_rad', 0.3)
        self.declare_parameter('turn_right_cmd_steering', -1000)
        self.declare_parameter('turn_left_cmd_steering', 1000)
        self.declare_parameter('verbose', 1)

        self.CMD_STEERING_TOPIC = self.get_parameter('cmd_steering_topic').get_parameter_value().string_value
        self.STREAM_TOPIC = self.get_parameter('stream_topic').get_parameter_value().string_value
        self.TURN_RIGHT_STEERING_ANGLE_RAD = self.get_parameter('turn_right_steering_angle_rad').get_parameter_value().double_value
        self.TURN_LEFT_STEERING_ANGLE_RAD = self.get_parameter('turn_left_steering_angle_rad').get_parameter_value().double_value
        self.TURN_RIGHT_CMD_STEERING = self.get_parameter('turn_right_cmd_steering').get_parameter_value().integer_value
        self.TURN_LEFT_CMD_STEERING = self.get_parameter('turn_left_cmd_steering').get_parameter_value().integer_value
        self.VERBOSE = self.get_parameter('verbose').get_parameter_value().integer_value

        self.cmd_steering_publisher = self.create_publisher(Int16, self.CMD_STEERING_TOPIC, 10)
        self.steering_debug_publisher = self.create_publisher(Image, 'steering_debug', 10)
        self.stream_subscriber = self.create_subscription(Image, self.STREAM_TOPIC, self.image_callback, 10)
        self.bridge = CvBridge()
        self.steering_wheel_image = cv2.imread(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                                            'share', 'images', 'steering-wheel-14-256.png'),
                                               cv2.IMREAD_UNCHANGED)
        self.steering_wheel_image = cv2.resize(self.steering_wheel_image, (100, 100))
        self.model = load_model(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                             'share', 'models', 'steering_predictor.keras'))
        self.model.summary()

        self.get_logger().info('Initialized without any errors. Looking for stream...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.resize(frame, (512, 256))
        background = frame.copy()
        cutoff_frame = frame[136:, :]
        yuv_frame = cv2.cvtColor(cutoff_frame, cv2.COLOR_BGR2YUV)
        resized_frame = cv2.resize(yuv_frame, (128, 64))

        X = resized_frame.reshape(1, 64, 128, 3)
        steering_angle_rad = self.model.predict(x=X, verbose=self.VERBOSE)[0]
        steering_angle_rad = min(max(steering_angle_rad[0], self.TURN_RIGHT_STEERING_ANGLE_RAD), self.TURN_LEFT_STEERING_ANGLE_RAD)

        cmd_steering_angle = Int16()
        cmd_steering_angle.data = int((steering_angle_rad - self.TURN_LEFT_STEERING_ANGLE_RAD) *
                                      (self.TURN_RIGHT_CMD_STEERING - self.TURN_LEFT_CMD_STEERING) /
                                      (self.TURN_RIGHT_STEERING_ANGLE_RAD - self.TURN_LEFT_STEERING_ANGLE_RAD) +
                                      self.TURN_LEFT_CMD_STEERING)
        self.get_logger().info('> CMD Steering: %d | Steering Angle [rad]: %f' % (cmd_steering_angle.data, steering_angle_rad))
        self.cmd_steering_publisher.publish(cmd_steering_angle)

        (h, w) = self.steering_wheel_image.shape[:2]
        center = (w // 2, h // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, np.rad2deg(steering_angle_rad) * 2, 1.0)
        rotated_steering_wheel = cv2.warpAffine(self.steering_wheel_image, rotation_matrix, (w, h),
                                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        self.add_transparent_image(background, rotated_steering_wheel, 30, 30)
        output_image = self.bridge.cv2_to_imgmsg(background, 'rgb8')
        self.steering_debug_publisher.publish(output_image)

    @staticmethod
    def build_model(input_shape: tuple[int, int, int], name: str = 'SteeringPredictor') -> Model:
        input_width: int = input_shape[0]
        input_height: int = input_shape[1]
        input_channels: int = input_shape[2]

        inputs = Input(shape=(input_height, input_width, input_channels))
        rescale = Rescaling(scale=1. / 127.5, offset=-1)(inputs)

        a = Conv2D(filters=16, kernel_size=5, strides=2, activation='elu',
                   kernel_initializer='he_normal')(rescale)
        a = Dropout(rate=0.1)(a)
        a = Conv2D(filters=16, kernel_size=5, strides=2, activation='elu',
                   kernel_initializer='he_normal')(a)
        a = Dropout(rate=0.1)(a)
        a = Conv2D(filters=32, kernel_size=5, strides=2, activation='elu',
                   kernel_initializer='he_normal')(a)
        a = Dropout(rate=0.1)(a)
        a = Conv2D(filters=64, kernel_size=3, strides=1, activation='elu',
                   kernel_initializer='he_normal')(a)
        a = Dropout(rate=0.1)(a)
        a = Conv2D(filters=64, kernel_size=3, strides=1)(a)
        a = MaxPooling2D(pool_size=2, strides=2, padding='same')(a)

        b = Flatten()(a)
        b = Dropout(rate=0.2)(b)
        b = BatchNormalization()(b)
        b = Activation('elu')(b)

        c = Dense(units=512)(b)
        c = Dense(units=128)(c)
        c = Dense(units=32)(c)
        outputs = Dense(units=1)(c)

        return Model(inputs=[inputs], outputs=[outputs], name=name)

    @staticmethod
    def add_transparent_image(background, foreground, x_offset=None, y_offset=None):
        bg_h, bg_w, bg_channels = background.shape
        fg_h, fg_w, fg_channels = foreground.shape

        assert bg_channels == 3, f'background image should have exactly 3 channels (RGB). found:{bg_channels}'
        assert fg_channels == 4, f'foreground image should have exactly 4 channels (RGBA). found:{fg_channels}'

        # center by default
        if x_offset is None: x_offset = (bg_w - fg_w) // 2
        if y_offset is None: y_offset = (bg_h - fg_h) // 2

        w = min(fg_w, bg_w, fg_w + x_offset, bg_w - x_offset)
        h = min(fg_h, bg_h, fg_h + y_offset, bg_h - y_offset)

        if w < 1 or h < 1: return

        # clip foreground and background images to the overlapping regions
        bg_x = max(0, x_offset)
        bg_y = max(0, y_offset)
        fg_x = max(0, x_offset * -1)
        fg_y = max(0, y_offset * -1)
        foreground = foreground[fg_y:fg_y + h, fg_x:fg_x + w]
        background_subsection = background[bg_y:bg_y + h, bg_x:bg_x + w]

        # separate alpha and color channels from the foreground image
        foreground_colors = foreground[:, :, :3]
        alpha_channel = foreground[:, :, 3] / 255  # 0-255 => 0.0-1.0

        # construct an alpha_mask that matches the image shape
        alpha_mask = np.dstack((alpha_channel, alpha_channel, alpha_channel))

        # combine the background with the overlay image weighted by alpha
        composite = background_subsection * (1 - alpha_mask) + foreground_colors * alpha_mask

        # overwrite the section of the background image that has been updated
        background[bg_y:bg_y + h, bg_x:bg_x + w] = composite


def main(args=None):
    rclpy.init(args=args)
    node = SteeringPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
