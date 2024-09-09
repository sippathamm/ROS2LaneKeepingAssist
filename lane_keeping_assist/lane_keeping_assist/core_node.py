"""
Author: Sippawit Thammawiset
Date: September 9, 2024.
File: core_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from custom_msgs.msg import Coefficients
from ament_index_python.packages import get_package_share_directory
from .utils import colors


class CoreNode(Node):
    ZERO_COEFFS = Coefficients()
    ZERO_COEFFS.data = [0.0, 0.0, 0.0, 0.0]

    def __init__(self) -> None:
        super().__init__('core_node')

        self.__declare_parameters()
        self.__get_parameters()

        # Core Timer
        self.timer = self.create_timer(0.033, self.__timer_callback)

        # Publishers
        self.cmd_steering_publisher = self.create_publisher(Int16, self.CMD_STEERING_TOPIC, 10)
        self.cmd_speed_publisher = self.create_publisher(Int16, self.CMD_SPEED_TOPIC, 10)

        # Subscribers
        self.steering_angle_subscriber = self.create_subscription(Float32, 'steering_predictor/steering_angle',
                                                                  self.__steering_angle_callback, 10)
        self.lane_coeffs_subscriber = {
            'left': self.create_subscription(Coefficients, 'lane_detector/left_lane/coefficients',
                                             self.__left_lane_coeff_callback, 10),
            'right': self.create_subscription(Coefficients, 'lane_detector/right_lane/coefficients',
                                             self.__right_lane_coeff_callback, 10)
        }

        # Messages
        self.cmd_steering = Int16()
        self.cmd_speed = Int16()
        self.steering_angle = Float32()
        self.lane_coeffs = {
            'left': Coefficients(),
            'right': Coefficients(),
        }

        # self.get_logger().info(
        #     f'{colors.OKGREEN}'
        #     f'> OK'
        #     f'{colors.ENDC}'
        # )

    def __declare_parameters(self) -> None:
        self.declare_parameter('cmd_steering_topic', 'cmd_steering')
        self.declare_parameter('cmd_speed_topic', 'cmd_speed')
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('min_speed', 80)
        self.declare_parameter('turn_right_cmd_steering', -1000)
        self.declare_parameter('turn_left_cmd_steering', 1000)
        self.declare_parameter('gain', 1.0)

    def __get_parameters(self) -> None:
        self.CMD_STEERING_TOPIC = self.get_parameter('cmd_steering_topic').get_parameter_value().string_value
        self.CMD_SPEED_TOPIC = self.get_parameter('cmd_speed_topic').get_parameter_value().string_value
        self.MAX_SPEED = self.get_parameter('max_speed').get_parameter_value().integer_value
        self.MIN_SPEED = self.get_parameter('min_speed').get_parameter_value().integer_value
        self.TURN_RIGHT_CMD_STEERING = self.get_parameter('turn_right_cmd_steering').get_parameter_value().integer_value
        self.TURN_LEFT_CMD_STEERING = self.get_parameter('turn_left_cmd_steering').get_parameter_value().integer_value
        self.GAIN = self.get_parameter('gain').get_parameter_value().double_value

    def __timer_callback(self) -> None:
        # If there are no left and right lanes, stop the car.
        if self.lane_coeffs['left'].data == self.ZERO_COEFFS.data and \
           self.lane_coeffs['right'].data == self.ZERO_COEFFS.data:
            cmd_steering = 0
            cmd_speed = 0
        else:
            steering_angle = self.steering_angle.data * self.GAIN
            cmd_steering = self.steering_angle_to_cmd_steering(steering_angle)
            cmd_speed = self.steering_angle_to_cmd_speed(steering_angle)

        self.publish_cmd_steering(cmd_steering)
        self.publish_cmd_speed(cmd_speed)

    def __steering_angle_callback(self, msg: Float32) -> None:
        self.steering_angle = msg

    def __left_lane_coeff_callback(self, msg: Coefficients) -> None:
        self.lane_coeffs['left'] = msg

    def __right_lane_coeff_callback(self, msg: Coefficients) -> None:
        self.lane_coeffs['right'] = msg

    def steering_angle_to_cmd_steering(self, steering_angle: float) -> int:
        return int((steering_angle - 1.0) *
                   (self.TURN_RIGHT_CMD_STEERING - self.TURN_LEFT_CMD_STEERING) /
                   (-1.0 - 1.0) + self.TURN_LEFT_CMD_STEERING)

    def steering_angle_to_cmd_speed(self, steering_angle: float) -> int:
        k = 1 if steering_angle < 0 else -1
        return int(k * (self.MAX_SPEED - self.MIN_SPEED) * steering_angle / 1.0 + self.MAX_SPEED)

    def publish_cmd_steering(self, cmd_steering: int) -> None:
        self.cmd_steering.data = cmd_steering
        self.cmd_steering_publisher.publish(self.cmd_steering)

    def publish_cmd_speed(self, cmd_speed: int) -> None:
        self.cmd_speed.data = cmd_speed
        self.cmd_speed_publisher.publish(self.cmd_speed)


def main(args=None):
    rclpy.init(args=args)
    node = CoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
