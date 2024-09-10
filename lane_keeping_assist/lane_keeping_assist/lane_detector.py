"""
Author: Sippawit Thammawiset
Date: September 5, 2024.
File: lane_detector.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_msgs.msg import Coefficients
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from onnx_inference import ONNXInference
from .utils import colors
import numpy as np
import cv2
import os


class LaneDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('lane_detector')

        self.__declare_parameters()
        self.__get_parameters()

        # Publishers
        self.lane_coeffs_publisher = {
            'left': self.create_publisher(Coefficients, 'lane_detector/left_lane/coefficients', 10),
            'right': self.create_publisher(Coefficients, 'lane_detector/right_lane/coefficients', 10)
        }
        self.debug_image_publisher = self.create_publisher(Image, 'lane_detector/debug_image', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(Image, self.IMAGE_TOPIC, self.__image_callback, 10)

        # Messages
        self.lane_coeffs = {
            'left': Coefficients(),
            'right': Coefficients()
        }
        self.debug_image = Image()

        self.bridge = CvBridge()
        self.model = ONNXInference(os.path.join(get_package_share_directory('lane_keeping_assist'),
                                                'share', 'models', 'lane_detector-512x256-rgb8-onnx.onnx'),
                                   'input_1',
                                   ['bin', 'inst'],
                                   ['CUDAExecutionProvider', 'CPUExecutionProvider'],
                                   'LaneDetector')
        # self.M = np.float32([
        #     [-3.45040403e-01, -3.04785699e+00, 3.45629957e+02],
        #     [-1.27798502e-16, -3.69174048e+00, 4.20834220e+02],
        #     [0.00000000e+00, -1.19056907e-02, 1.00000000e+00]
        # ])

        self.get_logger().info(
            f'{colors.OKGREEN}'
            f'> Initialized {self.model} without any errors. Waiting for image...'
            f'{colors.ENDC}'
        )

    def __declare_parameters(self) -> None:
        self.declare_parameter('image_topic', 'image_raw')

    def __get_parameters(self) -> None:
        self.IMAGE_TOPIC = self.get_parameter('image_topic').get_parameter_value().string_value

    def __image_callback(self, msg) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        resized_frame = cv2.resize(frame, (512, 256))

        X = np.expand_dims(resized_frame, axis=0)

        bin_pred, inst_pred = self.model.predict(X)

        # bin_pred_thresh = (bin_pred >= 0.5).astype(np.uint8) * 255
        inst_pred_thresh = (inst_pred >= 0.6).astype(np.uint8) * 255

        left_lane_mask = inst_pred_thresh[0, :, :, 1]
        right_lane_mask = inst_pred_thresh[0, :, :, 2]

        left_lane_u, left_lane_v, _, _ = self.find_lane_px(left_lane_mask)
        _, _, right_lane_u, right_lane_v = self.find_lane_px(right_lane_mask)

        try:
            left_lane_v_sample = np.linspace(left_lane_v.min(), left_lane_v.max(), int(0.005 * len(left_lane_v)))
        except ValueError:
            left_lane_v_sample = []

        try:
            right_lane_v_sample = np.linspace(right_lane_v.min(), right_lane_v.max(), int(0.005 * len(right_lane_v)))
        except ValueError:
            right_lane_v_sample = []

        try:
            left_lane_coeffs = np.polyfit(left_lane_v, left_lane_u, 3)
            left_lane_fit_u = np.poly1d(left_lane_coeffs)(left_lane_v_sample)

            for u, v in zip(left_lane_fit_u, left_lane_v_sample):
                center_coordinates = (int(u), int(v))
                radius = 3
                color = (255, 0, 0)
                thickness = -1
                cv2.circle(resized_frame, center_coordinates, radius, color, thickness)

            cv2.putText(resized_frame,
                        'Left lane coeffs: {}.'.format(left_lane_coeffs),
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
        except TypeError:
            left_lane_coeffs = np.zeros((4,))

            cv2.putText(resized_frame,
                        'Left lane coeffs: Failed to fit',
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 0), 1)

        try:
            right_lane_coeffs = np.polyfit(right_lane_v, right_lane_u, 3)
            right_lane_fit_u = np.poly1d(right_lane_coeffs)(right_lane_v_sample)

            for u, v in zip(right_lane_fit_u, right_lane_v_sample):
                center_coordinates = (int(u), int(v))
                radius = 3
                color = (0, 0, 255)
                thickness = -1
                cv2.circle(resized_frame, center_coordinates, radius, color, thickness)

            cv2.putText(resized_frame,
                        'Right lane coeffs: {}.'.format(right_lane_coeffs),
                        (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
        except TypeError:
            right_lane_coeffs = np.zeros((4,))

            cv2.putText(resized_frame,
                        'Right lane coeffs: Failed to fit',
                        (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 0), 1)

        # left_lane_stack = np.stack((left_lane_fit_u, left_lane_v_sample, np.ones(len(left_lane_fit_u))), axis=0)
        # right_lane_stack = np.stack((right_lane_fit_u, right_lane_v_sample, np.ones(len(right_lane_fit_u))), axis=0)
        #
        # left_lane_tf = self.M @ left_lane_stack
        # left_lane_tf /= left_lane_tf[2]
        # right_lane_tf = self.M @ right_lane_stack
        # right_lane_tf /= right_lane_tf[2]
        #
        # birdeye = np.zeros_like(resized_frame)
        # for i in range(left_lane_tf.shape[1]):
        #     center_coordinates = (int(left_lane_tf[0, i]), int(left_lane_tf[1, i]))
        #     radius = 2
        #     color = (255, 255, 255)
        #     thickness = -1
        #     cv2.circle(birdeye, center_coordinates, radius, color, thickness)
        # for i in range(right_lane_tf.shape[1]):
        #     center_coordinates = (int(right_lane_tf[0, i]), int(right_lane_tf[1, i]))
        #     radius = 2
        #     color = (255, 255, 255)
        #     thickness = -1
        #     cv2.circle(birdeye, center_coordinates, radius, color, thickness)

        self.publish_lane_coeffs(left_lane_coeffs, right_lane_coeffs)
        self.publish_debug_image(resized_frame)

        # debug_image = self.bridge.cv2_to_imgmsg(birdeye, 'rgb8')
        # self.birdeye_publisher.publish(debug_image)
        #
        # path = Path()
        # path.header.stamp = self.get_clock().now().to_msg()
        # path.header.frame_id = 'base_link'
        # for i in range(left_lane_tf.shape[1]):
        #     pose = PoseStamped()
        #     pose.pose.position.x = (left_lane_tf[0, i]) * 0.05
        #     pose.pose.position.y = (left_lane_tf[1, i]) * 0.05
        #     path.poses.append(pose)
        # self.left_bound_publisher.publish(path)
        #
        # path = Path()
        # path.header.stamp = self.get_clock().now().to_msg()
        # path.header.frame_id = 'base_link'
        # for i in range(right_lane_tf.shape[1]):
        #     pose = PoseStamped()
        #     pose.pose.position.x = (right_lane_tf[0, i]) * 0.05
        #     pose.pose.position.y = (right_lane_tf[1, i]) * 0.05
        #     path.poses.append(pose)
        # self.right_bound_publisher.publish(path)

    @staticmethod
    def find_lane_px(image: np.ndarray,
                     n_windows: int = 9,
                     margin: int = 100,
                     min_px: int = 50) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(image[image.shape[0] // 2:, :], axis=0)

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int32(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = n_windows
        # Set the width of the windows +/- margin
        margin = margin
        # Set minimum number of pixels found to recenter window
        minpix = min_px

        # Set height of windows - based on nwindows above and image shape
        window_height = np.int32(image.shape[0] // nwindows)
        # Identify the x and v positions of all nonzero pixels in the image
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and v (and right and left)
            win_y_low = image.shape[0] - (window + 1) * window_height
            win_y_high = image.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # Identify the nonzero pixels in x and v within the window #
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass

        # Extract left and right line pixel positions
        left_lane_uw = nonzerox[left_lane_inds]
        left_lane_v = nonzeroy[left_lane_inds]
        right_lane_uw = nonzerox[right_lane_inds]
        right_lane_v = nonzeroy[right_lane_inds]

        return left_lane_uw, left_lane_v, right_lane_uw, right_lane_v

    def publish_lane_coeffs(self, left_lane_coeffs: np.ndarray, right_lane_coeffs: np.ndarray) -> None:
        self.lane_coeffs['left'].data = [coeff for coeff in left_lane_coeffs]
        self.lane_coeffs['right'].data = [coeff for coeff in right_lane_coeffs]

        self.lane_coeffs_publisher['left'].publish(self.lane_coeffs['left'])
        self.lane_coeffs_publisher['right'].publish(self.lane_coeffs['right'])

    def publish_debug_image(self, debug_image: np.ndarray) -> None:
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
