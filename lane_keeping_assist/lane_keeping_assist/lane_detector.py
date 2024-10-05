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
                                            'lane_detector-512x256-rgb8-onnx.onnx'))
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

        bin_pred, inst_pred = self.model.predict(X)  # Expensive!

        # bin_pred_thresh = (bin_pred >= 0.5).astype(np.uint8)
        inst_pred_thresh = (inst_pred >= 0.5).astype(np.uint8)

        left_lane_mask = inst_pred_thresh[0, :, :, 1]
        right_lane_mask = inst_pred_thresh[0, :, :, 2]

        # TODO: Optimize find_mask_px(...) function
        left_lane_u, left_lane_v, _, _ = self.find_mask_px(left_lane_mask)
        _, _, right_lane_u, right_lane_v = self.find_mask_px(right_lane_mask)

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
    def find_mask_px(image: np.ndarray,
                     n_windows: int = 9,
                     margin: int = 100,
                     min_px: int = 50) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray):
        """
        Legacy version
        """
        
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
