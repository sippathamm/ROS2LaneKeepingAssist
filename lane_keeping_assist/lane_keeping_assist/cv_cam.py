import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .utils import colors
import cv2


class CVCam(Node):
    def __init__(self):
        super().__init__('cv_cam')

        self.capture = cv2.VideoCapture(0)

        if not self.capture.isOpened():
            raise RuntimeError(
                f'{colors.ERROR}'
                f'[ERROR] Could not open video.'
                f'{colors.ENDC}'
            )

        self.video_publisher = self.create_publisher(Image, 'image_raw', 10)
        timer_period = 1 / self.capture.get(cv2.CAP_PROP_FPS)
        self.timer = self.create_timer(timer_period, self.__timer_callback)
        self.bridge = CvBridge()

    def __timer_callback(self):
        _, frame = self.capture.read()

        if not _:
            self.capture.release()

            raise RuntimeError(
                f'{colors.ERROR}'
                f'[ERROR] Could not read frame.'
                f'{colors.ENDC}'
            )

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        output_frame = self.bridge.cv2_to_imgmsg(rgb_frame, 'rgb8')
        self.video_publisher.publish(output_frame)


def main(args=None):
    rclpy.init(args=args)
    node = CVCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
