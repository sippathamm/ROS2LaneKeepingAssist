from setuptools import find_packages, setup
import glob
import os

package_name = 'lane_keeping_assist'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '/utils', package_name + '/models'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'share', 'images'), glob.glob('share/images/*.png')),
        (os.path.join('share', package_name, 'share', 'test_videos'), glob.glob('share/test_videos/*.mp4')),
        (os.path.join('share', package_name, 'share', 'models'), glob.glob('share/models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'core_node = lane_keeping_assist.core_node:main',
            'lane_detector_node = lane_keeping_assist.lane_detector_node:main',
            'steering_predictor_node = lane_keeping_assist.steering_predictor_node:main',
            'test_video_publisher_node = lane_keeping_assist.test_video_publisher_node:main',
        ],
    },
)
