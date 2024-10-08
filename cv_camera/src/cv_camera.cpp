/*
Author: Sippawit Thammawiset
Date: August 06, 2024.
File: cv_camera.cpp
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> Node = rclcpp::Node::make_shared("cv_camera");
  image_transport::ImageTransport ImageTransport(Node);
  image_transport::Publisher ImagePublisher = ImageTransport.advertise("video_raw", 1);
  rclcpp::Rate LoopRate(33.33);

  const std::string SourceString(argv[1]);

  cv::VideoCapture Capture(SourceString, cv::CAP_GSTREAMER);

  if (!Capture.isOpened())
  {
    Capture.release();
    RCLCPP_INFO(Node->get_logger(), "Could not open camera.");

    return 1;
  }

  cv::Mat Frame;
  sensor_msgs::msg::Image::SharedPtr ImageMessage;

  while (rclcpp::ok())
  {
    Capture >> Frame;

    if (!Frame.empty())
    {
      ImageMessage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", Frame).toImageMsg();
      ImagePublisher.publish(ImageMessage);
    }

    rclcpp::spin_some(Node);
    LoopRate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
