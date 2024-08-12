/*
Author: Sippawit Thammawiset
Date: 06.08.2024
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
 
using namespace std::chrono_literals;
 
class StreamPublisherVideo : public rclcpp::Node 
{
public:
  StreamPublisherVideo() : Node("stream_publisher_video")
  {
    this->declare_parameter("stream_source_path", "/home");
    this->declare_parameter("silence", false);

    const std::string StreamSourcePath = this->get_parameter("stream_source_path").as_string();
    this->Silence_ = this->get_parameter("silence").as_bool();
    
    this->VideoCapture_ = cv::VideoCapture(StreamSourcePath);
    
    if (!this->VideoCapture_.isOpened())
    {
      throw std::runtime_error("Unable to open stream");
    }
    
    this->VideoCapture_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    this->VideoCapture_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    this->StreamPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("raw_stream", 10);
    this->Timer_ = this->create_wall_timer(33.33ms, std::bind(&StreamPublisherVideo::TimerCallback, this));
  }
 
private:
  cv::VideoCapture VideoCapture_;
  rclcpp::TimerBase::SharedPtr Timer_;
  sensor_msgs::msg::Image::SharedPtr ImageMessage_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr StreamPublisher_;
  bool Silence_;

  void TimerCallback()
  {
    cv::Mat Frame;
    this->VideoCapture_ >> Frame;
    
    if (Frame.empty())
    {
      throw std::runtime_error("Unable to read stream. Terminated.");
    }
    
    this->ImageMessage_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", Frame).toImageMsg();
 
    this->StreamPublisher_->publish(*this->ImageMessage_.get());
    
    if (!this->Silence_)
    {
      RCLCPP_INFO(this->get_logger(), "Stream is being published.");
    }
  }
};
 
int main (int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StreamPublisherVideo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

