#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "can_interface/controlcan.h"

using namespace std::chrono_literals;

class CANInterface : public rclcpp::Node
{
  public:
    CANInterface () : Node("can_interface")
    {
      // publisher_ = this->create_publisher<std_msgs::msg::Int16>("servo", 10);
      this->Timer_ = this->create_wall_timer(500ms, std::bind(&CANInterface::TimerCallback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr Timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void TimerCallback ()
    {
      
    }
};

int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CANInterface>());
  rclcpp::shutdown();
  return 0;
}
