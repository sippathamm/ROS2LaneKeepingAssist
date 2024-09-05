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
      VCI_CAN_OBJ DataOut[1];
      send[0].ID = 0x201; //65
      send[0].SendType = 0;
      send[0].RemoteFlag = 0;
      send[0].ExternFlag = 0;
      send[0].DataLen = 8;
      send[0].Data[0] = p_rpm1[0];
      send[0].Data[1] = p_rpm1[1];
      send[0].Data[2] = p_rpm1[2];
      send[0].Data[3] = p_rpm1[3];

      send[0].Data[4] = p_rpm2[0];
      send[0].Data[5] = p_rpm2[1];
      send[0].Data[6] = p_rpm2[2];
      send[0].Data[7] = p_rpm2[3];

      VCI_Transmit(VCI_USBCAN2, 0, 0, DataOut, 1);
      RCLCPP_INFO(this->get_logger(), "sent");
    }
};

int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CANInterface>());
  rclcpp::shutdown();
  return 0;
}
