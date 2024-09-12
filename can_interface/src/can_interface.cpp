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
      this->Timer_ = this->create_wall_timer(500ms, std::bind(&CANInterface::TimerCallback, this));

      this->CANInitialize();
    }

    ~CANInterface () override
    {
      VCI_ResetCAN(VCI_USBCAN1, 0, 0);
      VCI_CloseDevice(VCI_USBCAN1,0);
    }

  private:
    rclcpp::TimerBase::SharedPtr Timer_;

    void TimerCallback ()
    {
      VCI_CAN_OBJ CANSend[1];
      CANSend[0].ID = 0x201;
      CANSend[0].SendType = 0;
      CANSend[0].RemoteFlag = 0;
      CANSend[0].ExternFlag = 0;
      CANSend[0].DataLen = 8;
      CANSend[0].Data[0] = 123;
      CANSend[0].Data[1] = 0;
      CANSend[0].Data[2] = 0;
      CANSend[0].Data[3] = 0;
      CANSend[0].Data[4] = 0;
      CANSend[0].Data[5] = 0;
      CANSend[0].Data[6] = 0;
      CANSend[0].Data[7] = 0;

      if (VCI_Transmit(VCI_USBCAN1, 0, 0, CANSend, 1))
      {
        RCLCPP_INFO(this->get_logger(), "> Sent OK.");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "> Sent failed.");
      }
    }

    void CANInitialize () const
    {
      if (VCI_OpenDevice(VCI_USBCAN1, 0, 0) == 1)
      {
        RCLCPP_INFO(this->get_logger(), "> Open CAN1 success.");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "> Failed to open CAN1.");
        exit(1);
      }

      VCI_BOARD_INFO CANInfo;
      if(VCI_ReadBoardInfo(VCI_USBCAN1,0, &CANInfo) == 1)
      {
        RCLCPP_INFO(this->get_logger(), "> Serial number: %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
          CANInfo.str_Serial_Num[0],
          CANInfo.str_Serial_Num[1],
          CANInfo.str_Serial_Num[2],
          CANInfo.str_Serial_Num[3],
          CANInfo.str_Serial_Num[4],
          CANInfo.str_Serial_Num[5],
          CANInfo.str_Serial_Num[6],
          CANInfo.str_Serial_Num[7],
          CANInfo.str_Serial_Num[8],
          CANInfo.str_Serial_Num[9],
          CANInfo.str_Serial_Num[10],
          CANInfo.str_Serial_Num[11],
          CANInfo.str_Serial_Num[12],
          CANInfo.str_Serial_Num[13],
          CANInfo.str_Serial_Num[14],
          CANInfo.str_Serial_Num[15],
          CANInfo.str_Serial_Num[16],
          CANInfo.str_Serial_Num[17],
          CANInfo.str_Serial_Num[18],
          CANInfo.str_Serial_Num[19]);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "> Failed to get CAN1 info.");
      }

      VCI_INIT_CONFIG CANConfig;
      CANConfig.AccCode = 0;
      CANConfig.AccMask = 0xFFFFFFFF;
      CANConfig.Filter = 1;
      CANConfig.Timing0 = 0x00;
      CANConfig.Timing1 = 0x1C;
      CANConfig.Mode = 0;

      if (VCI_InitCAN(VCI_USBCAN1, 0, 0, &CANConfig) != 1)
      {
        RCLCPP_INFO(this->get_logger(), "> Failed to initialize CAN1.");
      }

      if (VCI_StartCAN(VCI_USBCAN1,0,0) !=1 )
      {
        RCLCPP_INFO(this->get_logger(), "> Failed to start CAN1.");
        VCI_CloseDevice(VCI_USBCAN1,0);
      }
    }
};

int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CANInterface>());
  rclcpp::shutdown();
  return 0;
}
