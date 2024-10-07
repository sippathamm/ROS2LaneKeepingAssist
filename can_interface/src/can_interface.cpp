#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "can_interface/controlcan.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CANInterface : public rclcpp::Node
{
  public:
    CANInterface () : Node("can_interface")
    {
      // Timers
      this->CANTransmitTimer_ = this->create_wall_timer(0.0167ms, [this] { CANTransmitTimerCallback(); });
      this->CANReceiveTimer_ = this->create_wall_timer(10ms, [this] { CANReceiveTimerCallback(); });

      // Publishers
      this->FBKSteeringAnglePublisher_ = this->create_publisher<std_msgs::msg::Float32>("feedback/steering", 10);
      this->FBKVelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>("feedback/velocity", 10);

      // Subscribers
      this->CMDSteeringSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("cmd_steering",10,
        std::bind(&CANInterface::CMDSteeringCallback, this, _1));
      this->CMDVelocitySubscriber_ = this->create_subscription<std_msgs::msg::Float32>("cmd_velocity",10,
        std::bind(&CANInterface::CMDVelocityCallback, this, _1));

      this->CANInitialize();
      this->CANTransmitSetup();
      this->CANReceiveSetup();
    }

    ~CANInterface () override
    {
      VCI_ResetCAN(VCI_USBCAN1, 0, 0);
      VCI_CloseDevice(VCI_USBCAN1,0);
    }

  private:
    rclcpp::TimerBase::SharedPtr CANTransmitTimer_;
    rclcpp::TimerBase::SharedPtr CANReceiveTimer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr FBKSteeringAnglePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr FBKVelocityPublisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr CMDSteeringSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr CMDVelocitySubscriber_;
    std_msgs::msg::Float32 CMDSteering_;
    std_msgs::msg::Float32 CMDVelocity_;
    std_msgs::msg::Float32 FeedbackSteeringAngleMsg_;
    std_msgs::msg::Float32 FeedbackVelocityMsg_;
    float FeedbackSteeringAngle_ = 0.0;
    float FeedbackVelocity_ = 0.0;
    unsigned char TxBuffer_[8] = {};
    unsigned char RxBuffer_[8] = {};
    VCI_CAN_OBJ CANTransmit_[1] {};
    VCI_CAN_OBJ CANReceive_[1] {};

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

    void CANTransmitSetup ()
    {
      this->CANTransmit_[0].ID = 0x202;
      this->CANTransmit_[0].SendType = 0;
      this->CANTransmit_[0].RemoteFlag = 0;
      this->CANTransmit_[0].ExternFlag = 0;
      this->CANTransmit_[0].DataLen = 8;
    }

    void CANReceiveSetup ()
    {
      this->CANReceive_[0].ID = 0x182;
      this->CANReceive_[0].SendType = 1;
      this->CANReceive_[0].RemoteFlag = 0;
      this->CANReceive_[0].ExternFlag = 0;
      this->CANReceive_[0].DataLen = 8;
    }

    void CANTransmitTimerCallback ()
    {
      float CMDSteering = this->CMDSteering_.data;
      float CMDVelocity = this->CMDVelocity_.data;
      const auto *CMDSteeringPointer = reinterpret_cast<unsigned char*>(&CMDSteering);
      const auto *CMDVelocityPointer = reinterpret_cast<unsigned char*>(&CMDVelocity);
      this->TxBuffer_[0] = CMDVelocityPointer[0];
      this->TxBuffer_[1] = CMDVelocityPointer[1];
      this->TxBuffer_[2] = CMDVelocityPointer[2];
      this->TxBuffer_[3] = CMDVelocityPointer[3];
      this->TxBuffer_[4] = CMDSteeringPointer[0];
      this->TxBuffer_[5] = CMDSteeringPointer[1];
      this->TxBuffer_[6] = CMDSteeringPointer[2];
      this->TxBuffer_[7] = CMDSteeringPointer[3];

      for (int i = 0; i < 8; i++)
      {
        this->CANTransmit_[0].Data[i] = this->TxBuffer_[i];
      }

      if (VCI_Transmit(VCI_USBCAN1, 0, 0, this->CANTransmit_, 1))
      {
        // RCLCPP_INFO(this->get_logger(), "\n"
        //                                 "       > CMD Steering: %f\n"
        //                                 "       > CMD Velocity: %f",
        //                                 CMDSteering,
        //                                 CMDVelocity);
        // RCLCPP_INFO(this->get_logger(), "> Transmit OK.");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "> Transmit failed.");
      }
    }

    void CANReceiveTimerCallback ()
    {
      if (VCI_Receive(VCI_USBCAN1, 0, 0, this->CANReceive_, 1, 5))
      {
        ClearBuffer(this->RxBuffer_);

        // RCLCPP_INFO(this->get_logger(), "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
        //   this->CANReceive_[0].Data[0],
        //   this->CANReceive_[0].Data[1],
        //   this->CANReceive_[0].Data[2],
        //   this->CANReceive_[0].Data[3],
        //   this->CANReceive_[0].Data[4],
        //   this->CANReceive_[0].Data[5],
        //   this->CANReceive_[0].Data[6],
        //   this->CANReceive_[0].Data[7]
        //   );

        this->RxBuffer_[0] = this->CANReceive_[0].Data[0];
        this->RxBuffer_[1] = this->CANReceive_[0].Data[1];
        this->RxBuffer_[2] = this->CANReceive_[0].Data[2];
        this->RxBuffer_[3] = this->CANReceive_[0].Data[3];
        this->RxBuffer_[4] = this->CANReceive_[0].Data[4];
        this->RxBuffer_[5] = this->CANReceive_[0].Data[5];
        this->RxBuffer_[6] = this->CANReceive_[0].Data[6];
        this->RxBuffer_[7] = this->CANReceive_[0].Data[7];

        // this->FeedbackSteeringAngle_ = *(float *) &this->RxBuffer_[4];
        // this->FeedbackVelocity_ = *(float *) &this->RxBuffer_[0];

        this->FeedbackSteeringAngle_ = *reinterpret_cast<float*>(&this->RxBuffer_[4]);
        this->FeedbackVelocity_ = *reinterpret_cast<float*>(&this->RxBuffer_[0]);

        RCLCPP_INFO(this->get_logger(), "\n"
                                        "       > Feedback steering angle [deg]: %f\n"
                                        "       > Feedback speed [m/s]: %f",
                                        this->FeedbackSteeringAngle_,
                                        this->FeedbackVelocity_);


        this->FeedbackSteeringAngleMsg_.data = this->FeedbackSteeringAngle_;
        this->FeedbackVelocityMsg_.data = this->FeedbackVelocity_;

        this->FBKSteeringAnglePublisher_->publish(FeedbackSteeringAngleMsg_);
        this->FBKVelocityPublisher_->publish(FeedbackVelocityMsg_);
      }
    }

    static void ClearBuffer (unsigned char *Buffer)
    {
      for (int i = 0; i < 8; i++) { Buffer[i] = 0; }
    }

    void CMDSteeringCallback (const std_msgs::msg::Float32 &Message)
    {
      this->CMDSteering_ = Message;
    }

    void CMDVelocityCallback (const std_msgs::msg::Float32 &Message)
    {
      this->CMDVelocity_ = Message;
    }
};

int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CANInterface>());
  rclcpp::shutdown();
  return 0;
}
