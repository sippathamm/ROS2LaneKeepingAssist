
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


#include "cpp_pubsub/controlcan.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define SDO_ID2  0x601

#define samplingHz 100
#define Kp 2.0
#define Ki 0.1
#define Kd 0.02

#define d2check 1
#define score_threshold 2.5
#define vel_max 10.0

#define offset 30
#define center_point 512
#define max_angle 0.5
#define max_int_from_center 384

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class CanInterface : public rclcpp::Node
{
public:

  int send_frame = 0;
  int receive_frame = 0;
  int receive_frame1 = 0;
  int receive_frame2 = 0;

  int can_lost =0;

  int alarm_flag=0;

  float vel_linear_x =0.0;
  float vel_angular_z = 0.0;

  int fb_rpm1;
  int fb_rpm2;
  int fb_on;
  float fb_volt2;
  float fb_status2;
  float fb_fault_code1;
  float fb_fault_code2;

  float wheel_dist = 0.6;
  float pi = 3.1415926535;
  float diameter = 0.165;
  

  std_msgs::msg::Int8 can_status;
  rclcpp::Time current_time, last_time ,init_time ,current_init_time;
  
  geometry_msgs::msg::Twist encoder;
  geometry_msgs::msg::Twist status;

  rclcpp::TimerBase::SharedPtr recieve_loop_timer_, can_recieve_loop_timer_,send_loop_timer_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr encoder_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr can_stat_pub_;
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_command_;
  size_t count_;

  VCI_CAN_OBJ receive[1];
  VCI_CAN_OBJ receive1[1];
    



  CanInterface(): Node("can_interface"), count_(0)
  {
    // pub = this->create_publisher<std_msgs::msg::String>("/topic", 10);
    encoder_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/encoder_count", 1000);
    status_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/roboteq_status", 1000);
    can_stat_pub_ = this->create_publisher<std_msgs::msg::Int8>("/can_status", 1000);

    
    
    recieve_loop_timer_ = this->create_wall_timer(20ms, std::bind(&CanInterface::Receive_encoder, this));
	can_recieve_loop_timer_ = this->create_wall_timer(5ms, std::bind(&CanInterface::CAN_receive, this));
	
	
    // vehicle_cmd_timer_ = rclcpp::create_timer(this, get_clock(), 20ms, std::bind(&CanInterface::publishVehicleCmd, this));
    sub_vel_command_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,std::bind(&CanInterface::cmd_callback,this, _1));

    // sub_alarm = this->create_subscription<geometry_msgs::msg::Twist>("alarm",10,std::bind(&CanInterface::cmd_callback,this, _1));

    CAN_init();
	CAN_receive_setup();
    // CAN_send_cmd();
    // CAN_receive();
    // Receive_encoder();
    // Time_out();
    Initialize_Motor();
    zltech_TPDO1();
    zltech_TPDO2();
    zltech_RPDO1();
    Switch_NMT();

	send_loop_timer_ = this->create_wall_timer(20ms, std::bind(&CanInterface::CAN_send_cmd, this));

  }

  void CAN_init()
  {

    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
      RCLCPP_INFO(this->get_logger(),">>open deivce success!\n");//打开设备成功
    }else
    {
      RCLCPP_INFO(this->get_logger(),">>open deivce error!\n");
      exit(1);
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//接收所有帧
    config.Timing0 = 0x00;/*波特率500 Kbps  0x00  0x1C*/
    config.Timing1 = 0x1C;
    config.Mode = 0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
    {
      printf(">>Init CAN1 error\n");
      VCI_CloseDevice(VCI_USBCAN2,0);
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
      printf(">>Start CAN1 error\n");
      VCI_CloseDevice(VCI_USBCAN2,0);
    }
  }     

  //----------------------------------------------------------------
  void CAN_receive_setup()
  {
	
    receive[0].ID = 0x181;
    receive[0].SendType = 1;
    receive[0].RemoteFlag = 0;
    receive[0].ExternFlag = 0;
    receive[0].DataLen = 8;

	
    receive1[0].ID = 0x381;
    receive1[0].SendType = 1;
    receive1[0].RemoteFlag = 0;
    receive1[0].ExternFlag = 0;
    receive1[0].DataLen = 8;
  }

  void CAN_send_cmd()
  {
  
    int lin_x = (int)(vel_linear_x);
    int ang_z = (int)(vel_angular_z);

    //  ROS_INFO("X_a[k]:[%d %d]",lin_x,ang_z);
    // RCLCPP_INFO(this->get_logger(),"X_a[k]:[%d %d]",lin_x,ang_z);

	int rpm1 = (-lin_x - (ang_z*wheel_dist/2))*0.6/(pi*diameter);
	int rpm2 = (lin_x - (ang_z*wheel_dist/2))*0.6/(pi*diameter);
	RCLCPP_INFO(this->get_logger(),"rpm:[%d %d]",rpm1,rpm2);

    unsigned char* p_rpm1;
    unsigned char* p_rpm2;

    p_rpm1 = (unsigned char*)&rpm1;
    p_rpm2 = (unsigned char*)&rpm2;

	VCI_CAN_OBJ send[1];
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

    // send[0].Data[4] = p_ang_z[0];
    // send[0].Data[5] = p_ang_z[1];
    // send[0].Data[6] = p_ang_z[2];
    // send[0].Data[7] = p_ang_z[3];
	if(fb_on == 0){
		auto send_cmd_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
	}
    

  }
	
  void CAN_receive()
  {
		receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
		// receive_frame1 = VCI_Receive(VCI_USBCAN2, 0, 0, receive1, 1, 5);

    unsigned char* p_fb_on;
    p_fb_on = (unsigned char*)&fb_on;

    int fb_encoder1;
    int fb_encoder2;

    unsigned char* p_fb_encoder1;
    unsigned char* p_fb_encoder2;

    p_fb_encoder1 = (unsigned char*)&fb_encoder1;
    p_fb_encoder2 = (unsigned char*)&fb_encoder2;




    unsigned char* p_fb_rpm1;
    unsigned char* p_fb_rpm2;

    p_fb_rpm1 = (unsigned char*)&fb_rpm1;
    p_fb_rpm2 = (unsigned char*)&fb_rpm2;

    unsigned short fb_volt1;
    unsigned short fb_volt2;

    unsigned char* p_fb_volt1;
    unsigned char* p_fb_volt2;

    p_fb_volt1 =(unsigned char*)&fb_volt1;
    p_fb_volt2 =(unsigned char*)&fb_volt2;

    unsigned short fb_status1;
    unsigned short fb_status2;

    unsigned char* p_fb_status1;
    unsigned char* p_fb_status2;

    p_fb_status1 = (unsigned char*)&fb_status1;
    p_fb_status2 = (unsigned char*)&fb_status2;

    
    unsigned short fb_fault1;
    unsigned short fb_fault2;

    unsigned char* p_fb_fault1;
    unsigned char* p_fb_fault2;

    p_fb_fault1 = (unsigned char*)&fb_fault1;
    p_fb_fault2 = (unsigned char*)&fb_fault2;

    unsigned short fb_fault_code1;
    unsigned short fb_fault_code2;

    unsigned char* p_fb_fault_code1;
    unsigned char* p_fb_fault_code2;

	

    p_fb_fault_code1 = (unsigned char*)&fb_fault_code1;
    p_fb_fault_code2 = (unsigned char*)&fb_fault_code2;

		// RCLCPP_INFO(this->get_logger(), "can receive: %d %d",receive_frame,receive_frame1);

      if(receive_frame==1)
      {
        switch(receive[0].ID)
        {
          case 0x181:	    
            p_fb_rpm1[0] = receive[0].Data[0];
            p_fb_rpm1[1] = receive[0].Data[1];
            p_fb_rpm1[2] = receive[0].Data[2];
            p_fb_rpm1[3] = receive[0].Data[3];

            p_fb_rpm2[0] = receive[0].Data[4];
            p_fb_rpm2[1] = receive[0].Data[5];
            p_fb_rpm2[2] = receive[0].Data[6];
            p_fb_rpm2[3] = receive[0].Data[7];
          break;


          case 0x381:
                p_fb_volt2[0] = receive[0].Data[0];
                p_fb_volt2[1] = receive[0].Data[1];
                p_fb_status2[0] = receive[0].Data[2];
                p_fb_status2[1] = receive[0].Data[3];

                p_fb_fault_code1[0] = receive[0].Data[4];
                p_fb_fault_code1[1] = receive[0].Data[5];
                p_fb_fault_code2[0] = receive[0].Data[6];
                p_fb_fault_code2[1] = receive[0].Data[7];
          break;

		  case 0x091:
                p_fb_on[0] = receive[0].Data[0];
				p_fb_on[1] = receive[0].Data[1];
				p_fb_on[2] = receive[0].Data[2];
				p_fb_on[3] = receive[0].Data[3];
                // p_fb_volt2[1] = receive[0].Data[1];
                // p_fb_status2[0] = receive[0].Data[2];
                // p_fb_status2[1] = receive[0].Data[3];

                // p_fb_fault_code1[0] = receive[0].Data[4];
                // p_fb_fault_code1[1] = receive[0].Data[5];
                // p_fb_fault_code2[0] = receive[0].Data[6];
                // p_fb_fault_code2[1] = receive[0].Data[7];
          break;
        }
				// if(receive[0].ID == 0x091){
        // RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %d %d %d %d %d %d %d]",receive[0].ID,fb_rpm1,fb_rpm2,fb_volt2,fb_status2,fb_fault_code1,fb_fault_code2,fb_on);
		// }
        // ROS_INFO("X_a[k]:[%x %d %d %d %d %d %d]",receive[0].ID,fb_rpm1,fb_rpm2,fb_volt2,fb_status2,fb_fault_code1,fb_fault_code2);

      }

      if(receive_frame1==1)
        {
          switch(receive1[0].ID)
          {
            case 0x181:	    
              p_fb_rpm1[0] = receive1[0].Data[0];
              p_fb_rpm1[1] = receive1[0].Data[1];
              p_fb_rpm1[2] = receive1[0].Data[2];
              p_fb_rpm1[3] = receive1[0].Data[3];

              p_fb_rpm2[0] = receive1[0].Data[4];
              p_fb_rpm2[1] = receive1[0].Data[5];
              p_fb_rpm2[2] = receive1[0].Data[6];
              p_fb_rpm2[3] = receive1[0].Data[7];
            break;

            case 0x381:
              p_fb_volt2[0] = receive1[0].Data[0];
              p_fb_volt2[1] = receive1[0].Data[1];
              p_fb_status2[0] = receive1[0].Data[2];
              p_fb_status2[1] = receive1[0].Data[3];

              p_fb_fault_code1[0] = receive1[0].Data[4];
              p_fb_fault_code1[1] = receive1[0].Data[5];
              p_fb_fault_code2[0] = receive1[0].Data[6];
              p_fb_fault_code2[1] = receive1[0].Data[7];
            break;

			case 0x091:
                p_fb_on[0] = receive1[0].Data[0];
                // p_fb_volt2[1] = receive[0].Data[1];
                // p_fb_status2[0] = receive[0].Data[2];
                // p_fb_status2[1] = receive[0].Data[3];

                // p_fb_fault_code1[0] = receive[0].Data[4];
                // p_fb_fault_code1[1] = receive[0].Data[5];
                // p_fb_fault_code2[0] = receive[0].Data[6];
                // p_fb_fault_code2[1] = receive[0].Data[7];
          break;

            if(receive_frame != 1 && receive_frame1 != 1 )
            {

            can_lost++;
              if(can_lost >= 5)
              {
                  receive[0].Data[0] = 0;
                  receive[0].Data[1] = 0;
                  receive[0].Data[2] = 0;
                  receive[0].Data[3] = 0;
                  receive[0].Data[4] = 0;
                  receive[0].Data[5] = 0;
                  receive[0].Data[6] = 0;
                  receive[0].Data[7] = 0;
                  receive1[0].Data[0] = 0;
                  receive1[0].Data[1] = 0;
                  receive1[0].Data[2] = 0;
                  receive1[0].Data[3] = 0;
                  receive1[0].Data[4] = 0;
                  receive1[0].Data[5] = 0;
                  receive1[0].Data[6] = 0;
                  receive1[0].Data[7] = 0;
                  
				  VCI_CAN_OBJ send[1];
				  send[0].ID = 0x201; //65
				  send[0].SendType = 0;
				  send[0].RemoteFlag = 0;
				  send[0].ExternFlag = 0;
				  send[0].DataLen = 8;
                  send[0].Data[0] = 0;
                  send[0].Data[1] = 0;
                  send[0].Data[2] = 0;
                  send[0].Data[3] = 0;
                  send[0].Data[4] = 0;
                  send[0].Data[5] = 0;
                  send[0].Data[6] = 0;
                  send[0].Data[7] = 0;

                  fb_encoder1 =0;
                  fb_encoder2 =0;
                  fb_rpm1 =0;
                  fb_rpm2 =0;
                  fb_volt2 =0;
                  fb_status2 =0;
                  fb_fault2 =0;
                  p_fb_fault_code2 =0;

                  can_lost = 0;

                  if(receive_frame == -1 || receive_frame1 == -1 )
                  {
                    RCLCPP_INFO(this->get_logger(), "CAN-CANALYST-LOST");
                    can_status.data=1;
                  }
                  else if(receive_frame == 0 || receive_frame1 == 0 )
                        {
                          RCLCPP_INFO(this->get_logger(), "CAN-CAN-CONECTION_LOST");
                          can_status.data=2;
                        }
              }
            }
            else 
            {
              can_lost = 0;
              can_status.data = 3;
            }
            //ROS_INFO("X_a[k]:[%x %d %d]",receive[0].ID,fb_encoder1,fb_encoder2);
          }
        }
  }
  void Receive_encoder(){

    
    
        // encoder.linear.x = (float)fb_encoder1;
        // encoder.linear.y = (float)fb_encoder2;

        encoder.angular.x = (float)fb_rpm1/10;
        encoder.angular.y = (float)fb_rpm2/10;

        status.linear.x = (float)fb_volt2;
        status.linear.y = (float)fb_status2;

        status.angular.x = (float)fb_fault_code1;
        status.angular.y = (float)fb_fault_code2;

        encoder_pub_->publish(encoder);
        status_pub_->publish(status);
        can_stat_pub_->publish(can_status);


	

}

  void Time_out(){
  auto current_init_time = rclcpp::Clock().now();
  double dt = (current_time - last_time).seconds();
    if(dt > 0.5)
    {
    	vel_linear_x = 0;
    	vel_angular_z = 0;
    }
}

  void  Initialize_Motor(){
          VCI_CAN_OBJ receive[1];

					VCI_CAN_OBJ CAN_TxMsg[1];
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          			CAN_TxMsg[0].Data[0] = 0x2F;
					CAN_TxMsg[0].Data[1] = 0x60;
					CAN_TxMsg[0].Data[2] = 0x60;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x03;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

					receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
           			 RCLCPP_INFO_STREAM(this->get_logger(), "loop1");
    				// ROS_INFO_STREAM(" loop1");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
					RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);


					CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          			CAN_TxMsg[0].Data[0] = 0x2B;
					CAN_TxMsg[0].Data[1] = 0x0F;
					CAN_TxMsg[0].Data[2] = 0x20;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x00;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

					receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
    				RCLCPP_INFO_STREAM(this->get_logger(), "loop1.5");
            // ROS_INFO_STREAM(" loop1.5");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
					RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);
    			



    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
         			CAN_TxMsg[0].Data[0] = 0x2B;
					CAN_TxMsg[0].Data[1] = 0x40;
					CAN_TxMsg[0].Data[2] = 0x60;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x00;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop2");
    				// ROS_INFO_STREAM(" loop2");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
					RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);
					
					
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x2B;
					CAN_TxMsg[0].Data[1] = 0x40;
					CAN_TxMsg[0].Data[2] = 0x60;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x06;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop3");
    				// ROS_INFO_STREAM(" loop3");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
    			RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);
    			
    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x2B;
					CAN_TxMsg[0].Data[1] = 0x40;
					CAN_TxMsg[0].Data[2] = 0x60;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x07;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop4");
    				// ROS_INFO_STREAM(" loop4");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
    			// ROS_INFO("X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);
    			RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);
    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x2B;
					CAN_TxMsg[0].Data[1] = 0x40;
					CAN_TxMsg[0].Data[2] = 0x60;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x0F;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop5");
    				// ROS_INFO_STREAM(" loop5");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
					// ROS_INFO("X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);
    			RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]);

					


  }

  void zltech_TPDO1()
{
	VCI_CAN_OBJ receive[1];

	VCI_CAN_OBJ CAN_TxMsg[1];
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x81; //80+id
					CAN_TxMsg[0].Data[5] = 0x01;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0xC0;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop6");
    				// ROS_INFO_STREAM(" loop6");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(), "X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			   

    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x1A;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x20;
					CAN_TxMsg[0].Data[5] = 0x01;
					CAN_TxMsg[0].Data[6] = 0x6C;
					CAN_TxMsg[0].Data[7] = 0x60;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop8");
    				// ROS_INFO_STREAM(" loop8");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x1A;
					CAN_TxMsg[0].Data[3] = 0x02;

					CAN_TxMsg[0].Data[4] = 0x20;
					CAN_TxMsg[0].Data[5] = 0x02;
					CAN_TxMsg[0].Data[6] = 0x6C;
					CAN_TxMsg[0].Data[7] = 0x60;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop9");
    				// ROS_INFO_STREAM(" loop9");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x2F;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x1A;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x02;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop10");
    				// ROS_INFO_STREAM(" loop10");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
           RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x2F;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x02;

					CAN_TxMsg[0].Data[4] = 0xFF;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop11");
    				// ROS_INFO_STREAM(" loop11");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x2B;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x03;

					CAN_TxMsg[0].Data[4] = 0x90;
					CAN_TxMsg[0].Data[5] = 0x01;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop12");
    				// ROS_INFO_STREAM(" loop12");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


					CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x81;//80+id
					CAN_TxMsg[0].Data[5] = 0x01;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x40;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop12");
    				// ROS_INFO_STREAM(" loop12");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);



}
  void zltech_TPDO2()
{
					VCI_CAN_OBJ receive[1];
					VCI_CAN_OBJ CAN_TxMsg[1];
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x81; //80+id
					CAN_TxMsg[0].Data[5] = 0x03;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0xC0;


    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop13");
    				// ROS_INFO_STREAM(" loop13");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    

    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x1A;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x10;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x35;
					CAN_TxMsg[0].Data[7] = 0x20;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop15");
    				// ROS_INFO_STREAM(" loop15");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x1A;
					CAN_TxMsg[0].Data[3] = 0x02;

					CAN_TxMsg[0].Data[4] = 0x10;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x40;
					CAN_TxMsg[0].Data[7] = 0x60;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop16");
    				// ROS_INFO_STREAM(" loop16");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x1A;
					CAN_TxMsg[0].Data[3] = 0x03;

					CAN_TxMsg[0].Data[4] = 0x20;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x3F;
					CAN_TxMsg[0].Data[7] = 0x60;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop17");
    				// ROS_INFO_STREAM(" loop17");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
			    CAN_TxMsg[0].ID = SDO_ID2;
			    CAN_TxMsg[0].SendType = 0;
			    CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x2F;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x1A;
					CAN_TxMsg[0].Data[3] = 0x00;

					CAN_TxMsg[0].Data[4] = 0x03;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop18");
    				// ROS_INFO_STREAM(" loop18");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


					
					CAN_TxMsg[0].ID = SDO_ID2;
					CAN_TxMsg[0].SendType = 0;
					CAN_TxMsg[0].RemoteFlag = 0;
					CAN_TxMsg[0].ExternFlag = 0;
					CAN_TxMsg[0].DataLen = 8;                           
          
          			CAN_TxMsg[0].Data[0] = 0x2F;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x02;

					CAN_TxMsg[0].Data[4] = 0xFF;
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            		RCLCPP_INFO_STREAM(this->get_logger(), "loop19");
    				// ROS_INFO_STREAM(" loop19");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
         			RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
					CAN_TxMsg[0].ID = SDO_ID2;
					CAN_TxMsg[0].SendType = 0;
					CAN_TxMsg[0].RemoteFlag = 0;
					CAN_TxMsg[0].ExternFlag = 0;
					CAN_TxMsg[0].DataLen = 8;                           
			
          			CAN_TxMsg[0].Data[0] = 0x2B;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x03;

					CAN_TxMsg[0].Data[4] = 0x90;
					CAN_TxMsg[0].Data[5] = 0x01;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop20");
    				// ROS_INFO_STREAM(" loop20");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


					CAN_TxMsg[0].ID = SDO_ID2;
					CAN_TxMsg[0].SendType = 0;
					CAN_TxMsg[0].RemoteFlag = 0;
					CAN_TxMsg[0].ExternFlag = 0;
					CAN_TxMsg[0].DataLen = 8;                           
			
          			CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x02;
					CAN_TxMsg[0].Data[2] = 0x18;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x81;
					CAN_TxMsg[0].Data[5] = 0x03;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x40;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop20");
    				// ROS_INFO_STREAM(" loop20");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);

    			
			    


}

  void zltech_RPDO1()
{				
				VCI_CAN_OBJ receive[1];
				VCI_CAN_OBJ CAN_TxMsg[1];
			  	CAN_TxMsg[0].ID = SDO_ID2;
			  	CAN_TxMsg[0].SendType = 0;
			  	CAN_TxMsg[0].RemoteFlag = 0;
			    CAN_TxMsg[0].ExternFlag = 0;
			    CAN_TxMsg[0].DataLen = 8;                           
          
          CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x14;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x01; //0+id
					CAN_TxMsg[0].Data[5] = 0x02;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x80;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            RCLCPP_INFO_STREAM(this->get_logger(), "loop22");
    				// ROS_INFO_STREAM(" loop22");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
					CAN_TxMsg[0].ID = SDO_ID2;
					CAN_TxMsg[0].SendType = 0;
					CAN_TxMsg[0].RemoteFlag = 0;
					CAN_TxMsg[0].ExternFlag = 0;
					CAN_TxMsg[0].DataLen = 8;                           
          
          			CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x16;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x20;
					CAN_TxMsg[0].Data[5] = 0x01;
					CAN_TxMsg[0].Data[6] = 0xFF;
					CAN_TxMsg[0].Data[7] = 0x60;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            		RCLCPP_INFO_STREAM(this->get_logger(), "loop23");
    				// ROS_INFO_STREAM(" loop23");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
         			 RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


					
					CAN_TxMsg[0].ID = SDO_ID2;
					CAN_TxMsg[0].SendType = 0;
					CAN_TxMsg[0].RemoteFlag = 0;
					CAN_TxMsg[0].ExternFlag = 0;
					CAN_TxMsg[0].DataLen = 8;                           
          
          			CAN_TxMsg[0].Data[0] = 0x23;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x16;
					CAN_TxMsg[0].Data[3] = 0x02;

					CAN_TxMsg[0].Data[4] = 0x20;
					CAN_TxMsg[0].Data[5] = 0x02;
					CAN_TxMsg[0].Data[6] = 0xFF;
					CAN_TxMsg[0].Data[7] = 0x60;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            		RCLCPP_INFO_STREAM(this->get_logger(), "loop24");
    				// ROS_INFO_STREAM(" loop24");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          			RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


    			
					CAN_TxMsg[0].ID = SDO_ID2;
					CAN_TxMsg[0].SendType = 0;
					CAN_TxMsg[0].RemoteFlag = 0;
					CAN_TxMsg[0].ExternFlag = 0;
					CAN_TxMsg[0].DataLen = 8;                           
			
          			CAN_TxMsg[0].Data[0] = 0x2F;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x16;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x02; //0+id
					CAN_TxMsg[0].Data[5] = 0x00;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            		RCLCPP_INFO_STREAM(this->get_logger(), "loop25");
    				// ROS_INFO_STREAM(" loop25");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          			RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);

					CAN_TxMsg[0].ID = SDO_ID2;
					CAN_TxMsg[0].SendType = 0;
					CAN_TxMsg[0].RemoteFlag = 0;
					CAN_TxMsg[0].ExternFlag = 0;
					CAN_TxMsg[0].DataLen = 8;                           
			
         			CAN_TxMsg[0].Data[0] = 0x22;
					CAN_TxMsg[0].Data[1] = 0x00;
					CAN_TxMsg[0].Data[2] = 0x14;
					CAN_TxMsg[0].Data[3] = 0x01;

					CAN_TxMsg[0].Data[4] = 0x01; //0+id
					CAN_TxMsg[0].Data[5] = 0x02;
					CAN_TxMsg[0].Data[6] = 0x00;
					CAN_TxMsg[0].Data[7] = 0x00;

    			receive_frame = 0;

    			send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    			while(receive_frame==0)
    			{
            		RCLCPP_INFO_STREAM(this->get_logger(), "loop25");
    				// ROS_INFO_STREAM(" loop25");
    				receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    			}
          			RCLCPP_INFO(this->get_logger(),"X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3],receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);
					// ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
					// 	,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);

}

  void Switch_NMT()
{
	VCI_CAN_OBJ receive[1];
	VCI_CAN_OBJ CAN_TxMsg[1];
		CAN_TxMsg[0].ID = 0x00;
		CAN_TxMsg[0].SendType = 0;
		CAN_TxMsg[0].RemoteFlag = 0;
		CAN_TxMsg[0].ExternFlag = 0;
		CAN_TxMsg[0].DataLen = 2;                           
          
    	CAN_TxMsg[0].Data[0] = 0x80;
		CAN_TxMsg[0].Data[1] = 0x01; //id
		

    receive_frame = 0;

    send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
    //while(receive_frame==0)
    //{
    //	ROS_INFO_STREAM(" loop26");
    //	receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
    //}
		//	ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
		//				,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);


		CAN_TxMsg[0].ID = 0x00;
		CAN_TxMsg[0].SendType = 0;
		CAN_TxMsg[0].RemoteFlag = 0;
		CAN_TxMsg[0].ExternFlag = 0;
		CAN_TxMsg[0].DataLen = 2;                           
          
    	CAN_TxMsg[0].Data[0] = 0x01;
		CAN_TxMsg[0].Data[1] = 0x01;  //id
		

    receive_frame = 0;

    send_frame = VCI_Transmit(VCI_USBCAN2, 0, 0, CAN_TxMsg, 1);

    			
 //   while(receive_frame==0)
 //   {
 //   	ROS_INFO_STREAM(" loop27");
 //   	receive_frame = VCI_Receive(VCI_USBCAN2, 0, 0, receive, 1, 5);
 //   }
//			ROS_INFO("X_a[k]:[%x %x %x %x %x %x %x %x ]",receive[0].Data[0],receive[0].Data[1],receive[0].Data[2],receive[0].Data[3]
//						,receive[0].Data[4],receive[0].Data[5],receive[0].Data[6],receive[0].Data[7]);

}
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg ) 
{
  
	vel_linear_x = msg->linear.x*100;
  	vel_angular_z = msg->angular.z*100;
  	last_time = current_time;

}
};



// void CanInterface::show(){
//   RCLCPP_INFO(this->get_logger(), "Hello");
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // CanInterface a;
  // a.CAN_init();
  rclcpp::spin(std::make_shared<CanInterface>());
  rclcpp::shutdown();
  return 0;
};


