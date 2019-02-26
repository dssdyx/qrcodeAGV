#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <cmath>
using namespace std;
//serial::Serial ser; //声明串口对象

///////////////////////////////

#define DATA_LENGTH   5
#define IMPACT 5
#define DATA_LENGTH_1 2
#define PI 3.1415926


class serialsend
{
	public:
		serialsend()
		{
		  write_sub = nh.subscribe("velocity", 1, &serialsend::write_callback,this);
		  read_pub = nh.advertise<std_msgs::String>("read", 1000);
		  SPEED=0;
		  ANGLE=0;
		  try
   		 {
                        ser.setPort("/dev/ttyUSB0");///in Raspberry
      			ser.setBaudrate(500000);
        		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        		ser.setTimeout(to);
        		ser.open();
   		 }
   		 catch (serial::IOException& e)
   		 {
       			ROS_ERROR_STREAM("Unable to open port ");
        	 }
 
		 if(ser.isOpen())
    		{
        		ROS_INFO_STREAM("Serial Port initialized");
    		}
    		
		}
		serial::Serial ser;
		int SPEED,ANGLE;
		void write_callback(const geometry_msgs::Point::ConstPtr& cmd_vel)
		{
		  float speed,angle,vx,vy;
		  vx=cmd_vel->x;
		  vy=cmd_vel->y;
   		  speed=sqrt(vx*vx+vy*vy);
		  angle = atan2(vy,vx)-atan2(-1,0);
		  if(angle < 0) angle+=2*PI;
		  if(angle > 2*PI) angle-=2*PI;
		  SPEED=(int)speed;
		  ANGLE=(int)angle;
		  ROS_INFO_STREAM("Writing to serial port : speed"<<SPEED);
		  ROS_INFO_STREAM("Writing to serial port : angle"<<ANGLE);
	  	}
		unsigned char datapack_1[DATA_LENGTH_1+6];
                void Base_Move(int Angle, int Speed) //STM32
		{
			unsigned char datapack[DATA_LENGTH+6];
			unsigned char sum=0,sum_1=0;
			datapack[0] = 0xFF;
			datapack[1] = 0xFF;
			datapack[2] = 0xe5;
			datapack[3] = DATA_LENGTH+2;
			datapack[4] = 0x03;
			datapack[5] = 0x06;
                        datapack[6] = Angle&0xFF;
                        datapack[7] = (Angle >> 8)&0xFF;
                        datapack[8] = Speed&0xFF;
                        datapack[9] = (Speed >> 8)&0xFF;
			for(int i=0;i<DATA_LENGTH+3;i++)
			{
				sum+=datapack[i+2];
			}
			datapack[DATA_LENGTH+5] = ~sum;
			ser.write(datapack,DATA_LENGTH+6);
		}
void Mode_Change(uint8_t MODE){
    unsigned char datapack[7];
    uint8_t sum=0;
    datapack[0] = 0xF5;
    datapack[1] = 0x5F;
    datapack[2] = 0x07;
    datapack[3] = 0x55;
    datapack[4] = 0x01;
    datapack[5] = MODE;
    for(int i=0;i<4;i++){
        sum+=datapack[i+2];
    }
    datapack[6]=~sum;
    ser.write(datapack,7);
}
void Base_Move_2560(int Angle,int Speed, int TurnRate, int Time){
    uint8_t datapack[14];
    uint8_t sum=0;
    datapack[0] = 0xF5;
    datapack[1] = 0x5F;
    datapack[2] = 0x08;
    datapack[3] = 0x02;
    datapack[4] = 0x08;
    datapack[5] = Angle&0xFF;
    datapack[6] = (Angle >> 8)&0xFF;
    datapack[7] = Speed&0xFF;
    datapack[8] = (Speed >> 8)&0xFF;
    datapack[9] = TurnRate&0xFF;
    datapack[10] = (TurnRate>>8)&0xFF;
    datapack[11] = Time&0xFF;
    datapack[12] = (Time>>8)&0xFF;
    for (int i=0;i<11;i++) {
        sum+=datapack[i+2];
    }
    datapack[13] = ~sum;
    ser.write(datapack,14);

}

        ros::Publisher read_pub;
	private:
	ros::NodeHandle nh;
	ros::Subscriber write_sub;

	
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "serial_node");
    serialsend ib;
    ros::Rate loop_rate(100);
    while(ros::ok())
    {

        ros::spinOnce();
	ib.Base_Move(ib.ANGLE,ib.SPEED);
        if(ib.ser.available()){
           ROS_INFO_STREAM("Reading from serial port");
           std_msgs::String result;
           result.data = ib.ser.read(ib.ser.available());
           ROS_INFO_STREAM("Read: " << result.data);
           ib.read_pub.publish(result);
        }
        loop_rate.sleep();
 
    }
}

