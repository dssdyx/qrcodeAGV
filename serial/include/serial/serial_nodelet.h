#ifndef _SERIAL_NODELET_H
#define _SERIAL_NODELET_H
#include <serial/serial.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
using namespace std;

namespace serial {
class SerialNodelet:public nodelet::Nodelet{
public:
    SerialNodelet();
    ~SerialNodelet();
    virtual void onInit();
    void mode_Callback(const std_msgs::BoolConstPtr& m_);
    void velocity_Callback(const geometry_msgs::Pose2DConstPtr& v_);
    void Grip();
    void Release();
    void time_callback(const ros::TimerEvent&);
    void sendvel(int SPEED,int THETA,int TR, int Duration);
    void SetInit();
private:
    ros::NodeHandle nh_;
    serial::Serial ser;
    ros::Subscriber sub_mode,sub_velocity;
    bool MODE;
    int Speed,Theta,TR,dur;
    bool temp;
    ros::Timer time_;
};

}

#endif
