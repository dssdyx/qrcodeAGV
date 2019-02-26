#ifndef _ZBAR_OPENCV_2_H
#define _ZBAR_OPENCV_2_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#define PI 3.1415926
#define k_d 0.1

using namespace std;

namespace zbar_opencv_2 {
    class Velocitycal {
    public:
        Velocitycal(ros::NodeHandle nh_);
        void QRpoint_Callback(const geometry_msgs::PointConstPtr &Pt);
        void QRnext_Callback(const geometry_msgs::PointConstPtr &Pt);
        void Judge_Callback(const std_msgs::BoolConstPtr &SIGN);
        void depart_Callback(const std_msgs::BoolConstPtr &Depart);
        void Fourpoint_Callback(const geometry_msgs::PolygonConstPtr &Fpt);
        void Centralpoint_Callback(const geometry_msgs::PointConstPtr &Pt);
        enum dir {top,bottom,Left,Right,stop};
        dir detectdir(geometry_msgs::Point now, geometry_msgs::Point next);
        geometry_msgs::Point Tonow_vectorcal(dir DIR,geometry_msgs::Polygon Fourpoint);
        geometry_msgs::Point Tonext_vectorcal(dir DIR,geometry_msgs::Polygon Fourpoint);
        geometry_msgs::Point vectormul(geometry_msgs::Point vector_,double scalar);
        geometry_msgs::Point Tocenter_vectorcal(geometry_msgs::Point center);
        double vectorToangle(geometry_msgs::Point x, geometry_msgs::Point y);
        double vectorToscalar(geometry_msgs::Point x, geometry_msgs::Point y);
        double anglecal(geometry_msgs::Point a);

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_velocity;
        ros::Subscriber sub_QRpoint,sub_QRnext,sub_Fourpoint,sub_Judge,sub_depart,sub_centralpoint;
        geometry_msgs::Polygon Fourpoint;
        geometry_msgs::Point QR_now,QR_next,P0,P1,P2,P3,centralpoint,
        vector_Tonow,vector_Tonext,speed_Tonow,speed_Tonext,temp;
        bool Judge,depart;
        geometry_msgs::Pose2D vel;
        dir DIR;
        int d,SPD;
        int H,W;
        double theta,speed;

    };

}

#endif //_ZBAR_OPENCV_2_H
