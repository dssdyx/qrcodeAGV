#ifndef _ZBAR_OPENCV_1_H
#define _ZBAR_OPENCV_1_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

namespace zbar_opencv_1 {
    class Trackcal {
    public:
        Trackcal(ros::NodeHandle nh_);
        void QRdata_Callback(const geometry_msgs::PointConstPtr &Pt);
        void QRend_Callback(const geometry_msgs::PointConstPtr &Pt);
        void end_sign_Callback(const std_msgs::Bool &SIGN);
        void Compare(geometry_msgs::Point P_END,geometry_msgs::Point P_QR);


    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_QRnext;
        ros::Subscriber sub_QRdata,sub_QRend,sub_end_sign;
        geometry_msgs::Point QR_now,QR_end,QR_next;
        bool END_SIGN;

    };

}

#endif //_ZBAR_OPENCV_1_H
