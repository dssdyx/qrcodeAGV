#include "../include/zbar_opencv_1/zbar_opencv_1_nodelet.hpp"

namespace zbar_opencv_1 {
        TrackcalNodelet::TrackcalNodelet(){}
        void TrackcalNodelet::onInit(){
            ros::NodeHandle nh = getNodeHandle();
            track = new Trackcal(nh);
        }
        TrackcalNodelet::~TrackcalNodelet(){
            if(track) delete track;
        }
}

PLUGINLIB_EXPORT_CLASS(zbar_opencv_1::TrackcalNodelet, nodelet::Nodelet)
