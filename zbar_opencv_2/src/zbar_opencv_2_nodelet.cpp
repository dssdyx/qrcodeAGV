#include "../include/zbar_opencv_2/zbar_opencv_2_nodelet.h"

namespace zbar_opencv_2 {
        VelocitycalNodelet::VelocitycalNodelet(){}
        void VelocitycalNodelet::onInit(){
            ros::NodeHandle nh = getNodeHandle();
            velo = new Velocitycal(nh);
        }
        VelocitycalNodelet::~VelocitycalNodelet(){
            if(velo) delete velo;
        }
}

PLUGINLIB_EXPORT_CLASS(zbar_opencv_2::VelocitycalNodelet, nodelet::Nodelet)
