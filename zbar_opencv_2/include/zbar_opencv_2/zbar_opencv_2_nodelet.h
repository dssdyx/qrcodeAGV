#ifndef _ZBAR_OPENCV_2_NODELET_H
#define _ZBAR_OPENCV_2_NODELET_H
#include "zbar_opencv_2.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace zbar_opencv_2 {
class VelocitycalNodelet:public nodelet::Nodelet{
public:
    VelocitycalNodelet();
    virtual void onInit();
    ~VelocitycalNodelet();
private:
    Velocitycal *velo;
};

}
#endif  //_ZBAR_OPENCV_NODELET_2_H
