#ifndef _ZBAR_OPENCV_1_NODELET_H
#define _ZBAR_OPENCV_1_NODELET_H
#include "zbar_opencv_1.hpp"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace zbar_opencv_1 {
class TrackcalNodelet:public nodelet::Nodelet{
public:
    TrackcalNodelet();
    virtual void onInit();
    ~TrackcalNodelet();
private:
    Trackcal *track;
};

}
#endif  //_ZBAR_OPENCV_NODELET_1_H
