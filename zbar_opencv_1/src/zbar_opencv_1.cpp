#include "../include/zbar_opencv_1/zbar_opencv_1.hpp"

namespace zbar_opencv_1 {
Trackcal::Trackcal(ros::NodeHandle nh_):
    nh_(nh_),END_SIGN(false){
    sub_QRend = nh_.subscribe("end_info",1,&Trackcal::QRend_Callback,this);
    sub_QRdata = nh_.subscribe("QRpoint",1,&Trackcal::QRdata_Callback,this);
    sub_end_sign = nh_.subscribe("end_sign",1,&Trackcal::end_sign_Callback,this);
    pub_QRnext = nh_.advertise<geometry_msgs::Point>("QRnext",1);
}
void Trackcal::QRend_Callback(const geometry_msgs::PointConstPtr &Pt){
    QR_end.x=Pt->x;
    QR_end.y=Pt->y;

}
void Trackcal::QRdata_Callback(const geometry_msgs::PointConstPtr &Pt){
    QR_now.x=Pt->x;
    QR_now.y=Pt->y;
    if(END_SIGN){
        Compare(QR_end,QR_now);
        pub_QRnext.publish(QR_next);
    }

}
void Trackcal::end_sign_Callback(const std_msgs::Bool &SIGN){
    END_SIGN=SIGN.data;
}
void Trackcal::Compare(geometry_msgs::Point P_END, geometry_msgs::Point P_QR){
    if(!(P_END.x==P_QR.x&&P_END.y==P_QR.y))
  {

     if(P_QR.x!=P_END.x)
     {
          if(P_QR.x-P_END.x>0) {
              QR_next.x=P_QR.x-10;
              QR_next.y=P_QR.y;
          }
          else {
              QR_next.x=P_QR.x+10;
              QR_next.y=P_QR.y;
          }
     }
     else
     {
          if(P_QR.y-P_END.y>0) {
              QR_next.x=P_QR.x;
              QR_next.y=P_QR.y-10;
          }
          else {
              QR_next.x=P_QR.x;
              QR_next.y=P_QR.y+10;
          }
     }
  }
  else
  {
     QR_next.x=P_QR.x;//if at the beginning??
     QR_next.y=P_QR.y;
  }

}

}

