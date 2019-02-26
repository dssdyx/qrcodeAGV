#include "../include/zbar_opencv_2/zbar_opencv_2.h"

namespace zbar_opencv_2 {
Velocitycal::Velocitycal(ros::NodeHandle nh_):
    nh_(nh_),Judge(false),depart(false),d(0),SPD(10),W(320),H(240){
    //ros::Publisher pub_velocity;
    //ros::Subscriber sub_QRpoint,sub_QRnext,sub_Fourpoint,sub_Judge,sub_depart;
    sub_QRpoint = nh_.subscribe("QRpoint",1,&Velocitycal::QRpoint_Callback,this);
    sub_Judge = nh_.subscribe("Judge",1,&Velocitycal::Judge_Callback,this);
    sub_QRnext = nh_.subscribe("QRnext",1,&Velocitycal::QRnext_Callback,this);
    sub_depart = nh_.subscribe("depart",1,&Velocitycal::depart_Callback,this);
    sub_Fourpoint = nh_.subscribe("Fourpoint",1,&Velocitycal::Fourpoint_Callback,this);
    sub_centralpoint = nh_.subscribe("Centralpoint",1,&Velocitycal::Centralpoint_Callback,this);
    pub_velocity = nh_.advertise<geometry_msgs::Pose2D>("velocity",1);
    temp.x = 0;
    temp.y = 0;
}
void Velocitycal::QRpoint_Callback(const geometry_msgs::PointConstPtr &Pt){
    QR_now.x=Pt->x;
    QR_now.y=Pt->y;
}
void Velocitycal::QRnext_Callback(const geometry_msgs::PointConstPtr &Pt){
    QR_next.x=Pt->x;
    QR_next.y=Pt->y;
    if(Judge&&depart){
        DIR = detectdir(QR_now,QR_next);
        if(DIR == stop){
            if(d<50){
                vector_Tonow.x = 0;
                vector_Tonow.y = 0;
                vector_Tonext.x = 0;
                vector_Tonext.y = 0;
            }
            else {
                vector_Tonext.x = 0;
                vector_Tonext.y = 0;
                vector_Tonow = Tocenter_vectorcal(centralpoint);
            }
        }
        else {
            vector_Tonow = Tonow_vectorcal(DIR,Fourpoint);
            vector_Tonext = Tonext_vectorcal(DIR,Fourpoint);
        }
        temp.x=vector_Tonext.x;
        temp.y=vector_Tonext.y;
        speed_Tonow = vectormul(vector_Tonow,k_d*d);
        speed_Tonext = vectormul(vector_Tonext,SPD);
        theta = vectorToangle(speed_Tonow,speed_Tonext);
        speed = vectorToscalar(speed_Tonow,speed_Tonext);
        vel.x = speed;
        vel.theta = theta;
        //cout<<int(theta*180/PI)<<endl;
        //cout<<d<<endl;
        pub_velocity.publish(vel);
    }
    else {
        if(depart){
            speed_Tonow = vectormul(vector_Tonow,0);
            speed_Tonext = vectormul(vector_Tonext,SPD);
            theta = vectorToangle(speed_Tonow,speed_Tonext);
            speed = vectorToscalar(speed_Tonow,speed_Tonext);
            vel.x = speed;
            vel.theta = theta;
            cout<<int(theta*180/PI)<<endl;
            pub_velocity.publish(vel);

        }
    }
}
geometry_msgs::Point Velocitycal::Tocenter_vectorcal(geometry_msgs::Point center){
    geometry_msgs::Point ox,v_,v;
    ox.x=W/2;
    ox.y=H/2;
    v_.x=center.x-ox.x;
    v_.y=center.y-ox.y;
    v.x=v_.x/sqrt(v_.x*v_.x+v_.y*v_.y);
    v.y=v_.y/sqrt(v_.x*v_.x+v_.y*v_.y);
    return v;
}
double Velocitycal::vectorToscalar(geometry_msgs::Point x, geometry_msgs::Point y){
    double speed_=sqrt((x.x+y.x)*(x.x+y.x)+(x.y+y.y)*(x.y+y.y));
    return speed_;
}
double Velocitycal::anglecal(geometry_msgs::Point a){
    double angle_ = -atan2(a.y,a.x)+atan2(-1,0);
    if(angle_ < 0) angle_+=2*PI;
    if(angle_ > 2*PI) angle_-=2*PI;
    return angle_;
}
double Velocitycal::vectorToangle(geometry_msgs::Point x, geometry_msgs::Point y){
    geometry_msgs::Point sum;
    sum.x=x.x+y.x;
    sum.y=x.y+y.y;
    double theta_ = anglecal(sum);
    return theta_;
}
geometry_msgs::Point Velocitycal::vectormul(geometry_msgs::Point vector_, double scalar){
    geometry_msgs::Point v_;
    v_.x = vector_.x*scalar;
    v_.y = vector_.y*scalar;
    return  v_;
}
geometry_msgs::Point Velocitycal::Tonow_vectorcal(dir DIR, geometry_msgs::Polygon Fourpoint){
    geometry_msgs::Point32 p0,p1,p2,p3,center;
    Fourpoint.points[0].x = p0.x;
    Fourpoint.points[0].y = p0.y;
    Fourpoint.points[1].x = p1.x;
    Fourpoint.points[1].y = p1.y;
    Fourpoint.points[2].x = p2.x;
    Fourpoint.points[2].y = p2.y;
    Fourpoint.points[3].x = p3.x;
    Fourpoint.points[3].y = p3.y;
    center.x = int((p0.x+p1.x+p2.x+p3.x)/4);
    center.y = int((p0.y+p1.y+p2.y+p3.y)/4);
    double k_tb,k_lr;
    k_tb=(p1.y-p0.y+p2.y-p3.y)/(p1.x-p0.x+p2.x-p3.x);
    k_lr=(p0.y-p3.y+p1.y-p2.y)/(p0.x-p3.x+p1.x-p2.x);
    geometry_msgs::Point P_vd,vector_;
    double a,b,c,m,n;
    double x,y;
    m=W/2;n=H/2;
    switch(DIR)
    {
      case top:
      case bottom:
        a=k_tb;
        b=-1;
        c=-k_tb*center.x+center.y;
        break;
      case Left:
      case Right:
        a=k_lr;
        b=-1;
        c=-k_lr*center.x+center.y;
        break;
    }
    P_vd.x=int((b*b*m-a*b*n-a*c)/(a*a+b*b));
    P_vd.y=int((a*a*n-a*b*m-b*c)/(a*a+b*b));
    x=P_vd.x-m;
    y=P_vd.y-n;
    vector_.x=x/sqrt(x*x+y*y);
    vector_.y=y/sqrt(x*x+y*y);
    return vector_;

}
geometry_msgs::Point Velocitycal::Tonext_vectorcal(dir DIR, geometry_msgs::Polygon Fourpoint){
    double x,y;
    geometry_msgs::Point32 p0,p1,p2,p3;
    Fourpoint.points[0].x = p0.x;
    Fourpoint.points[0].y = p0.y;
    Fourpoint.points[1].x = p1.x;
    Fourpoint.points[1].y = p1.y;
    Fourpoint.points[2].x = p2.x;
    Fourpoint.points[2].y = p2.y;
    Fourpoint.points[3].x = p3.x;
    Fourpoint.points[3].y = p3.y;
    switch(DIR)
  {
      case top:
          x=-(p1.x-p0.x+p2.x-p3.x)/4;
          y=-(p1.y-p0.y+p2.y-p3.y)/4;
          break;
      case bottom:
          x=(p1.x-p0.x+p2.x-p3.x)/4;
          y=(p1.y-p0.y+p2.y-p3.y)/4;
          break;
      case Left:
          x=(p1.x+p0.x-p2.x-p3.x)/4;
          y=(p1.y+p0.y-p2.y-p3.y)/4;
          break;
      case Right:
          x=-(p1.x+p0.x-p2.x-p3.x)/4;
          y=-(p1.y+p0.y-p2.y-p3.y)/4;
          break;
          }
    geometry_msgs::Point vector_;
    vector_.x=x/sqrt(x*x+y*y);
    vector_.y=y/sqrt(x*x+y*y);
    return vector_;
}
Velocitycal::dir Velocitycal::detectdir(geometry_msgs::Point now, geometry_msgs::Point next){
    int x0,y0;
    x0=next.x-now.x;
    y0=next.y-now.y;
    dir d;
    if(x0==0)
    {
            if(y0>0) d=top;//bottom
            else if(y0<0) d= bottom;///top
            else d=stop;///stop
    }
    else
    {
            if(x0>0) d=Right; //rigth
            else d=Left;//left
    }
    return d;
}
void Velocitycal::Judge_Callback(const std_msgs::BoolConstPtr &SIGN){
    Judge = SIGN->data;
}
void Velocitycal::depart_Callback(const std_msgs::BoolConstPtr &Depart){
    depart = Depart->data;
}
void Velocitycal::Fourpoint_Callback(const geometry_msgs::PolygonConstPtr &Fpt){
    Fourpoint.points = Fpt->points;
}
void Velocitycal::Centralpoint_Callback(const geometry_msgs::PointConstPtr &Pt){
    centralpoint.x = Pt->x;
    centralpoint.y = Pt->y;
    d=int(sqrt((W/2-centralpoint.x)*(W/2-centralpoint.x)+(H/2-centralpoint.y)*(H/2-centralpoint.y)));
}
}
