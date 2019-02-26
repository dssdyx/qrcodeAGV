#include <serial/serial_nodelet.h>

namespace serial {
SerialNodelet::SerialNodelet():
    MODE(false),Speed(0),Theta(0),TR(0),dur(100),temp(false){
}
void SerialNodelet::onInit(){

    try {
        ser.setPort("/dev/ttyUSB0");///Raspberry to 2560
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
    }
   if(ser.isOpen()){
          ROS_INFO_STREAM("Serial Port initialized");
    }
   sub_mode = nh_.subscribe("mode",1,&SerialNodelet::mode_Callback,this);
   sub_velocity = nh_.subscribe("velocity",1,&SerialNodelet::velocity_Callback,this);
   time_=nh_.createTimer(ros::Duration(0.02),&SerialNodelet::time_callback,this);
   SetInit();
}
void SerialNodelet::sendvel(int SPEED, int THETA,int TR, int Duration){
    uint8_t datapack[14];
    uint8_t sum=0;
    datapack[0] = 0xF5;
    datapack[1] = 0x5F;
    datapack[2] = 0x08;
    datapack[3] = 0x02;
    datapack[4] = 0x08;
    datapack[5] = THETA&0xFF;
    datapack[6] = (THETA >> 8)&0xFF;
    datapack[7] = SPEED&0xFF;
    datapack[8] = (SPEED >> 8)&0xFF;
    datapack[9] = TR&0xFF;
    datapack[10] = (TR>>8)&0xFF;
    datapack[11] = Duration&0xFF;
    datapack[12] = (Duration>>8)&0xFF;
    for (int i=0;i<11;i++) {
        sum+=datapack[i+2];
    }
    datapack[13] = ~sum;
    ser.write(datapack,14);
}
void SerialNodelet::SetInit(){
    uint8_t datapack[7];
    uint8_t sum=0;
    datapack[0] = 0xF5;
    datapack[1] = 0x5F;
    datapack[2] = 0x08;
    datapack[3] = 0x01;
    datapack[4] = 0x01;
    datapack[5] = 0x02;
    for (int i=0;i<4;i++) {
        sum+=datapack[i+2];
    }
    datapack[6] = ~sum;
    ser.write(datapack,7);
}
void SerialNodelet::Grip(){  //0
    uint8_t datapack[7];
    uint8_t sum=0;
    datapack[0] = 0xF5;
    datapack[1] = 0x5F;
    datapack[2] = 0x07;
    datapack[3] = 0x55;
    datapack[4] = 0x01;
    datapack[5] = 0x00;
    for (int i=0;i<4;i++) {
        sum+=datapack[i+2];
    }
    datapack[6] = ~sum;
    ser.write(datapack,7);
}
void SerialNodelet::Release(){ //1
    uint8_t datapack[7];
    uint8_t sum=0;
    datapack[0] = 0xF5;
    datapack[1] = 0x5F;
    datapack[2] = 0x07;
    datapack[3] = 0x55;
    datapack[4] = 0x01;
    datapack[5] = 0x01;
    for (int i=0;i<4;i++) {
        sum+=datapack[i+2];
    }
    datapack[6] = ~sum;
    ser.write(datapack,7);
}
void SerialNodelet::time_callback(const ros::TimerEvent &){
    sendvel(Speed,Theta,TR,dur);
    //cout<<Speed<<endl;
}
void SerialNodelet::mode_Callback(const std_msgs::BoolConstPtr &m_){
    MODE = m_->data;
    switch (MODE) {
        case true:
        if(!temp){
            Grip();
            temp=true;
        }
        break;
        case false:
        Release();
        temp=false;
        break;
    }
}
void SerialNodelet::velocity_Callback(const geometry_msgs::Pose2DConstPtr &v_){
    Speed=int(v_->x);
    Theta=int(v_->theta);
}
SerialNodelet::~SerialNodelet(){

}
}
PLUGINLIB_EXPORT_CLASS(serial::SerialNodelet, nodelet::Nodelet)
