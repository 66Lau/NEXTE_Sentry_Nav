#include "ros/ros.h"  
#include "serial_send.h"


#include <sstream>  
#include <geometry_msgs/Twist.h>
#include <string>
#include <iostream>

// 接收到订阅的消息后，会进入消息回调函数
void callback(const geometry_msgs::Twist& cmd_vel)
{
    // receive the msg from cmd_vel
    ROS_INFO("Receive a /cmd_vel msg\n");
    ROS_INFO("The linear  velocity: x=%f, y=%f, z=%f\n",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
    ROS_INFO("The augular velocity: roll=%f, pitch=%f, yaw=%f\n",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
    // put the data in union
    Serial_Package serial_package;
    serial_package.header = 0xA5;
    serial_package.linear_x = cmd_vel.linear.x;
    serial_package.linear_y = cmd_vel.linear.y;
    serial_package.angular_z = cmd_vel.angular.z;

    sentry_ser.flush ();
    sentry_ser.write(serial_package.Send_Buffer,data_len);
    ROS_INFO("\nSend date finished!\n");
}


int main (int argc, char** argv){

    ros::init(argc, argv, "sentry_send");

    ros::NodeHandle n;
    n.getParam("cmd_vel_topic", cmd_vel_topic);
    std::cout<<argv[1]<<std::endl;
    std::cout<<argc<<std::endl;
    try
    {
        std::string serial_port;
        if(argc == 2){
            serial_port = argv[1];
        }
        else{
            serial_port = "/dev/ttyUSB0";
        }
        std::cout<<serial_port<<std::endl;
        sentry_ser.setPort(serial_port);
        sentry_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sentry_ser.setTimeout(to);
        sentry_ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(sentry_ser.isOpen()){
        ROS_INFO_STREAM("Serial Port opened");
    }else{
        return -1;
    }
    ROS_INFO_STREAM("Init Finished!");

    // 创建一个Subscriber，订阅名为data_chatter的topic，注册回调函数chatterCallback 
    ros::Subscriber sub = n.subscribe(cmd_vel_topic, 1000, callback); 

    ros::spin();
}
