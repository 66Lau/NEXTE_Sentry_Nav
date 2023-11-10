#include "ros/ros.h"  

#include <std_msgs/String.h>  
#include <sstream>  
#include <string>
#include <iostream>

int main (int argc, char** argv){

    ros::init(argc, argv, "random_deci");

    ros::NodeHandle n;
    ros::Rate r(100);
    

    // 创建一个Subscriber，订阅名为data_chatter的topic，注册回调函数chatterCallback 
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, callback); 
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_2d",50);
    while(ros::ok()){
        ros::spinOnce();
        odom_pub.publish(odom_2d);
        r.sleep();}
        
}

