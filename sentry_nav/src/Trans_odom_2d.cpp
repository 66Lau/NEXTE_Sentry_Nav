#include "ros/ros.h"  


#include <serial/serial.h>  
#include <std_msgs/String.h>  
#include <sstream>  
#include <nav_msgs/Odometry.h>
#include <string>
#include <iostream>


nav_msgs::Odometry odom_2d;

// 接收到订阅的消息后，会进入消息回调函数
void callback(const nav_msgs::Odometry& odom)
{
    // receive the msg from cmd_vel
    ROS_INFO("Receive a odom msg\n");
    // ROS_INFO("The linear  velocity: x=%f, y=%f, z=%f\n",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
    // ROS_INFO("The augular velocity: roll=%f, pitch=%f, yaw=%f\n",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
    // put the data in union
    odom_2d.header.stamp = ros::Time::now();
    odom_2d.header.frame_id = "camera_init_2d";
    odom_2d.child_frame_id = "map";

    odom_2d.pose.pose.position.x = odom.pose.pose.position.x;
    odom_2d.pose.pose.position.y = odom.pose.pose.position.y;
    odom_2d.pose.pose.position.z = 0.00;
    odom_2d.pose.pose.orientation.x = 0.00;
    odom_2d.pose.pose.orientation.y = 0.00;
    odom_2d.pose.pose.orientation.z = odom.pose.pose.orientation.z;
    odom_2d.pose.pose.orientation.w = odom.pose.pose.orientation.w;

    odom_2d.twist.twist.linear.x = odom.twist.twist.linear.x;
    odom_2d.twist.twist.linear.y = odom.twist.twist.linear.y;
    odom_2d.twist.twist.linear.z = 0;
    odom_2d.twist.twist.angular.x = 0;
    odom_2d.twist.twist.angular.y = 0;
    odom_2d.twist.twist.angular.z = odom.twist.twist.angular.z;
}


int main (int argc, char** argv){

    ros::init(argc, argv, "Trans_odom_2d");

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
