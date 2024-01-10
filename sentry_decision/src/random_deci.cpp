#include "ros/ros.h"  
#include "random_deci.h"

#include <std_msgs/String.h>  
#include <sstream>  
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <random>


geometry_msgs::PoseStamped Goal;


double generateRandomNumber(double min, double max){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(min,max);
    return distribution(gen);
}


//:TODO
//0.publish the goal point "geometry_msgs/GoalStatusArray"
//1.suscribe the topic /localization "nav_msgs/Odometry"
//2.If the Vx+Vy < min_velocity?
    //if No: back to step 1
//3.if Yes:get a random new goal and back to step 0

// 接收到订阅的消息后，会进入消息回调函数
void callback(const nav_msgs::Odometry& odom)
{
    // receive the msg from cmd_vel
    ROS_INFO("Receive a imu msg\n");
    std::cout<< "Receive a imu msg\n";
    // ROS_INFO("The linear  velocity: x=%f, y=%f, z=%f\n",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
    // ROS_INFO("The augular velocity: roll=%f, pitch=%f, yaw=%f\n",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
    // put the data in union
    // std::cout << imu.orientation.x << std::endl;
    // std::cout << imu.orientation.y << std::endl;

    distance = abs(odom.pose.pose.position.x-Goal.pose.position.x)+abs(odom.pose.pose.position.y-Goal.pose.position.y);
    publish_flag = (distance < trigger_distance);

    if (publish_flag ){
        Goal.header.frame_id = "robot_foot_init";
        Goal.pose.position.x = generateRandomNumber(min_x_range, max_x_range);
        Goal.pose.position.y = generateRandomNumber(min_y_range, max_y_range);
        Goal.pose.position.z = 0;
        Goal.pose.orientation.x = 0;
        Goal.pose.orientation.y = 0;
        Goal.pose.orientation.z = 0.1;
        Goal.pose.orientation.w = 1;
        ROS_INFO("GET NEW GOAL!! : x=%f, y=%f\n",Goal.pose.position.x,Goal.pose.position.y);


    }
}

int main (int argc, char** argv){

    ros::init(argc, argv, "random_deci");

    ros::NodeHandle n;
    ros::Rate r(1);
    Goal.header.frame_id = "robot_foot_init";
    Goal.pose.position.x = init_goal_x;
    Goal.pose.position.y = init_goal_y;
    
    Goal.pose.position.z = 0;
    Goal.pose.orientation.x = 0;
    Goal.pose.orientation.y = 0;
    Goal.pose.orientation.z = 0.1;
    Goal.pose.orientation.w = 0.95;

    // 创建一个Subscriber，订阅名为data_chatter的topic，注册回调函数chatterCallback 
    
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);
    goal_pub.publish(Goal);
    ros::Subscriber sub = n.subscribe("/Odometry", 1000, callback);  

    while(ros::ok()){
        ROS_INFO("run decis\n");
        ros::spinOnce();
        // if (publish_flag ){
            goal_pub.publish(Goal);
            // }
        r.sleep();}
        
}

