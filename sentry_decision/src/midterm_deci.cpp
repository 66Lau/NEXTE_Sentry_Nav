#include "ros/ros.h"  
#include "midterm_deci.h"

#include <std_msgs/String.h>  
#include <sstream>  
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <random>

/*
4个模式
1. 速推前哨战模式-导航去outpost
2. 两点巡逻模式
3. 探索索敌模式
4. 原地旋转防守模式-电控遥控器切换

*/

geometry_msgs::PoseStamped Goal;
// geometry_msgs::PoseStamped Goal_outpost;
// geometry_msgs::PoseStamped Goal_double1;
// geometry_msgs::PoseStamped Goal_double2;
// geometry_msgs::PoseStamped Goal;

void getStartGoal();
void getOutpostGoal();
void getDoublePoints1Goal();
void getDoublePoints2Goal();
void getRandomGoal();


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

void callback(const nav_msgs::Odometry& odom)
{

    distance = abs(odom.pose.pose.position.x-Goal.pose.position.x)+abs(odom.pose.pose.position.y+Goal.pose.position.y);
    ROS_INFO("distance = %f\n",distance);


    
}

int main (int argc, char** argv){

    ros::init(argc, argv, "midterm_deci");

    ros::NodeHandle n;
    ros::Rate r(0.5);//一秒执行一次

    getStartGoal();
    
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);
    goal_pub.publish(Goal);

    ros::Subscriber sub = n.subscribe("/Odometry", 1000, callback);  

    while(ros::ok()){
        ROS_INFO("run midterm_decis\n");
        ros::spinOnce();

        publish_flag = (distance < trigger_distance);
        if(publish_flag)
        {
            switch(switch_num)
            {
                case 1:
                    getOutpostGoal();
                    break;
                case 2:
                    getOutpostGoal();
                    break;
                case 3:
                    getOutpostGoal();
                    break;
                case 4:
                    getOutpostGoal();
                    break;
                
                default:
                    if(switch_num >= 5 && switch_num <= 12 && switch_num % 2 != 0)
                        {getDoublePoints1Goal();}
                    else if(switch_num >= 5 && switch_num <= 12 && switch_num % 2 == 0)
                        {getDoublePoints2Goal();}
                    else if(switch_num >= 13)
                        {getRandomGoal();}
                    break;
            }
            switch_num++;
        }

        goal_pub.publish(Goal);
        publish_flag = false;

        r.sleep();}
        
}

void getStartGoal()
{
    Goal.header.frame_id = "robot_foot_init";
    Goal.pose.position.x = init_goal_x;
    Goal.pose.position.y = init_goal_y;
    
    Goal.pose.position.z = 0;
    Goal.pose.orientation.x = 0;
    Goal.pose.orientation.y = 0;
    Goal.pose.orientation.z = 0.1;
    Goal.pose.orientation.w = 0.95;
}


void getOutpostGoal()
{
    Goal.header.frame_id = "robot_foot_init";
    Goal.pose.position.x = 3.0;
    Goal.pose.position.y = 0.8;
    Goal.pose.position.z = 0;
    Goal.pose.orientation.x = 0;
    Goal.pose.orientation.y = 0;
    Goal.pose.orientation.z = 0.1;
    Goal.pose.orientation.w = 1;
    ROS_INFO("GET NEW GOAL!! : x=%f, y=%f\n",Goal.pose.position.x,Goal.pose.position.y);
}

void getDoublePoints1Goal()
{
    Goal.header.frame_id = "robot_foot_init";
    Goal.pose.position.x = 1.0;
    Goal.pose.position.y = 1.0;
    Goal.pose.position.z = 0;
    Goal.pose.orientation.x = 0;
    Goal.pose.orientation.y = 0;
    Goal.pose.orientation.z = 0.1;
    Goal.pose.orientation.w = 1;
    ROS_INFO("GET NEW GOAL!! : x=%f, y=%f\n",Goal.pose.position.x,Goal.pose.position.y);
}

void getDoublePoints2Goal()
{
    Goal.header.frame_id = "robot_foot_init";
    Goal.pose.position.x = 1.0;
    Goal.pose.position.y = -0.4;
    Goal.pose.position.z = 0;
    Goal.pose.orientation.x = 0;
    Goal.pose.orientation.y = 0;
    Goal.pose.orientation.z = 0.1;
    Goal.pose.orientation.w = 1;
    ROS_INFO("GET NEW GOAL!! : x=%f, y=%f\n",Goal.pose.position.x,Goal.pose.position.y);
}

void getRandomGoal()
{
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


