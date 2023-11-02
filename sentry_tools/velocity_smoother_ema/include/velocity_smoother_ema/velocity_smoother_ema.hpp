#ifndef VELOCITY_SMOOTHER_EMA_H
#define VELOCITY_SMOOTHER_EMA_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class VelocitySmootherEma
{
    public:
        VelocitySmootherEma(ros::NodeHandle* nh);
        ~VelocitySmootherEma();
        void twist_callback(const geometry_msgs::Twist::ConstPtr msg);
        void update(const ros::TimerEvent&);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber velocity_sub_;
        ros::Publisher velocity_pub_;
        ros::Timer timer;
        double alpha_v, alpha_w;
        double cmd_rate;
        std::string raw_cmd_topic = "raw_cmd_topic";
        std::string cmd_topic = "cmd_topic";
        float previous_x_vel, previous_y_vel, previous_w_vel;
        float x_vel, y_vel, w_vel;
        float smoothed_x_vel, smoothed_y_vel, smoothed_w_vel;
        geometry_msgs::Twist cmd_vel_msg_;
};

#endif