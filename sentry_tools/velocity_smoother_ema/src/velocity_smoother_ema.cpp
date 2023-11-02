#include <velocity_smoother_ema/velocity_smoother_ema.hpp>

VelocitySmootherEma::VelocitySmootherEma(ros::NodeHandle* nh):nh_(*nh)
{
    nh_.param<double>("/alpha_v", alpha_v, 0.4);
    nh_.param<double>("/alpha_w", alpha_w, 0.4);
    nh_.param<std::string>("/raw_cmd_topic", raw_cmd_topic, "raw_cmd_vel");
    nh_.param<std::string>("/cmd_topic", cmd_topic, "cmd_vel");
    nh_.param<double>("/cmd_rate", cmd_rate, 30.0);

    velocity_sub_ = nh_.subscribe(raw_cmd_topic, 10, &VelocitySmootherEma::twist_callback, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 10, true);
    timer = nh_.createTimer(ros::Duration(1.0 / cmd_rate), &VelocitySmootherEma::update, this);

    previous_x_vel = 0.0;
    previous_y_vel = 0.0;
    previous_w_vel = 0.0;
    x_vel = 0.0;
    y_vel = 0.0;
    w_vel = 0.0;
    smoothed_x_vel = 0.0;
    smoothed_y_vel = 0.0;
    smoothed_w_vel = 0.0;
}

VelocitySmootherEma::~VelocitySmootherEma()
{
    ros::shutdown();
}

void VelocitySmootherEma::twist_callback(const geometry_msgs::Twist::ConstPtr msg)
{
    // ROS_INFO("I RECEIVED A NEW MESSAGE");
    cmd_vel_msg_ = *msg;
}

void VelocitySmootherEma::update(const ros::TimerEvent&)
{
    x_vel = cmd_vel_msg_.linear.x;
    y_vel = cmd_vel_msg_.linear.y;
    w_vel = cmd_vel_msg_.angular.z;

    smoothed_x_vel = alpha_v * x_vel + (1 - alpha_v) * previous_x_vel;
    smoothed_y_vel = alpha_v * y_vel + (1 - alpha_v) * previous_y_vel;
    smoothed_w_vel = alpha_w * w_vel + (1 - alpha_w) * previous_w_vel;

    cmd_vel_msg_.linear.x = smoothed_x_vel;
    cmd_vel_msg_.linear.y = smoothed_y_vel;
    cmd_vel_msg_.angular.z = smoothed_w_vel;

    previous_x_vel = smoothed_x_vel;
    previous_y_vel = smoothed_y_vel;
    previous_w_vel = smoothed_w_vel;    

    velocity_pub_.publish(cmd_vel_msg_);
    // ROS_INFO("PUBLISHING TWIST MESSAGE!");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_smoother_ema");

    ros::NodeHandle nh;

    VelocitySmootherEma vse(&nh);

    ros::spin();

    return 0;
}
