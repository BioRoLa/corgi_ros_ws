#ifndef WHEELED_GEN_HPP
#define WHEELED_GEN_HPP


#include "Simple_fsm.hpp"
#include "wheeled_cmd.hpp"
#include <corgi_msgs/msg/wheel_cmd.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>

class Wheeled : public GaitSelector, public WheeledCmd
{
public:
    Wheeled(rclcpp::Node& nh);
    ~Wheeled() = default;
    void Roll(int pub_time, int do_pub, bool dir, bool ground_rotate, int voltage, float angle);
private:
    void wheelCmdCallback(const corgi_msgs::msg::WheelCmd::ConstSharedPtr& msg);
    void steerCmdCallback(const corgi_msgs::msg::SteeringCmdStamped::ConstSharedPtr& msg);

    ros::Subscriber wheel_cmd_sub_;
    ros::Subscriber steer_cmd_sub_;

    corgi_msgs::msg::WheelCmd current_wheel_cmd_;
    corgi_msgs::msg::SteeringCmdStamped current_steer_cmd_;
    
    float beta_adjustment;
    float beta_adjustment_r;
    float beta_adjustment_l;

    void MoveLinear(bool dir);
    void MoveAngular(bool dir);

};

#endif
