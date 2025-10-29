#ifndef WHEELED_HPP
#define WHEELED_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include <std_msgs/msg/string.hpp>

#include "joystick_control.hpp"

#include <corgi_msgs/msg/wheel_cmd.hpp>
#include <corgi_msgs/msg/motor_state.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>

class Wheeled
{
public:
    Wheeled();
    ~Wheeled() = default;

private:
    void wheelCmdCallback(const corgi_msgs::msg::WheelCmd::ConstSharedPtr& msg);
    void motorsStateCallback(const corgi_msgs::msg::MotorStateStamped::ConstSharedPtr& msg);
    void steerStateCallback(const corgi_msgs::msg::SteeringCmdStamped::ConstSharedPtr& msg);

    
    rclcpp::Node wnh_;
    ros::Publisher  motor_cmd_pub_;
    ros::Subscriber wheel_cmd_sub_;
    ros::Subscriber motor_state_sub_;
    ros::Subscriber steer_cmd_sub_;

    corgi_msgs::msg::MotorStateStamped current_motor_state_;
    corgi_msgs::msg::MotorCmdStamped current_motor_cmd_;
    corgi_msgs::msg::WheelCmd current_wheel_cmd_;
    corgi_msgs::msg::SteeringCmdStamped current_steer_cmd_;

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &current_motor_state_.module_a,
        &current_motor_state_.module_b,
        &current_motor_state_.module_c,
        &current_motor_state_.module_d
    };

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmds = {
        &current_motor_cmd_.module_a,
        &current_motor_cmd_.module_b,
        &current_motor_cmd_.module_c,
        &current_motor_cmd_.module_d
    };
};


#endif