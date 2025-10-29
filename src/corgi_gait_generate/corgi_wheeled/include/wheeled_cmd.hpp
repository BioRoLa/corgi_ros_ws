#ifndef WHEEL_CMD_HPP
#define WHEEL_CMD_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <corgi_msgs/msg/wheel_cmd.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>
#include <algorithm> 

class WheeledCmd
{
  public:
    WheeledCmd(std::string control_mode);
    WheeledCmd() : WheeledCmd("joystick") {}
    void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy);
    void teleopCallback(const std_msgs::msg::String::ConstSharedPtr& key_msg);

    void steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::ConstSharedPtr& msg);
    // Timers for continuous publishing when commands are active
    void wheelCmdTimerCallback(const rclcpp::TimerEvent&);
    void steerCmdTimerCallback(const rclcpp::TimerEvent&);
    corgi_msgs::msg::SteeringCmdStamped steering_cmd_;
    // Current steering state feedback (from /steer/state)
    corgi_msgs::msg::SteeringStateStamped current_steering_state_;

  private:
    rclcpp::Node nh_;
    rclcpp::Node pnh_;

    ros::Subscriber joy_sub_;
    ros::Subscriber teleop_sub_;
    ros::Subscriber steering_state_sub_;

    ros::Publisher wheel_cmd_pub_;
    ros::Publisher steering_cmd_pub_;
    ros::Publisher debug_pub_;

    rclcpp::Timer wheel_cmd_timer_;
    rclcpp::Timer steer_cmd_timer_;

    corgi_msgs::msg::WheelCmd last_wheel_cmd_;
    

    int axis_steer_;         
    int axis_move_;          
    int axis_accel_;         
    int button_toggle_hold_; 
    int button_reset_;
    int button_ground_;

    // Control mode ("joystick" or "teleop" or "pure")
    std::string control_mode_;

    // Internal state variables
    bool hold_active_;
    bool was_hold_pressed_;
    bool was_reset_pressed_;
    double current_velocity_;    

    // Helper function: clamp a value between a minimum and maximum
    double clamp(double value, double min_val, double max_val);
};

#endif 
