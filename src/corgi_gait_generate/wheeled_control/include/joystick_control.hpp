#ifndef JOYSTICK_CONTROL_HPP
#define JOYSTICK_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <cmath>
#include <string>

// Custom messages
#include <corgi_msgs/msg/steering_state_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/wheel_cmd.hpp>

class JoystickControl
{
public:
  JoystickControl();

private:
  // Callbacks
  void steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::ConstSharedPtr& msg);
  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy);

  // Timer callback for 1 kHz
  void wheelCmdTimerCallback(const rclcpp::TimerEvent&);
  void steerCmdTimerCallback(const rclcpp::TimerEvent&);

  double clamp(double value, double min_val, double max_val);

  // ROS members
  rclcpp::Node nh_;
  ros::Publisher steering_cmd_pub_;
  ros::Publisher wheel_cmd_pub_;
  ros::Publisher debug_pub_;

  ros::Subscriber steering_state_sub_;
  ros::Subscriber joy_sub_;

  // Timer at 1 kHz
  rclcpp::Timer wheel_cmd_timer_;
  rclcpp::Timer steering_cmd_timer_;
  // Store the current steering state
  corgi_msgs::msg::SteeringStateStamped current_steering_state_;
  corgi_msgs::msg::SteeringCmdStamped steer;
  // Indices for axes/buttons
  int axis_left_right_;
  int axis_forward_back_;
  int axis_velocity_;
  int button_hold_;
  int button_reset_;

  // Logic
  bool hold_active_;
  bool was_hold_pressed_;
  bool was_reset_pressed_;

  bool last_direction_;   
  double current_velocity_;

  // The last WheelCmd we published
  corgi_msgs::msg::WheelCmd last_wheel_cmd_;
};

#endif // JOYSTICK_CONTROL_HPP
