#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/wheel_cmd.hpp>

// 1) For general debug/log strings from joystick_control
void debugInfoCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  // Print debug/log messages
  ROS_INFO_STREAM("[DEBUG] " << msg->data);
}

// 2) For SteeringCmd messages
void steeringCmdCallback(const corgi_msgs::msg::SteeringCmdStamped::ConstSharedPtr& msg)
{
  ROS_INFO_STREAM("SteeringCmd => angle=" << msg->angle
                  << ", voltage=" << msg->voltage);
}

// 3) For WheelCmd messages
void wheelCmdCallback(const corgi_msgs::msg::WheelCmd::ConstSharedPtr& msg)
{
  ROS_INFO_STREAM("WheelCmd => direction=" << (msg->direction ? "FWD" : "BWD")
                  << ", stop=" << (msg->stop ? "TRUE" : "FALSE")
                  << ", velocity=" << msg->velocity);
}

// 4) For SteeringState
void steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::ConstSharedPtr& msg)
{
  ROS_INFO_STREAM("SteeringState => angle=" << msg->current_angle
                  << ", state=" << (msg->current_state ? "TRUE" : "FALSE")
                  << ", cmd_finish=" << (msg->cmd_finish ));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("display_node");

  // Subscribing to all topics
  auto debug_sub = nh.subscribe("debug_info", 10, debugInfoCallback);
  auto steering_state_sub = nh.subscribe("steering_state", 10, steeringStateCallback);
  auto steering_cmd_sub   = nh.subscribe("steering_cmd", 10, steeringCmdCallback);
  auto wheel_cmd_sub      = nh.subscribe("wheel_cmd", 10, wheelCmdCallback);

  rclcpp::spin(node);
  return 0;
}
