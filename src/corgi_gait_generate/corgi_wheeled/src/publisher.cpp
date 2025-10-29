#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>

/**
 * A node that publishes SteeringStateStamped based on joystick button presses.
 * - B (button index 1) => set current_state=2
 * - Y (button index 3) => set current_state=1
 * - RB (button index 5) => shutdown
 */
class StatePublisher
{
public:
  StatePublisher();

private:
  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy);

  rclcpp::Node nh_;
  ros::Publisher steering_state_pub_;
  ros::Publisher debug_pub_; 
  ros::Subscriber joy_sub_;

  corgi_msgs::msg::SteeringStateStamped current_steering_state_;

  // For indexing the B/Y/RB buttons
  int button_b_;
  int button_y_;
  int button_rb_;
};

StatePublisher::StatePublisher()
{
  // Load parameters or use defaults
  rclcpp::Node pnh("~");
  pnh.param("button_b",  button_b_,  1);
  pnh.param("button_rb", button_rb_, 5);

  // Advertise SteeringState and debug_info
  steering_state_pub_ = nh_.advertise<corgi_msgs::msg::SteeringStateStamped>("/steer/state", 1);
  debug_pub_ = nh_.advertise<std_msgs::msg::String>("debug_info", 10);

  // Subscribe to joystick
  joy_sub_ = nh_.subscribe("joy", 1, &StatePublisher::joyCallback, this);

  // Initialize the steering state
  current_steering_state_.header.stamp = rclcpp::Time::now();
  current_steering_state_.current_angle = 0;
  current_steering_state_.current_state = false; // default
  current_steering_state_.cmd_finish    = 0;

  // Publish an initial debug message
  std_msgs::msg::String dbg;
  dbg.data = "[StatePublisher] Initialized. Listening for B/RB presses.";
  debug_pub_.publish(dbg);
}

void StatePublisher::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy)
{
  // Check we have enough buttons
  if (joy->buttons.size() <= std::max({button_b_, button_rb_}))
  {
    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] Not enough joystick buttons for B/RB!";
    debug_pub_.publish(dbg);
    return;
  }

  // Press B => set current_state=2
  if (joy->buttons[button_b_] == 1)
  {
    current_steering_state_.current_state = true;
    current_steering_state_.header.stamp = rclcpp::Time::now();
    steering_state_pub_.publish(current_steering_state_);

    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] B pressed => set current_state = true";
    debug_pub_.publish(dbg);
  }

  // Press RB => shutdown
  if (joy->buttons[button_rb_] == 1)
  {
    std_msgs::msg::String dbg;
    dbg.data = "[StatePublisher] RB pressed => shutting down!";
    debug_pub_.publish(dbg);
    ros::shutdown();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("state_publisher");
  StatePublisher node;
  rclcpp::spin(node);
  return 0;
}
