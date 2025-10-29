#include "wheeled_cmd.hpp"

WheeledCmd::WheeledCmd(std::string control_mode)
  : pnh_("~"),
    hold_active_(false),
    was_hold_pressed_(false),
    was_reset_pressed_(false),
    current_velocity_(0.0),
    control_mode_(control_mode)
    // Load control mode parameter ("joystick" or "teleop" or "pure"")
{
  
  // pnh_.param("control_mode", control_mode_, std::string("pure"));

  // Load renamed parameters with defaults
  pnh_.param("axis_steer",         axis_steer_,         3);
  pnh_.param("axis_move",          axis_move_,          1);
  pnh_.param("axis_accel",         axis_accel_,         7);
  pnh_.param("button_toggle_hold", button_toggle_hold_, 2); 
  pnh_.param("button_reset",       button_reset_,       0);
  pnh_.param("button_ground",      button_ground_,      3);

  // B state pub
  // x toggle
  // A reset
  // Y groub_rotate
  // 左邊搖桿 前後 
  // 十字鍵盤 上下 加減速度

  // Publishers on existing topics
  steering_cmd_pub_ = nh_.advertise<corgi_msgs::msg::SteeringCmdStamped>("/steer/command", 1);
  wheel_cmd_pub_    = nh_.advertise<corgi_msgs::msg::WheelCmd>("wheel_cmd", 1);
  debug_pub_        = nh_.advertise<std_msgs::msg::String>("debug_info", 10);

  // Subscribe to control inputs based on mode
  if (control_mode_ == "teleop")
  {
    // Teleop mode: subscribe to keyboard key events (std_msgs::msg::String) on "teleop_keys"
    teleop_sub_ = nh_.subscribe("teleop_keys", 1, &WheeledCmd::teleopCallback, this);
  }
  else if (control_mode_ == "pure")
  {
    // Pure code mode: no joystick or teleop input
    // This mode is not implemented in this example
    RCLCPP_WARN(rclcpp::get_logger("CorgiWheeled"), "Pure code mode selected");
  }
  else  // default to joystick control
  {
    joy_sub_ = nh_.subscribe("joy", 1, &WheeledCmd::joyCallback, this);
  }

  // Steering state subscriber remains active
  steering_state_sub_ = nh_.subscribe("/steer/state", 1, &WheeledCmd::steeringStateCallback, this);

  // Timers to republish commands continuously if needed (at 1 kHz)
  wheel_cmd_timer_ = nh_.createTimer(rclcpp::Duration(0.001), &WheeledCmd::wheelCmdTimerCallback, this);
  steer_cmd_timer_ = nh_.createTimer(rclcpp::Duration(0.001), &WheeledCmd::steerCmdTimerCallback, this);

  // Initialize last wheel command: ensure stop is true initially
  last_wheel_cmd_.header.stamp = rclcpp::Time::now();
  last_wheel_cmd_.stop         = true;
  last_wheel_cmd_.direction    = false;
  last_wheel_cmd_.velocity     = 0.0f;
  last_wheel_cmd_.ground_rotate  = false;

  // Initialize steering command
  steering_cmd_.voltage = 0;
  steering_cmd_.angle   = 0.0;
}

double WheeledCmd::clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}

void WheeledCmd::wheelCmdTimerCallback(const rclcpp::TimerEvent&)
{
  if (!last_wheel_cmd_.stop)
  {
    last_wheel_cmd_.header.stamp = rclcpp::Time::now();
    wheel_cmd_pub_.publish(last_wheel_cmd_);
  }
}

void WheeledCmd::steerCmdTimerCallback(const rclcpp::TimerEvent&)
{
  if (steering_cmd_.voltage != 0)
  {
    steering_cmd_.header.stamp = rclcpp::Time::now();
    steering_cmd_pub_.publish(steering_cmd_);
  }
}

void WheeledCmd::steeringStateCallback(const corgi_msgs::msg::SteeringStateStamped::ConstSharedPtr& msg)
{
  current_steering_state_ = *msg;
}

void WheeledCmd::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& joy)
{
  // 1) Reset: when reset button is pressed (edge-triggered)
  bool reset_now = (button_reset_ >= 0 && button_reset_ < (int)joy->buttons.size() && joy->buttons[button_reset_] == 1);
  if (reset_now && !was_reset_pressed_)
  {
    std_msgs::msg::String dbg;
    dbg.data = "[JoyCB] Reset: stopping, zero velocity and angle, hold OFF";
    debug_pub_.publish(dbg);

    hold_active_ = false;
    current_velocity_ = 0.0;

    // Reset steering command
    steering_cmd_.angle   = 0.0;
    steering_cmd_.voltage = 0;
    steering_cmd_.header.stamp = rclcpp::Time::now();
    steering_cmd_pub_.publish(steering_cmd_);

    // Reset wheel command
    last_wheel_cmd_.header.stamp = rclcpp::Time::now();
    last_wheel_cmd_.stop       = true;
    last_wheel_cmd_.direction  = false;
    last_wheel_cmd_.velocity   = 0.0f;
    last_wheel_cmd_.ground_rotate = false;
    wheel_cmd_pub_.publish(last_wheel_cmd_);
  }
  was_reset_pressed_ = reset_now;

  // 2) Toggle hold mode on button press
  bool hold_now = (button_toggle_hold_ >= 0 && button_toggle_hold_ < (int)joy->buttons.size() && joy->buttons[button_toggle_hold_] == 1);
  if (hold_now && !was_hold_pressed_)
  {
    hold_active_ = !hold_active_;
    std_msgs::msg::String dbg;
    dbg.data = "[JoyCB] Toggled hold: " + std::string(hold_active_ ? "ON" : "OFF");
    debug_pub_.publish(dbg);

    if (hold_active_)
    {
      last_wheel_cmd_.stop = false;
      last_wheel_cmd_.header.stamp = rclcpp::Time::now();
      wheel_cmd_pub_.publish(last_wheel_cmd_);
    }
  }
  was_hold_pressed_ = hold_now;

  // 3) Steering command: if steering state is active, use the steering axis
  if (current_steering_state_.current_state)
  {
    if ((int)joy->axes.size() > axis_steer_)
    {
      double steer_input = joy->axes[axis_steer_];
      steering_cmd_.angle = clamp(steer_input * -10.0, -10.0, 10.0);
    }
    else
    {
      steering_cmd_.angle = 0.0;
    }
    steering_cmd_.voltage = 4095; // maximum voltage value
    steering_cmd_.header.stamp = rclcpp::Time::now();
    steering_cmd_pub_.publish(steering_cmd_);
  }

  // 4) Wheel command: update based on move axis
  bool stop = true;
  bool direction = false;
  if (hold_active_)
  {
    stop = false;
    direction = last_wheel_cmd_.direction;
  }
  else
  {
    if ((int)joy->axes.size() > axis_move_)
    {
      double move_input = joy->axes[axis_move_];
      if (std::fabs(move_input) > 0.05)
      {
        stop = false;
        direction = (move_input > 0.0);
      }
    }
  }

  // 5) Adjust velocity using the acceleration axis
  if ((int)joy->axes.size() > axis_accel_)
  {
    double accel_input = joy->axes[axis_accel_];
    if (accel_input > 0.1)
      current_velocity_ += 0.05;
    else if (accel_input < -0.1)
      current_velocity_ -= 0.05;
  }

  // 6)
  if (button_ground_ >= 0 && button_ground_ < (int)joy->buttons.size() && joy->buttons[button_ground_] == 1)
    last_wheel_cmd_.ground_rotate = true;
  else
    last_wheel_cmd_.ground_rotate = false;


  last_wheel_cmd_.stop      = stop;
  last_wheel_cmd_.direction = direction;
  last_wheel_cmd_.velocity  = current_velocity_;
  last_wheel_cmd_.header.stamp = rclcpp::Time::now();
  //  last_wheel_cmd_.ground_rotate = read from button_ground_ (if pressed); 
  wheel_cmd_pub_.publish(last_wheel_cmd_);
}

void WheeledCmd::teleopCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  // First, check if the entire message matches arrow key codes "72" or "80"
  if(msg->data == "72")
  {
      last_wheel_cmd_.velocity += 0.05;
  }
  else if(msg->data == "80")
  {
      last_wheel_cmd_.velocity -= 0.05;
  }
  else
  {
      // Get the first character from the message for single-key commands
      char key = msg->data.empty() ? '\0' : msg->data[0];
      std_msgs::msg::String dbg;
      dbg.data = "[TeleopCB] Received key: " + std::string(1, key);
      debug_pub_.publish(dbg);
      switch(key)
      {
        case 'k': // Stop command
          last_wheel_cmd_.stop = true;
          last_wheel_cmd_.ground_rotate = false;
          break;

        case 'i': // Forward
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = false;
          last_wheel_cmd_.direction = true; // forward
          break;

        case ',': // Backward
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = false;
          last_wheel_cmd_.direction = false; // backward
          break;

        case 'l': // Clockwise ground rotation
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = true;
          last_wheel_cmd_.direction = true;
          break;

        case 'j': // Counter-clockwise ground rotation
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = true;
          last_wheel_cmd_.direction = false;
          break;

        case 'u': // Forward + Left steering
          steering_cmd_.angle = -10;
          steering_cmd_.voltage = 4095;
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = false;
          last_wheel_cmd_.direction = true;
          break;

        case 'm': // Backward + Left steering
          steering_cmd_.angle = -10;
          steering_cmd_.voltage = 4095;
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = false;
          last_wheel_cmd_.direction = false;
          break;

        case 'o': // Forward + Right steering
          steering_cmd_.angle = 10;
          steering_cmd_.voltage = 4095;
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = false;
          last_wheel_cmd_.direction = true;
          break;

        case '.': // Backward + Right steering
          steering_cmd_.angle = 10;
          steering_cmd_.voltage = 4095;
          last_wheel_cmd_.stop = false;
          last_wheel_cmd_.ground_rotate = false;
          last_wheel_cmd_.direction = false;
          break;
        case 'y': // Increase velocity
          last_wheel_cmd_.velocity += 0.05;
          break;
        case 'n': // Decrease velocity
          last_wheel_cmd_.velocity -= 0.05;
          break;
        case 'a': // Reset command
          last_wheel_cmd_.stop = true;
          last_wheel_cmd_.ground_rotate = false;
          last_wheel_cmd_.velocity = 0.0;
          steering_cmd_.angle = 0.0;
          steering_cmd_.voltage = 0;
          steering_cmd_.header.stamp = rclcpp::Time::now();
          steering_cmd_pub_.publish(steering_cmd_);
          break;
        default:
          // Unmapped key: do nothing
          break;
      }
  }
  
  // Publish updated wheel command
  last_wheel_cmd_.header.stamp = rclcpp::Time::now();
  wheel_cmd_pub_.publish(last_wheel_cmd_);

  // Publish updated steering command
  steering_cmd_.header.stamp = rclcpp::Time::now();
  steering_cmd_pub_.publish(steering_cmd_);
}
