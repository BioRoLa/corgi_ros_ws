#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("teleop_twist_keyboard");
  auto teleop_pub = nh.advertise<std_msgs::msg::String>("teleop_keys", 1);

  // Display instructions and mapping
  puts("Reading from the keyboard and Publishing to Twist!");
  puts("---------------------------");
  puts("Moving around:");
  puts("   u    i    o");
  puts("   j    k    l");
  puts("   m    ,    .");
  puts("");
  puts("Mapping cmd:");
  puts("  k       = stop");
  puts("  i       = forward");
  puts("  ,       = backward");
  puts("  l       = ground_rotate CW");
  puts("  j       = ground_rotate CCW");
  puts("  u       = forward+left (steer left, angle = -10)");
  puts("  m       = backward+left (steer left, angle = -10)");
  puts("  o       = forward+right (steer right, angle = 10)");
  puts("  .       = backward+right (steer right, angle = 10)");
  puts("  y       = increase velocity by 0.05 ");
  puts("  n       = decrease velocity by 0.05 ");
  puts("  a       = reset command");
  puts("Press CTRL-C to quit.");

  // Set terminal to raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(kfd, TCSANOW, &raw);

  signal(SIGINT, quit);

  char c;
  while (rclcpp::ok())
  {
    if (read(kfd, &c, 1) < 0)
    {
      perror("read()");
      exit(-1);
    }
    std_msgs::msg::String msg;
    msg.data = std::string(1, c);
    teleop_pub.publish(msg);
    rclcpp::spin_some(node);
  }
  return 0;
}
