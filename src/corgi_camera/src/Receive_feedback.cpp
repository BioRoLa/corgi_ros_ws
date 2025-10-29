#include <X11/Xlib.h>  // XInitThreads
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

class ZedListener {
public:
  ZedListener()
    : it_(nh_)
  {
    // Odometry subscriber
    odom_sub_ = nh_.subscribe<nav_msgs::msg::Odometry>(
      "/zedxm/zed_node/odom", 1,
      &ZedListener::odomCb, this,
      ros::TransportHints().tcpNoDelay());

    // IMU subscriber
    imu_sub_ = nh_.subscribe<sensor_msgs::msg::Imu>(
      "/zedxm/zed_node/imu/data", 1,
      &ZedListener::imuCb, this,
      ros::TransportHints().tcpNoDelay());

    // RGB image subscriber (raw, tcpNoDelay + unreliable)
    rgb_sub_ = it_.subscribe(
      "/zedxm/zed_node/rgb/image_rect_color", 1,
      &ZedListener::rgbCb, this,
      image_transport::TransportHints(
        "raw",
        ros::TransportHints().tcpNoDelay().unreliable()));

    // Depth image subscriber (raw, tcpNoDelay + unreliable)
    depth_sub_ = it_.subscribe(
      "/zedxm/zed_node/depth/depth_registered", 1,
      &ZedListener::depthCb, this,
      image_transport::TransportHints(
        "raw",
        ros::TransportHints().tcpNoDelay().unreliable()));

    // Publishers: Vector3 for position & linear velocity
    pos_pub_ = nh_.advertise<geometry_msgs::msg::Vector3>(
      "odometry/position", 1);
    vel_pub_ = nh_.advertise<geometry_msgs::msg::Vector3>(
      "odometry/velocity", 1);

    RCLCPP_INFO(rclcpp::get_logger("CorgiCamera"), "ZedListener initialized.");
  }

  void spin() {
    XInitThreads();  // 必須最先呼叫，確保 OpenCV GUI 在多執行緒下安全
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    cv::destroyAllWindows();
  }

private:
  rclcpp::Node nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber odom_sub_, imu_sub_;
  image_transport::Subscriber rgb_sub_, depth_sub_;
  ros::Publisher pos_pub_, vel_pub_;

  // Odometry callback
  void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    // 發佈 position
    geometry_msgs::msg::Vector3 p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;
    pos_pub_.publish(p);

    // 發佈 linear velocity
    geometry_msgs::msg::Vector3 v;
    v.x = msg->twist.twist.linear.x;
    v.y = msg->twist.twist.linear.y;
    v.z = msg->twist.twist.linear.z;
    vel_pub_.publish(v);
  }

  // IMU callback
  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    const auto& o = msg->orientation;
    const auto& g = msg->angular_velocity;
    const auto& a = msg->linear_acceleration;
    RCLCPP_INFO(rclcpp::get_logger("CorgiCamera"), "IMU Ori[%.3f,%.3f,%.3f,%.3f] "
             "Gyro[%.3f,%.3f,%.3f] "
             "Acc[%.3f,%.3f,%.3f]",
             o.x,o.y,o.z,o.w,
             g.x,g.y,g.z,
             a.x,a.y,a.z);
  }

  // RGB callback
  void rgbCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::imshow("ZED RGB", img);
      cv::waitKey(1);
    } catch (...) {}
  }

  // Depth callback
  void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
      cv::Mat depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
      cv::imshow("ZED Depth", depth);
      cv::waitKey(1);
    } catch (...) {}
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("zed_listener");
  ZedListener node;
  node.spin();
  return 0;
}
