#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <corgi_msgs/srv/imu.hpp>
#include <corgi_msgs/msg/headers.hpp>
#include "cx5.hpp" 
#include <sys/time.h>
#include <mutex>
#include <thread>

std::shared_ptr<CX5_AHRS> imu;
std::mutex cb_lock;

enum SensorMode {
    REST = 0,
    CALIBRATION = 1,
    SENSOR = 2,
    RESET = 3
};            

int mode = REST;
bool cb(corgi_msgs::srv::imu::Request &req, corgi_msgs::srv::imu::Response &res){
    cb_lock.lock();
    switch (req.mode) {
        case REST:
            mode = REST;
            res.mode = REST;
            RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Mode set to REST");
            break;
        case CALIBRATION:
            RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Calibrating...");
            mode = SENSOR;
            res.mode = SENSOR;
            imu->calibrate(1000);  // 1 second averaging
            break;
        case RESET:
            mode = RESET;
            res.mode = RESET;
            RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Mode set to RESET");
            break;
        case SENSOR:
            mode = SENSOR;
            res.mode = SENSOR;
            RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Mode set to SENSOR");
            break;
        default:
            RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Invalid mode");
            return false;
    }
    return true;
    cb_lock.unlock();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("imu_node");
    
    printf("Starting IMU node\n");

    // imu = std::make_shared<CX5_AHRS>("/dev/ttyACM0", 921600, 1000, 500); //change the port, baudrate, sensor sample rate, filter sample rate (origin port: ttyTHS0)
    // imu = std::make_shared<CX5_AHRS>("/dev/ttyTHS0", 921600, 1000, 500);

    imu = std::make_shared<CX5_AHRS>("/dev/ttyTHS0", 921600, 1000, 500); //change the port, baudrate, sensor sample rate, filter sample rate

    rclcpp::Rate rate(1000);

    auto pub = nh.advertise<sensor_msgs::msg::Imu>("imu", 1000); // "imu" is the topic name
    ros::ServiceServer srv = nh.advertiseService("imu_service", cb);

    std::thread imu_thread([&]() {
        imu->start();
    });

    sensor_msgs::msg::Imu imu_msg;
    corgi_msgs::msg::Headers headers_msg;
    headers_msg.frame_id = "imu_base";

    Eigen::Vector3f acceleration, twist;
    Eigen::Quaternionf orientation;
    int seq = 0;

    while (rclcpp::ok()) {
        imu->get(acceleration, twist, orientation);

        timeval currentTime;
        gettimeofday(&currentTime, nullptr);

        headers_msg.seq = seq++;
        headers_msg.stamp.sec = currentTime.tv_sec;
        headers_msg.stamp.nsec = currentTime.tv_usec * 1000;
        
        imu_msg.header.seq = headers_msg.seq;
        imu_msg.header.stamp = headers_msg.stamp;

        imu_msg.linear_acceleration.x = acceleration.x();
        imu_msg.linear_acceleration.y = acceleration.y();
        imu_msg.linear_acceleration.z = acceleration.z();

        imu_msg.angular_velocity.x = twist.x();
        imu_msg.angular_velocity.y = twist.y();
        imu_msg.angular_velocity.z = twist.z();

        imu_msg.orientation.x = orientation.x();
        imu_msg.orientation.y = orientation.y();
        imu_msg.orientation.z = orientation.z();
        imu_msg.orientation.w = orientation.w();

        pub.publish(imu_msg);

        rate.sleep();
    }
    return 0;
}