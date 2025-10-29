// #include "rclcpp/rclcpp.hpp"
// #include <sensor_msgs/msg/imu.hpp>
// #include <corgi_msgs/srv/imu.hpp> // This is the service file
// #include <corgi_msgs/msg/headers.hpp> // This is the header file
// #include <fstream>
// #include <mutex>

// std::mutex mutex_;
// sensor_msgs::msg::Imu imu_info;
// void imu_info_cb(sensor_msgs::msg::Imu msg)
// {
//     mutex_.lock();
//     imu_info = msg;
//     mutex_.unlock();
// }

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
     auto node = rclcpp::Node::make_shared("imu_node_listener");
//     rclcpp::Node nh;
//     rclcpp::Rate rate(1000);
//     auto sub = nh.subscribe("imu", 1000, imu_info_cb);
//     std::ofstream file("imu.csv");
//     file << "seq" << "," <<"t.sec" << "," << "t.usec" << "," << 
//             "a.x" << "," << "a.y" << "," << "a.z" << "," << 
//             "w.x" << "," << "w.y" << "," << "w.z" << "," << 
//             "q.x" << "," << "q.y" << "," << "q.z" << "," << "q.w" << 
//             "\n";
//     while(ros::ok) { 
//         rclcpp::spin_some(node);
//         mutex_.lock();
//         // std::cout << imu_info.orientation().x() << "\t" << imu_info.orientation().y() << "\t" << imu_info.orientation().z() << "\t" << imu_info.orientation().w() << "\n";
//         // std::cout << imu_info.acceleration().x() << "\t" << imu_info.acceleration().y() << "\t" << imu_info.acceleration().z() << "\n";
//         file << imu_info.header.seq << ",";
//         file << imu_info.header.stamp.sec << ",";
//         file << imu_info.header.stamp.nsec << ",";
//         file << imu_info.linear_acceleration.x << ",";
//         file << imu_info.linear_acceleration.y << ",";
//         file << imu_info.linear_acceleration.z << ",";
//         file << imu_info.angular_velocity.x << ",";
//         file << imu_info.angular_velocity.y << ",";
//         file << imu_info.angular_velocity.z << ",";
//         file << imu_info.orientation.x << ",";
//         file << imu_info.orientation.y << ",";
//         file << imu_info.orientation.z << ",";
//         file << imu_info.orientation.w << "\n";
//         mutex_.unlock();
//         rate.sleep();
//     }
//     file.close();
// }

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <corgi_msgs/srv/imu.hpp> // This is the service file
#include <corgi_msgs/msg/headers.hpp> // This is the header file
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <fstream>
#include <mutex>
#include <sys/stat.h>

bool trigger = false;


std::mutex mutex_;
sensor_msgs::msg::Imu imu_info;

std::ofstream output_file;
std::string output_file_name = "";
std::string output_file_path = "";

bool file_exists(const std::string &filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
    
    output_file_name = msg.output_filename;

    output_file_name += + "_imu_data";

    if (trigger && msg.output_filename != "") {
        output_file_path = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros_ws/output_data/" + output_file_name;

        int index = 1;
        std::string file_path_with_extension = output_file_path +".csv";
        while (file_exists(file_path_with_extension)) {
            file_path_with_extension = output_file_path + "_" + std::to_string(index) + ".csv";
            index++;
        }
        if (index != 1) output_file_name += "_" + std::to_string(index-1);
        output_file_name += ".csv";

        output_file_path = file_path_with_extension;

        if (!output_file.is_open()) {
            output_file.open(output_file_path);
            output_file << "seq" << "," <<"t.sec" << "," << "t.usec" << "," << 
            "a.x" << "," << "a.y" << "," << "a.z" << "," << 
            "w.x" << "," << "w.y" << "," << "w.z" << "," << 
            "q.x" << "," << "q.y" << "," << "q.z" << "," << "q.w" << 
            "\n";
            RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Recording imu data to %s\n", output_file_name.c_str());
        }
    }
    else {
        if (output_file.is_open()) {
            output_file.close();
            RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Stopped recording data\n");
        }
    }
}

void write_data() {
    if (!output_file.is_open()){
        if (output_file_name != "") RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "Output file is not opened\n");
        return;
    }
    mutex_.lock();
    // std::cout << imu_info.orientation().x() << "\t" << imu_info.orientation().y() << "\t" << imu_info.orientation().z() << "\t" << imu_info.orientation().w() << "\n";
    // std::cout << imu_info.acceleration().x() << "\t" << imu_info.acceleration().y() << "\t" << imu_info.acceleration().z() << "\n";
    output_file << imu_info.header.seq << ","
                << imu_info.header.stamp.sec << ","
                << imu_info.header.stamp.nsec << ","
                << imu_info.linear_acceleration.x << ","
                << imu_info.linear_acceleration.y << ","
                << imu_info.linear_acceleration.z << ","
                << imu_info.angular_velocity.x << ","
                << imu_info.angular_velocity.y << ","
                << imu_info.angular_velocity.z << ","
                << imu_info.orientation.x << ","
                << imu_info.orientation.y << ","
                << imu_info.orientation.z << ","
                << imu_info.orientation.w << "\n";
    mutex_.unlock();

    output_file.flush();

}

void imu_info_cb(sensor_msgs::msg::Imu msg)
{
    mutex_.lock();
    imu_info = msg;
    mutex_.unlock();
}

int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiImu"), "IMU Listener Starts\n");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("imu_node_listener");
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    rclcpp::Rate rate(1000);
    auto sub = nh.subscribe("imu", 1000, imu_info_cb);

    while(rclcpp::ok()) { 
        rclcpp::spin_some(node);

        if (trigger) {
            write_data();
        }
        rate.sleep();
    }

    if (output_file.is_open()) {
        output_file.close();
    }

    ros::shutdown();

    return 0;
}