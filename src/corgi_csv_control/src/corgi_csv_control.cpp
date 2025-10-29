#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_csv_control");
    auto motor_cmd_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmds = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    if (argc < 2){
        RCLCPP_INFO(rclcpp::get_logger("CorgiCsvControl"), "Please input csv file path\n");
        return 1;
    }
    
    std::string csv_file_path;
    csv_file_path = std::getenv("HOME");
    csv_file_path += "/corgi_ws/corgi_ros_ws/input_csv/";
    csv_file_path += argv[1];
    csv_file_path += ".csv";
    

    std::ifstream csv_file(csv_file_path);
    if (!csv_file.is_open()) {
        RCLCPP_INFO(rclcpp::get_logger("CorgiCsvControl"), "Failed to open the CSV file\n");
        return 1;
    }

    std::string line;
    

    RCLCPP_INFO(rclcpp::get_logger("CorgiCsvControl"), "Leg Transform Starts\n");
    
    for (int i=0; i<5000; i++){
        std::getline(csv_file, line);
        std::vector<double> columns;
        std::stringstream ss(line);
        std::string item;
        
        for (auto& cmd : motor_cmds){
            std::getline(ss, item, ',');
            cmd->theta = std::stod(item);

            std::getline(ss, item, ',');
            cmd->beta = std::stod(item);

            cmd->kp_r = 90;
            cmd->kp_l = 90;
            cmd->ki_r = 0;
            cmd->ki_l = 0;
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub.publish(motor_cmd);

        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiCsvControl"), "Leg Transform Finished\n");

    
    while (rclcpp::ok()){
        rclcpp::spin_some(node);

        if (trigger){
            RCLCPP_INFO(rclcpp::get_logger("CorgiCsvControl"), "CSV Trajectory Starts\n");

            int seq = 0;
            while (rclcpp::ok() && std::getline(csv_file, line)) {
                std::vector<double> columns;
                std::stringstream ss(line);
                std::string item;
                
                for (auto& cmd : motor_cmds){
                    std::getline(ss, item, ',');
                    cmd->theta = std::stod(item);

                    std::getline(ss, item, ',');
                    cmd->beta = std::stod(item);

                    cmd->kp_r = 90;
                    cmd->kp_l = 90;
                    cmd->ki_r = 0;
                    cmd->ki_l = 0;
                    cmd->kd_r = 1.75;
                    cmd->kd_l = 1.75;
                }

                motor_cmd.header.seq = seq;

                motor_cmd_pub.publish(motor_cmd);

                seq++;

                rate.sleep();
            }
            break;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiCsvControl"), "CSV Trajectory Finished\n");

    ros::shutdown();
    
    return 0;
}
