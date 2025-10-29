#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <iomanip>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>

std::ofstream csv_file;

corgi_msgs::msg::TriggerStamped trigger_msg;
void trigger_cb(const corgi_msgs::msg::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb

void callback(const corgi_msgs::msg::MotorCmdStamped::ConstSharedPtr& msg) {
    if (!trigger_msg.enable) {
        return;  // 如果未啟用，則不儲存數據
    }

    const auto& h = msg->header.stamp;
    const auto& A = msg->module_a;
    const auto& B = msg->module_b;
    const auto& C = msg->module_c;
    const auto& D = msg->module_d;

    csv_file << h.sec << "." << std::setw(9) << std::setfill('0') << h.nsec;

    auto write_module = [](const corgi_msgs::msg::MotorCmd& m) {
        return "," + std::to_string(m.theta) + "," + std::to_string(m.beta) +
               "," + std::to_string(m.kp_r) + "," + std::to_string(m.kp_l) +
               "," + std::to_string(m.ki_r) + "," + std::to_string(m.ki_l) +
               "," + std::to_string(m.kd_r) + "," + std::to_string(m.kd_l) +
               "," + std::to_string(m.torque_r) + "," + std::to_string(m.torque_l);
    };

    csv_file << write_module(A) << write_module(B) << write_module(C) << write_module(D) << "\n";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("motor_cmd_saver");

    std::string filename = "motor_commands.csv";
    csv_file.open(filename);
    csv_file << "time"
             << ",a_theta,a_beta,a_kp_r,a_kp_l,a_ki_r,a_ki_l,a_kd_r,a_kd_l,a_torque_r,a_torque_l"
             << ",b_theta,b_beta,b_kp_r,b_kp_l,b_ki_r,b_ki_l,b_kd_r,b_kd_l,b_torque_r,b_torque_l"
             << ",c_theta,c_beta,c_kp_r,c_kp_l,c_ki_r,c_ki_l,c_kd_r,c_kd_l,c_torque_r,c_torque_l"
             << ",d_theta,d_beta,d_kp_r,d_kp_l,d_ki_r,d_ki_l,d_kd_r,d_kd_l,d_torque_r,d_torque_l\n";

    auto sub = nh.subscribe("motor/command", 1000, callback);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1, trigger_cb);
    
    rclcpp::spin(node);
    csv_file.close();
    return 0;
}
