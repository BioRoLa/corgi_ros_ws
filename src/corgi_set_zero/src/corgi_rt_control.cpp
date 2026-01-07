#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/fitted_coefficient.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/trigger_stamped.hpp"

bool sim = true;
bool trigger = false;

LegModel legmodel(sim);
std::array<double, 2> eta;

void trigger_cb(const corgi_msgs::msg::TriggerStamped::SharedPtr msg){
    trigger = msg->enable;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("corgi_rt_control");
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    RCLCPP_INFO(node->get_logger(), "Leg Transform Starts");
    
    for (auto& cmd : motor_cmd_modules) {
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 0.75;
            cmd->kd_l = 0.75;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    for (int i=0; i<2000; i++){
        motor_cmd_modules[0]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[1]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[2]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[3]->theta += 13/2000.0/180.0*M_PI;
        motor_cmd_modules[0]->beta += 40/2000.0/180.0*M_PI;
        motor_cmd_modules[1]->beta -= 40/2000.0/180.0*M_PI;
        motor_cmd_modules[2]->beta -= 40/2000.0/180.0*M_PI;
        motor_cmd_modules[3]->beta += 40/2000.0/180.0*M_PI;

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    for (int i=0; i<1000; i++){
        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Leg Transform Finished");

    while (rclcpp::ok()){
        rclcpp::spin_some(node);

        if (trigger){
            RCLCPP_INFO(node->get_logger(), "Real Time Trajectory Starts");
            int loop_count = 0;
            while (rclcpp::ok()) {
                if (loop_count < 1000) {
                }
                else if (loop_count < 3000) {
                    motor_cmd_modules[0]->theta -= 40/2000.0/180.0*M_PI;
                    motor_cmd_modules[1]->theta -= 40/2000.0/180.0*M_PI;
                    motor_cmd_modules[2]->theta -= 40/2000.0/180.0*M_PI;
                    motor_cmd_modules[3]->theta -= 40/2000.0/180.0*M_PI;
                }
                else if (loop_count < 5000) {
                    // move module A (single leg)
                    eta = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {0.04/1000.0, 0.04/1000.0});
                    motor_cmd_modules[0]->theta = eta[0];
                    motor_cmd_modules[0]->beta = eta[1];
                }
                else if (loop_count < 7000) {
                    // move module A (single leg)
                    eta = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {-0.08/1000.0, 0.0});
                    motor_cmd_modules[0]->theta = eta[0];
                    motor_cmd_modules[0]->beta = eta[1];
                }
                else if (loop_count < 9000) {
                    // move module A (single leg)
                    eta = legmodel.move(motor_cmd_modules[0]->theta, motor_cmd_modules[0]->beta, {0.04/1000.0, -0.04/1000.0});
                    motor_cmd_modules[0]->theta = eta[0];
                    motor_cmd_modules[0]->beta = eta[1];
                }
                else if (loop_count < 10000) {

                }
                else if (loop_count < 12000) {
                    // move module A (single leg)
                    motor_cmd_modules[0]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                    motor_cmd_modules[1]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                    motor_cmd_modules[2]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                    motor_cmd_modules[3]->beta = 40*sin((loop_count-10000)/1000.0*M_PI)/180.0*M_PI;
                }
                else {
                    break;
                } 

                motor_cmd.header.seq = loop_count;

                motor_cmd_pub->publish(motor_cmd);

                loop_count++;

                rclcpp::spin_some(node);
                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Real Time Trajectory Finished");

    rclcpp::shutdown();
    
    return 0;
}
