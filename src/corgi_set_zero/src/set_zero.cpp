#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "corgi_utils/leg_model.hpp"
#include "corgi_utils/fitted_coefficient.hpp"

#include "corgi_msgs/msg/motor_cmd_stamped.hpp"
#include "corgi_msgs/msg/motor_state_stamped.hpp"

std::array<double, 2> eta;
corgi_msgs::msg::MotorStateStamped motor_state;

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::SharedPtr msg){
    motor_state = *msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("corgi_set_zero");
    auto motor_cmd_pub = node->create_publisher<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto motor_state_sub = node->create_subscription<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    RCLCPP_INFO(node->get_logger(), "Set Zero Starts");

    double theta_err[4];
    double beta_err[4];

    for (int i=0; i<1000; i++) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->theta = motor_state_modules[i]->theta;
        motor_cmd_modules[i]->beta = motor_state_modules[i]->beta;
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;

        theta_err[i] = (17/180.0*M_PI-motor_state_modules[i]->theta);
        beta_err[i] = (-motor_state_modules[i]->beta);

        if (motor_cmd_modules[i]->theta < 17/180.0*M_PI) {
            RCLCPP_WARN(node->get_logger(), "Theta value too small, shutting down");
            rclcpp::shutdown();
            return 0;
        }
    }

    for (int i=0; i<2000; i++){
        for (int j=0; j<4; j++){
            motor_cmd_modules[j]->theta += theta_err[j]/2000.0;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    for (int i=0; i<5000; i++){
        for (int j=0; j<4; j++){
            motor_cmd_modules[j]->beta += beta_err[j]/5000.0;
        }

        motor_cmd.header.seq = -1;

        motor_cmd_pub->publish(motor_cmd);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Set Zero Completed");
    rclcpp::shutdown();
    
    return 0;
}
