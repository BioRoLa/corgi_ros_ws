#include "force_estimation.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Corgi Sim Leg Starts");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_sim_leg");
    auto motor_cmd_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    // initialize motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 1;
            cmd->kd_l = 1;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    // stay
    for (int i=0; i<1000; i++) {
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Transform Starts\n");

    // transform
    for (int i=0; i<3000; i++) {
        motor_cmd_modules[0]->beta += M_PI/3000.0;
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Transform Finished\n");

    // stay
    for (int i=0; i<2000; i++) {
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Controller Starts ...\n");

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                if (loop_count < 36000) {
                    motor_cmd_modules[0]->beta -= M_PI/6000.0;
                }
                else {
                    break;
                }

                motor_cmd.header.seq = loop_count;
                motor_cmd_pub.publish(motor_cmd);

                loop_count++;

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}