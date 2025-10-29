#include "force_estimation.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Corgi Wheel Starts");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_wheel");
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

    double init_eta[8] = {0.29670597283903605,-0.0,0.29670597283903605,0.0,0.29670597283903605,0.0,0.29670597283903605};    
    // double init_eta[8] = {18.0/180.0*M_PI,0.0,18.0/180.0*M_PI,0.0,18.0/180.0*M_PI,0.0,18.0/180.0*M_PI,0.0};    
    
    double velocity = 0.0;

    int target_loop = 8000;

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

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Transform Starts\n");

    // transform
    for (int i=0; i<3000; i++) {
        for (int j=0; j<4; j++) {
            motor_cmd_modules[j]->theta += (init_eta[2*j]-17/180.0*M_PI)/3000.0;
            motor_cmd_modules[j]->beta += init_eta[2*j+1]/3000.0;
        }
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

                if (loop_count < 500) {
                    velocity += 0.5 / 500.0;
                }
                else if (loop_count > target_loop - 500 && loop_count < target_loop) {
                    velocity -= 0.5 / 500.0;
                }

                motor_cmd_modules[0]->beta += velocity / 1000.0 / legmodel.radius / 2.0;
                motor_cmd_modules[1]->beta -= velocity / 1000.0 / legmodel.radius / 2.0;
                motor_cmd_modules[2]->beta -= velocity / 1000.0 / legmodel.radius / 2.0;
                motor_cmd_modules[3]->beta += velocity / 1000.0 / legmodel.radius / 2.0;

                motor_cmd.header.seq = loop_count;
                motor_cmd_pub.publish(motor_cmd);

                loop_count++;
                if (loop_count >= target_loop) break;

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}