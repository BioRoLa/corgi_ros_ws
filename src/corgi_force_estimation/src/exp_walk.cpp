#include "force_estimation.hpp"
#include "walk_gait.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Corgi Walk Starts");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_walk");
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

    // initialize gait
    // sim, h25, sl0.3
    double init_eta[8] = {1.9107879909396832,0.4678492649476779,1.6644526642960358,0.1256503306098462,1.6644526642960358,-0.1256503306098462,1.9107879909396832,-0.4678492649476779};
    
    // sim, h20, sl0.3
    // double init_eta[8] = {1.5145026111157143,0.573900181729176,1.1975094246645916,0.1586552621864014,1.1975094246645916,-0.1586552621864014,1.5145026111157143,-0.573900181729176};
    

    // real, h25, sl0.3
    // double init_eta[8] = {1.857467698281913,0.4791102940603916,1.6046663223045279,0.12914729012802004,1.6046663223045279,-0.12914729012802004,1.857467698281913,-0.4791102940603916};
    
    // real, h20, sl0.3
    // double init_eta[8] = {1.4863321792421085,0.6075431293162905,1.1354779956465793,0.16425262030677687,1.1354779956465793,-0.16425262030677687,1.4863321792421085,-0.6075431293162905};


    WalkGait walk_gait(sim, 0, 1000);
    walk_gait.initialize(init_eta);

    double velocity        = 0.1;  // 0.1, 0.15
    double stand_height    = 0.25;  // 0.2, 0.25
    double step_length     = 0.3;
    double step_height     = 0.04;
    
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);

    std::array<std::array<double, 4>, 2> eta_list = {{{17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI}, {0, 0, 0, 0}}};
    int target_loop = 20000;

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

                // if (loop_count > target_loop-3000 && loop_count < target_loop) {
                //     velocity -= 0.1/3000.0;
                //     walk_gait.set_velocity(velocity);
                // }

                // get next eta
                eta_list = walk_gait.step();

                for (int i=0; i<4; i++) {
                    if (eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
                }

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