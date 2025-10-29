#include "force_estimation.hpp"
#include "mpc.hpp"

bool trigger = false;
corgi_msgs::msg::ForceStateStamped force_state;

void force_state_cb(const corgi_msgs::msg::ForceStateStamped msg){
    force_state = msg;
}

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Corgi WLW Starts");

    ModelPredictiveController mpc;
    mpc.load_config();
    mpc.target_loop = 1500;

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_wlw");
    auto motor_cmd_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto contact_pub = nh.advertise<corgi_msgs::msg::ContactStateStamped>("odometry/contact", 1000);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    auto force_state_sub = nh.subscribe<corgi_msgs::msg::ForceStateStamped>("force/state", 1000, force_state_cb);
    
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    corgi_msgs::msg::ContactStateStamped contact_state;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<corgi_msgs::msg::ContactState*> contact_state_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    // initialize gait
    double CoM_bias = 0.0;

    double velocity = 0.16;

    auto gait_selector = std::make_shared<GaitSelector>(nh, sim, CoM_bias, 1000);
    gait_selector->do_pub = 0;
    gait_selector->stand_height = 0.16;
    gait_selector->step_length = 0.4;
    gait_selector->velocity = 0;

    for (int i=0; i<4; i++) {
        gait_selector->current_shift[i] = 0;
        gait_selector->current_stand_height[i] = gait_selector->stand_height;
    }

    Hybrid hybrid_gait(gait_selector); 

    std::cout << "hybrid" << std::endl;
    hybrid_gait.Initialize(1, 1);

    for(int i=0; i<4; i++){
        gait_selector->eta[i][0] = gait_selector->next_eta[i][0]; 
        gait_selector->eta[i][1] = gait_selector->next_eta[i][1];
    }

    hybrid_gait.update_nextFrame();
    gait_selector->body = gait_selector->next_body;
    gait_selector->hip = gait_selector->next_hip;
    gait_selector->foothold = gait_selector->next_foothold;

    double init_eta[8];
    for (int i=0; i<4; i++) {
        init_eta[2*i] = gait_selector->eta[i][0];
        init_eta[2*i+1] = -gait_selector->eta[i][1];
    }

    init_eta[3] *= -1;
    init_eta[5] *= -1;
    
    std::array<std::array<double, 4>, 2> eta_list = {{{17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI}, {0, 0, 0, 0}}};

    // initialize motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 1.5;
            cmd->kd_l = 1.5;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Wait ...\n");
    
    if (!sim) {
        for (int i=0; i<3000; i++) {
            rate.sleep();
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Transform Starts\n");

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

    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Transform Finished\n");

    // stay
    for (int i=0; i<2000; i++) {
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Wait For Odometry Node Initializing ...\n");

            // wait for odometry node
            if (!sim) {
                for (int i=0; i<3000; i++) {
                    for (auto& state: contact_state_modules) {
                        state->contact = true;
                    }
                    contact_pub.publish(contact_state);
                    rate.sleep();
                }
            }
            else {
                for (int i=0; i<1000; i++) {
                    for (auto& state: contact_state_modules) {
                        state->contact = true;
                    }
                    contact_pub.publish(contact_state);
                    rate.sleep();
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Controller Starts ...\n");

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                for (int i=0; i<4; i++) {
                    // if (gait_selector->swing_phase[i] == 1) {
                    //     check_contact_state(i, contact_state_modules);
                    // }
                    std::cout << gait_selector->duty[i] << ", ";
                    if (gait_selector->duty[i] < 0.7 && gait_selector->duty[i] > 0.1) {
                        contact_state_modules[i]->contact = true;
                    }
                    else {
                        contact_state_modules[i]->contact = false;
                    }
                }
                std::cout << std::endl;

                // update target vel and pos
                if (loop_count < 1000) {
                    mpc.target_vel_x += velocity/1000.0;
                    hybrid_gait.change_Velocity(mpc.target_vel_x);
                }
                if (loop_count > mpc.target_loop*10-1000 && loop_count < mpc.target_loop*10) {
                    mpc.target_vel_x -= velocity/1000.0;
                    hybrid_gait.change_Velocity(mpc.target_vel_x);
                }

                // mpc.target_vel_x = velocity;
                // hybrid_gait.change_Velocity(mpc.target_vel_x);

                mpc.target_pos_x += mpc.target_vel_x * mpc.dt / 10.0;

                // get next eta
                for(int i=0; i<4; i++){
                    gait_selector->eta[i][0] = gait_selector->next_eta[i][0]; 
                    gait_selector->eta[i][1] = gait_selector->next_eta[i][1];
                }

                hybrid_gait.Step();

                for (int i=0; i<4; i++) {
                    eta_list[0][i] = gait_selector->next_eta[i][0];
                    eta_list[1][i] = -gait_selector->next_eta[i][1];
                }

                for (int i=0; i<4; i++) {
                    if (eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 0 || i == 3) ? eta_list[1][i] : -eta_list[1][i];
                }

                motor_cmd.header.seq = loop_count;
                motor_cmd_pub.publish(motor_cmd);

                contact_state.header.seq = loop_count;
                // contact_pub.publish(contact_state);

                std::cout << std::fixed << std::setprecision(3);
                std::cout << "Target Position X: " << mpc.target_pos_x << std::endl << std::endl;
                std::cout << "Current Velocity X: " << mpc.target_vel_x << std::endl << std::endl;
                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                loop_count++;
                if (loop_count >= mpc.target_loop*10) break;

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}