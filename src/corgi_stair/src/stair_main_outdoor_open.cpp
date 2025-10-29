#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <fstream>

#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/sim_data_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include "leg_model.hpp"
#include "bezier.hpp"
#include "walk_gait.hpp"
#include "stair_climb.hpp"

#define INIT_THETA (M_PI*17.0/180.0)
#define INIT_BETA (0.0)

corgi_msgs::msg::TriggerStamped trigger_msg;
corgi_msgs::msg::SimDataStamped sim_data;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb

void robot_cb(const corgi_msgs::msg::SimDataStamped msg) {
    sim_data = msg;
}//end robot_cb

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("stair_climb");
    auto motor_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1, trigger_cb);
    auto robot_sub = nh.subscribe<corgi_msgs::msg::SimDataStamped>("sim/data", 1, robot_cb);
    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::msg::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    // pid command
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp_r = 150.0;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 150.0;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 
    std::ofstream command_pitch_CoM("command_pitch_CoM.csv");
    command_pitch_CoM << "Time," << "Trigger," << "pitch," << "CoM_x," << "CoM_z," << "\n";

    /* Setting variable */
    const std::vector<std::array<double, 2>> stair_size = {{1.0, 0.078}, 
                                                            {0.399, 0.089}, 
                                                            {0.401, 0.092}};    // 工博館
    // const std::vector<std::array<double, 2>> stair_size = {{1.0, 0.129}, 
    //                                                         {0.543, 0.134}, 
    //                                                         {0.325, 0.130}, 
    //                                                         {0.324, 0.130}, 
    //                                                         {0.338, 0.132},
    //                                                         {0.340, 0.135},
    //                                                         {0.339, 0.117}};    // 繡山演講廳
    // const std::vector<std::array<double, 2>> stair_size = {{1.0, 0.112}, 
    //                                                         {0.285, 0.112}, 
    //                                                         {0.288, 0.115}, 
    //                                                         {0.313, 0.131}, 
    //                                                         {0.315, 0.135},
    //                                                         {0.312, 0.130},
    //                                                         {0.315, 0.133},
    //                                                         {0.312, 0.134},
    //                                                         {0.312, 0.121}};    // 系館側門
    // const std::vector<std::array<double, 2>> stair_size = {{1.0, 0.109}, 
    //                                                         {0.449, 0.098}, 
    //                                                         {0.458, 0.100}};    // 化學系館
    // const std::vector<std::array<double, 2>> stair_size = {{1.0, 0.140}, 
    //                                                         {0.364, 0.131}, 
    //                                                         {0.358, 0.095}};    // 舊體
    // const std::vector<std::array<double, 2>> stair_size = {{1.0, 0.112}, 
    //                                                         {0.505, 0.102}, 
    //                                                         {0.486, 0.113}};    // 新體

    enum STATES {INIT, TRANSFORM, WAIT, WALK, STAIR, UPPER_WALK, END};
    const std::array<double, 2> CoM_bias = {0.0, 0.0};
    const int sampling_rate = 1000;
    const int transform_count = 5*sampling_rate; // 5s
    // const double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    // const double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    const double init_eta[8] = {1.857467698281913, 0.4791102940603915, 1.6046663223045279, 0.12914729012802004, 1.6046663223045279, -0.12914729012802004, 1.857467698281913, -0.4791102940603915};  // stand height 0.25, step length 0.3
    const double velocity = 0.1; // velocity for walk gait
    const double stand_height = 0.25; // stand height for walk gait
    const double step_length = 0.3; // step length for walk gait and stair gait
    const double step_height = 0.06; // step height for walk gait
    const double max_step_length = 0.3;
    const double min_step_length = 0.2;

    /* Initial variable */
    rclcpp::Rate rate(sampling_rate);
    WalkGait walk_gait(false, CoM_bias[0], sampling_rate);
    StairClimb stair_climb(false, CoM_bias, sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA , INIT_BETA , INIT_BETA , INIT_BETA }}};   // init eta (wheel mode)
    
    /* Other variable */
    STATES state = INIT, last_state = INIT;
    double transform_ratio;
    bool trigger;
    int command_count;
    double pitch;
    std::array<double, 2> CoM;
    double max_cal_time = 0.0;
    std::array<int, 4> swing_phase;
    double hip_x, to_enter_d;
    double exp_robot_x = 0.0; // expected robot x position
    std::array<bool, 4> if_contact_edge, last_if_contact_edge;
    std::array<int, 4> step_count;
    double optimal_foothold;
    int min_steps, max_steps;
    bool change_step_length;
    double D_sum, H_sum;


    /* Behavior loop */
    auto start = std::chrono::high_resolution_clock::now();
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);
    while (rclcpp::ok()) {
        auto one_loop_start = std::chrono::high_resolution_clock::now();
        rclcpp::spin_some(node);
        if (state == END) {
            break;
        }//end if
        // state machine
        switch (state) {
            case INIT:
                transform_ratio = 0.0;
                trigger = false;
                command_count = 0;
                D_sum = 0.0;
                H_sum = 0.0;
                break;
            case TRANSFORM:
                transform_ratio += 1.0 / transform_count;
                for (int i=0; i<4; i++) {
                    eta_list[0][i] = INIT_THETA + transform_ratio * (init_eta[i*2]   - INIT_THETA);
                    eta_list[1][i] = INIT_BETA  + transform_ratio * (init_eta[i*2+1] - INIT_BETA);
                    eta_list[1][i] = (i == 1 || i == 2)? eta_list[1][i] : -eta_list[1][i];  
                }//end for
                break;
            case WAIT:
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    walk_gait.initialize(current_eta);
                }//end if
                break;
            case WALK:
                /* Adjust step length of walk gait */
                optimal_foothold = stair_climb.get_optimal_foothold(stair_size[0][1], false);
                exp_robot_x += velocity / sampling_rate; // expected robot x position
                hip_x = exp_robot_x + 0.222; // front hip
                to_enter_d = stair_size[0][0] - optimal_foothold -(max_step_length/2) - hip_x; // Remaining distance to the gait change point
                min_steps = static_cast<int>(std::ceil(to_enter_d / (max_step_length/2)));   // max step length = 30cm
                max_steps = static_cast<int>(std::floor(to_enter_d / (min_step_length/2)));  // min step length = 20cm
                // std::cout << "to_enter_d: " << to_enter_d << std::endl;
                // std::cout << "hip: " << hip_x << std::endl;
                change_step_length = false;
                if (min_steps <= max_steps) {
                    change_step_length = true;
                    walk_gait.set_step_length(to_enter_d / (min_steps/2.0)); 
                }

                eta_list = walk_gait.step();
                if (change_step_length && walk_gait.if_touchdown() && (swing_phase[0]==1 || swing_phase[1]==1)) { // walk_gait apply new step_length
                    std::cout << "step_length:" << to_enter_d / (min_steps/2.0) << std::endl;
                }
                command_pitch_CoM << rclcpp::Time::now() << "," << (int)trigger_msg.enable << "," << 0.0 << "," << exp_robot_x << "," << stand_height << "\n";
                command_count ++;
                break;
            case STAIR:
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    stair_climb.initialize(current_eta, velocity, exp_robot_x);
                    for (int i=0; i<stair_size.size(); i++) {
                        D_sum += stair_size[i][0];
                        H_sum += stair_size[i][1];
                        stair_climb.add_stair_edge(D_sum, H_sum);
                    }//end for
                }//end if
                eta_list = stair_climb.step();
                pitch = stair_climb.get_pitch();
                CoM = stair_climb.get_CoM();
                command_pitch_CoM << rclcpp::Time::now() << "," << (int)trigger_msg.enable << "," << pitch << "," << CoM[0] << "," << CoM[1] << "\n";
                command_count ++;
                break;
            case UPPER_WALK:
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    walk_gait.initialize(current_eta, step_length);
                    walk_gait.set_step_height(0.04);
                }//end if
                pitch = 0.0;
                eta_list = walk_gait.step();
                command_count ++;
                break;
            default:
                break;
        }//end switch
        last_state = state;

        // next state
        switch (state) {
            case INIT:
                state = TRANSFORM;
                break;
            case TRANSFORM:
                if (transform_ratio > 1.0) {
                    state = WAIT;
                }//end if
                break;
            case WAIT:
                if (trigger_msg.enable) {
                    state = WALK;
                }//end if
                break;
            case WALK:
                // Entering stair climbing phase
                swing_phase = walk_gait.get_swing_phase();
                if (walk_gait.if_touchdown() && (swing_phase[0]==1 || swing_phase[1]==1)) { // hind leg touched down (front leg start to swing)
                    if (hip_x + (max_step_length/2) >= stair_size[0][0] - optimal_foothold) {   // max next foothold >= keep_stair_d_front_max, to swing up stair
                        state = STAIR;
                        std::cout << "Enter stair climbing phase." << std::endl;
                    }//end if
                }//end if
                break;
            case STAIR:
                if (!stair_climb.if_any_stair()) {
                    state = UPPER_WALK;
                }//end if  
                break;
            case UPPER_WALK:
                step_count = walk_gait.get_step_count();
                if (step_count[0] >= 2 && step_count[1] >= 2 && step_count[2] >= 2 && step_count[3] >= 2) { // all legs have stepped at least twice
                    state = END;
                }//end if  
                break;
            default:
                break;
        }//end switch

        if_contact_edge = stair_climb.get_contact_edge_leg();
        for (int i=0; i<4; i++) {
            if (if_contact_edge[i] && !last_if_contact_edge[i]) {
                std::cout << "Command: " << command_count << ". Leg " << i << " is contacting the stair edge." << std::endl;
            }//end if
        }//end for
        last_if_contact_edge = if_contact_edge;

        /* Publish motor commands */
        for (int i=0; i<4; i++) {
            if (eta_list[0][i] > M_PI*160.0/180.0) {
                std::cout << "Leg " << i << " exceed upper bound." << std::endl;
                eta_list[0][i] = M_PI*160.0/180.0;
            }//end if 
            if (eta_list[0][i] < M_PI*16.9/180.0) {
                std::cout << "Leg " << i << " exceed lower bound." << std::endl;
                eta_list[0][i] = M_PI*16.9/180.0;
            }//end if 
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2)? (eta_list[1][i]-pitch) : -(eta_list[1][i]-pitch);
        }//end for
        motor_pub.publish(motor_cmd);
        auto one_loop_end = std::chrono::high_resolution_clock::now();
        auto one_loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(one_loop_end - one_loop_start);
        if (one_loop_duration.count() > max_cal_time) {
            max_cal_time = one_loop_duration.count();
            std::cout << "max time: " << max_cal_time << " us" << std::endl;
        }//end if
        rate.sleep();
    }//end while
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "max time: " << max_cal_time << " us" << std::endl;
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    std::cout << "total count: " << command_count << std::endl;

    
    command_pitch_CoM.close();
    ros::shutdown();
    return 0;
}//end main

