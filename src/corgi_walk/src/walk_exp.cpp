#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>

#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include "walk_gait.hpp"
#include "leg_model.hpp"
#include "bezier.hpp"

#define INIT_THETA (M_PI*17.0/180.0)
#define INIT_BETA (0.0)

corgi_msgs::msg::TriggerStamped trigger_msg;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("walk_test");
    auto motor_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1, trigger_cb);
    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::msg::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp_r = 150;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 150;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 

    enum STATES {INIT, TRANSFORM, WAIT, WALK, END};
    const std::array<double, 2> CoM_bias = {0.0, 0.0};
    const int sampling_rate = 1000;
    const int transform_count = 5*sampling_rate; // 5s
    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    // double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    const double init_eta[8] = {1.857467698281913, 0.4791102940603915, 1.6046663223045279, 0.12914729012802004, 1.6046663223045279, -0.12914729012802004, 1.857467698281913, -0.4791102940603915};  // stand height 0.25, step length 0.3, swing time 0.2
    // const double init_eta[8] = {1.8264141254935087, 0.45320412446525266, 1.6024917635870688, 0.12115692429841468, 1.6024917635870688, -0.12115692429841468, 1.8264141254935087, -0.45320412446525266};  // stand height 0.25, step length 0.3, swing time 0.25
    // const double init_eta[8] = {1.8900999073259275, 0.5043376058303682, 1.6069784307289758, 0.13712110729189467, 1.6069784307289758, -0.13712110729189467, 1.8900999073259275, -0.5043376058303682};  // stand height 0.25, step length 0.3, swing time 0.15
    double velocity     = 0.1;
    double stand_height = 0.25;
    double step_length  = 0.3;
    double step_height  = 0.04;
    double curvature = 0.0;
    int count = 0;
    std::array<int, 4> step_count;
    double max_cal_time = 0.0;

    /* Initial variable */
    rclcpp::Rate rate(sampling_rate);
    WalkGait walk_gait(false, CoM_bias[0], sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA , INIT_BETA , INIT_BETA , INIT_BETA }}};   // init eta (wheel mode)
    

    /* Other variable */
    STATES state = INIT, last_state = INIT;
    double transform_ratio;
    bool trigger;
    int command_count;

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
                step_count = walk_gait.get_step_count();
                if (step_count[0] >= 10 && step_count[1] >= 10 && step_count[2] >= 10 && step_count[3] >= 10) { // all legs have stepped at least twice
                    state = END;
                }//end if  
                break;
            default:
                break;
        }//end switch

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
            motor_cmd_modules[i]->beta = (i == 1 || i == 2)? (eta_list[1][i]) : -(eta_list[1][i]);
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

    ros::shutdown();
    return 0;
}//end main
