#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>

#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include "walk_gait.hpp"
#include "leg_model.hpp"
#include "bezier.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("walk_test");
    auto motor_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1);
    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::msg::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 

    double CoM_bias = 0.0;
    int sampling_rate = 1000;
    rclcpp::Rate rate(sampling_rate);
    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    WalkGait walk_gait(true, CoM_bias, sampling_rate);
    walk_gait.initialize(init_eta);
    std::array<std::array<double, 4>, 2> eta_list;
    double velocity     = 0.1;
    double stand_height = 0.2;
    double step_length  = 0.3;
    double step_height  = 0.05;
    double curvature = 0.0;
    int count = 0;

    auto start = std::chrono::high_resolution_clock::now();
    while (rclcpp::ok()) {
    // for (int count=0; count<200000; ){
        count ++;
        velocity = 0.2*cos(count/1711.0);
        walk_gait.set_velocity(velocity);
        stand_height = 0.25 + 0.05*cos(count/1211.0);
        walk_gait.set_stand_height(stand_height);
        step_length = (count++/3311)%2 == 0? 0.3 : 0.1;
        walk_gait.set_step_length(step_length);
        step_height = (count++/2311)%2 == 0? 0.08 : 0.04;
        walk_gait.set_step_height(step_height);
        curvature = 1.0*sin(count/3911.0);
        walk_gait.set_curvature(curvature);
        eta_list = walk_gait.step();
        // Publish motor commands
        for (int i=0; i<4; i++) {
            if (eta_list[0][i] > M_PI*160.0/180.0) {
                std::cout << "Exceed upper bound." << std::endl;
            }//end if 
            if (eta_list[0][i] < M_PI*17.0/180.0) {
                std::cout << "Exceed lower bound." << std::endl;
            }//end if 
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
        }
        motor_pub.publish(motor_cmd);
        rate.sleep();
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    
    ros::shutdown();
    return 0;
}//end main
