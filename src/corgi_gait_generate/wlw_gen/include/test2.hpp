// test2.hpp
#ifndef TEST2_HPP
#define TEST2_HPP

#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
// #include "nlopt.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/motor_state.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>

#include "leg_model.hpp"
#include "bezier.hpp"
// #include "wlw.hpp"
#include "walk_gait.hpp"

class WLWGait {
public:
    WLWGait(rclcpp::Node& nh,
            bool sim=true,
            double CoM_bias=0.0,
            int pub_rate=1000,
            double BL=0.444,
            double BW=0.4,
            double BH=0.2);
    ~WLWGait();
    void motorsStateCallback(const corgi_msgs::msg::MotorStateStamped::ConstSharedPtr& msg);
    void setCmd(std::array<double, 2> send, int index, bool dir);
    void publish(int freq);
    std::array<double, 2> find_pose(double height, float shift, float steplength, double slope);
    void Send(int freq);
    void Initialize(int swing_index, int pub_time, int do_pub, int transfer_state, int transfer_sec, int wait_sec, double shift); 
    void Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg);
    void Swing_step(std::array<double, 2> target, std::array<double, 2> variation, double eta[4][2], int swing_leg, double duty_ratio);
    void Step(int pub_time, int do_pub, double shift);
    double closer_beta(double ref_rad, int leg_index);
    void Transform(int type, int do_pub, int transfer_state, int transfer_sec, int wait_sec, double shift);
    std::vector<double> linspace(double start, double end, int num_steps);
    void Transfer(int transfer_sec, int wait_sec, int do_pub);
    void change_Height(double new_value);
    void change_Step_length(double new_value);
    void change_Velocity(double new_value);

    ros::Publisher motor_pub;
    ros::Subscriber motor_state_sub_;
    rclcpp::Rate* rate_ptr;
    corgi_msgs::msg::MotorStateStamped current_motor_state_;
    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &current_motor_state_.module_a,
        &current_motor_state_.module_b,
        &current_motor_state_.module_c,
        &current_motor_state_.module_d
    };
    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::msg::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::array<double, 4> duty;
    std::array<int, 4> swing_phase = {0, 0, 0, 0};
    double incre_duty;
    double velocity     = 0.08; // m/s
    double stand_height = 0.14; //0.14
    double step_length  = 0.4; //0.4
    
    double current_eta[4][2];
    double next_eta[4][2];
    std::array<std::array<double, 2>, 4> foothold;
    std::array<std::array<double, 2>, 4> hip;
    std::array<std::array<double, 2>, 4> next_hip;
    double dS;

private:
    LegModel leg_model;
    const double CoM_bias;
    const double BL;
    const double BW;
    const double BH;
    const double swing_time = 0.2;

    int pub_rate;
    
   
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    
    
    std::array<double, 2> swing_pose;
    std::array<double, 2> swing_pose_temp;
    std::array<double, 2> swing_variation;
    std::array<double, 2> swing_variation_temp;

    std::random_device rd;                     
    std::mt19937 rng;                          
    std::uniform_int_distribution<int> dist;
    
    std::array<int, 4> walk_transform = {0, 0, 0, 0};
    
    double wheel_delta_beta;
    double check_beta[2];
    double delta_beta;
    double delta_theta;
    double body_move_dist;
    int delta_time_step;
    double target_theta;
    bool state;
    bool swing_dir = 0;
    double body_angle ;
    std::array<double, 2> temp;
    std::array<double, 2> pos;
    std::array<double, 4> duty_temp;
    std::array<int, 4> swing_phase_temp = {0, 0, 0, 0};    
    int check_point=0;

    std::array<double, 4> current_step_length = {step_length, step_length, step_length, step_length};
    std::array<double, 4> next_step_length    = {step_length, step_length, step_length, step_length};
    double new_step_length = step_length;
    
};
#endif



