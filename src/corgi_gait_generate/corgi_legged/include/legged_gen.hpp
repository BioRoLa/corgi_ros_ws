#ifndef LEGGED_GEN_HPP
#define LEGGED_GEN_HPP

#include "Simple_fsm.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>
#include <array>
#include <string>

#include "leg_model.hpp"
#include "bezier.hpp"
#include "rclcpp/rclcpp.hpp"

class Legged{
    public:
    std::shared_ptr<GaitSelector> gaitSelector;
        Legged(std::shared_ptr<GaitSelector> gait_selector_ptr);
        ~Legged()= default;

        void Initial();
        void next_Step();
        void set_velocity(double new_value);
        void set_stand_height(double new_value);
        void set_step_length(double new_value);
        void set_step_height(double new_value);
        void set_curvature(double new_value);
        

    private:
        std::array<int, 4> step_count  = {0, 0, 0, 0};
        
        bool touchdown;

        std::array<double, 4> theta;
        std::array<double, 4> beta;

        // Intermediate variables
        int current_rim;
        std::string touch_rim_list[5] = {"G", "L_l", "L_r", "U_l", "U_r"};
        int touch_rim_idx[5] = {3, 2, 4, 1, 5};
        double swing_phase_ratio;
        std::array<double, 2> curve_point_temp;
        std::array<double, 2> result_eta;
        std::array<double, 2> p_lo;
        std::array<double, 2> p_td;
        std::array<SwingProfile, 4> sp;

        // // For turning 
        // double outer_radius;
        // double inner_radius;
        // double diff_step_length = 0.0;  // Differential step length 
        // double new_diff_step_length = 0.0;  // New differential step length
        // double diff_dS = 0.0;   // Differential dS
        // int sign_diff[4];   // Differential sign
};

#endif 