#ifndef HYBRID_GEN_HPP
#define HYBRID_GEN_HPP

#include "Simple_fsm.hpp"
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
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "leg_model.hpp"
#include "hybrid_swing.hpp"

class Hybrid{   
    public:
        std::shared_ptr<GaitSelector> gaitSelector;
        Hybrid(std::shared_ptr<GaitSelector> gait_selector_ptr);
        
        ~Hybrid()= default;

        void Initialize(int swing_index, int set_type);
        std::array<double, 2> find_pose(double height, float shift, float steplength, double duty ,double slope);
        void Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg);
        void Swing_step(std::array<double, 2> target, std::array<double, 2> variation, int swing_leg, double duty_ratio);
        void Step();
        void Step_wheel();
        // void change_Height(double new_value);
        void change_Height(double new_value, int leg_index);
        void change_Height_all(double new_value);
        void change_Step_length(double new_value);
        void change_Velocity(double new_value);
        void csv_title(std::ofstream &file);
        void save_to_csv(std::ofstream &file, int step);

        std::array<double, 2> swing_pose;
        std::array<double, 2> swing_variation;
        SwingType swing_type = SwingType::FIVETIMES;
        double terrain_slope = 0 * M_PI / 180.0; // 10 degree
        double swing_desired_height = 0.0; 
        std::vector<SwingPoint> swing_traj[4];  // 每條腿都有自己的swing軌跡
        void update_nextFrame();
        void computeNextBody();
    private:
        double clamp(double value, double min_val, double max_val);
        
        
};


#endif