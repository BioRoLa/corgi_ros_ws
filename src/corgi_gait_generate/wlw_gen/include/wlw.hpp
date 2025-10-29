#ifndef WLW_HPP
#define WLW_HPP

#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <iomanip>
// #include "nlopt.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"
#include "bezier.hpp"


using namespace std;
using namespace Eigen;
const double PI = M_PI;
double walking_distance, step_length, velocity, tilt_angle_deg, tilt_angle_rad;
vector<pair<double, double>> terrain_positions;

double deg2rad(double degrees);
double rad2deg(double radians);
void genTerrainPosition(double walking_distance, double tilt_angle_rad, vector<pair<double, double>> &terrain_positions);
void setting(double &walking_distance, double &step_length, double &velocity, double &tilt_angle_deg, double &tilt_angle_rad);
void setting_calculation(double walking_distance, double step_length, double tilt_angle_rad, const vector<pair<double, double>> &terrain_positions, bool output_details);
void findMinMaxHeight(const vector<pair<double, double>> &terrain_positions);

struct InputData {
    double Height = 0.127; // m
    double Distance = 1.5; // m
    double Velocity = 0.2; // m/sec
    double MaxTDlength;
    double total_time;
};


struct Gendata {
    double h_desired;
    double S_max;
    vector<Vector2d> O_list; // List of desired trajectories
    vector<Vector2d> q_traj; // Actual trajectory
    vector<Vector2d> O_traj; // O trajectory
    vector<Vector2d> GT_traj; // Ground truth trajectory
    vector<int> Mode_traj; // Mode trajectory
    vector<double> fval_traj; // fval trajectory
};

#endif
