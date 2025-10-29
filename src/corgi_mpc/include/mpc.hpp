#ifndef MPC_HPP
#define MPC_HPP

#include <iostream>
#include <iomanip>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>

#include <corgi_msgs/msg/sim_data_stamped.hpp>
#include <corgi_msgs/msg/impedance_cmd_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <corgi_msgs/msg/force_state_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/contact_state_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include "walk_gait.hpp"
#include "trot_gait.hpp"
#include "hybrid_gen.hpp"

#include <array>
#include <vector>

class ModelPredictiveController {
    public:
        int target_loop = 2000;

        double Mx = 0;
        double My = 0;
        double Bx_swing = 80;
        double By_swing = 80;
        double Bx_stance = 80;
        // double Bx_stance = 50;
        double By_stance = 10;
        double Kx_swing = 2000;
        double Ky_swing = 2000;
        double Kx_stance = 2000;
        double Ky_stance = 200;


        const int freq = 100;
        const double dt = 1.0 / freq;

        const double m = 19.5;
        const double gravity = 9.81;

        const int N = 10;
        const int n_x = 13;
        const int n_u = 12;

        double roll = 0;
        double pitch = 0;
        double yaw = 0;

        double target_pos_x = 0;
        double target_pos_z = 0;
        double target_vel_x = 0;
        double target_vel_z = 0;

        std::array<double, 3> robot_vel = {0, 0, 0};
        std::array<double, 3> robot_pos = {0, 0, 0};
        std::array<double, 3> robot_ang_vel = {0, 0, 0};
        Eigen::Quaterniond robot_ang = Eigen::Quaterniond::Identity();

        std::array<std::array<double, 4>, 2> eta_list = {{{0, 0, 0, 0}, {0, 0, 0, 0}}};

        void load_config();
        void init_matrices(const double *ra, const double *rb, const double *rc, const double *rd);
        Eigen::VectorXd step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref, const bool *selection_matrix, std::vector<corgi_msgs::msg::ForceState*> force_state_modules);

    private:
        Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(n_x, n_x);
        Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(n_x, n_u);
        Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(n_x, n_x);
        Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(n_x, n_u);
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n_x, n_x);
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(n_u, n_u);

        int fx_upper_bound = 10;
        int fx_lower_bound = -10;
        int fz_upper_bound = 200;
        // int fz_lower_bound = -10;
        int fz_lower_bound = -50;

        double friction_coef = 1;

        double l = 0.62;
        double w = 0.33;
        double h = 0.17;
};

void check_contact_state(int swing_leg, std::vector<corgi_msgs::msg::ContactState*> contact_state_modules);

#endif