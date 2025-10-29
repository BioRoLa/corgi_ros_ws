#ifndef FORCE_ESTIMATION_HPP
#define FORCE_ESTIMATION_HPP

#include <iostream>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "leg_model.hpp"
#include "fitted_coefficient.hpp"

#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/force_state_stamped.hpp>
#include <corgi_msgs/msg/impedance_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <corgi_msgs/msg/contact_state_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

bool sim = true;
LegModel legmodel(sim);

Eigen::MatrixXd H_l_coef(2, 8);
Eigen::MatrixXd H_r_coef(2, 8);
Eigen::MatrixXd F_l_coef(2, 8);
Eigen::MatrixXd F_r_coef(2, 8);
Eigen::MatrixXd U_l_coef(2, 8);
Eigen::MatrixXd U_r_coef(2, 8);
Eigen::MatrixXd L_l_coef(2, 8);
Eigen::MatrixXd L_r_coef(2, 8);
Eigen::MatrixXd G_coef(2, 8);

Eigen::MatrixXd calculate_P_poly(int rim, double alpha);
Eigen::MatrixXd calculate_jacobian(Eigen::MatrixXd P_theta, Eigen::MatrixXd P_theta_deriv, double beta);
Eigen::MatrixXd estimate_force(double theta, double beta, double torque_r, double torque_l);
void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw);
#endif