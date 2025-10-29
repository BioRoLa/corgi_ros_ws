#pragma once

/*******  switch *******/ 
// true → simulation, false → real robot
inline constexpr bool SIM = true;
// [Hz]
inline constexpr bool FILTE_VEL = false;        
// use KLD or not (if KLD is not used, need to input the contact state for EKF estimation)
inline constexpr bool KLD = true;               
// publish contact state (only if KLD is used)
inline constexpr bool PUB_CONTACT = true;
// velocity are estimated in body frame, choose the frame that you want to estimate position
inline constexpr bool BODY_FRAME = false; 
inline constexpr bool WORLD_FRAME = true;
inline constexpr bool ESTIMATE_POSITION_FRAME = BODY_FRAME; // BODY_FRAME or WORLD_FRAME
//record data or not
inline constexpr bool RECORD_DATA = false;
// z_position calcution method
enum Method { AVG, MID, MAX, MIN };
inline constexpr Method Z_POS_METHOD = AVG;

/******* odometry *******/

inline constexpr float ODOM_ESTIMATOR_RATE = 500.0; //Hz
inline constexpr float THRESHOLD = 0.08; //threshold of KLD
inline constexpr float ODOM_ESTIMATION_TIME_RANGE = 10.0; // matrix size
inline constexpr float FILTE_VEL_CUT_OFF_FREQ = 10.0; //Hz

/******* std lib *******/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry>
#include <algorithm>
#include <numeric>
#include "rclcpp/rclcpp.hpp"

/******* ROS msg *******/

#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <corgi_msgs/msg/contact_state_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

/******* Leg kinematic *******/ 
#include "leg_model.hpp"
#include "fitted_coefficient.hpp"

/******* filter *******/  
#include "KLD_estimation/csv_reader.hpp"
#include "KLD_estimation/InformationFilter.hpp"

/******* mechanism *******/ 
inline constexpr double MOTOR_OFFSET_X = 0.222;   // [m]
inline constexpr double MOTOR_OFFSET_Y = 0.193;   // [m]
inline constexpr double MOTOR_OFFSET_Z = 0.0;     // [m]

inline constexpr double WHEEL_RADIUS = 0.10;     // [m]
inline constexpr double WHEEL_WIDTH_SIM  = 0.012; // [m]
inline constexpr double WHEEL_WIDTH_REAL = 0.019; // [m]
inline constexpr double WHEEL_WIDTH = SIM ? WHEEL_WIDTH_SIM : WHEEL_WIDTH_REAL;

inline constexpr double GRAVITY        = 9.80665; // [m/s²]

/******* odometry data logging *******/ 
inline constexpr int DATA_SIZE_ORIGIN = 39;
inline constexpr int DATA_SIZE_FILTER = 45;
inline constexpr int ODOM_DATA_SIZE = FILTE_VEL ? DATA_SIZE_FILTER : DATA_SIZE_ORIGIN; 

/******* z_position data logging *******/ 
inline constexpr int Z_POS_DATA_SIZE = 9; 

class Encoder{
    public:
        Encoder(corgi_msgs::msg::MotorState* m, sensor_msgs::msg::Imu* i, bool opposite): module(m), imu(i), opposite(opposite) {}
        void UpdateState(float dt);
        void init(float dt);
        //const reference, prevent modified
        const Eigen::Matrix<float, 5, 1>& GetState() const{return state;}
    private:
        corgi_msgs::msg::MotorState* module;
        sensor_msgs::msg::Imu* imu;
        bool opposite;
        float theta;
        float beta;
        float theta_prev;
        float beta_prev;
        float theta_d;
        float beta_d;
        float w_y;
        Eigen::Vector<float, 5> state;
};