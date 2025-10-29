#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/impedance_cmd_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "leg_model.hpp"
#include "force_estimation.hpp"

bool trigger = false;
sensor_msgs::msg::Imu imu;
corgi_msgs::msg::MotorStateStamped motor_state;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}

void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr &msg){
    imu = *msg;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped msg){
    motor_state = msg;
}

Eigen::Vector4d distribute_forces(double sa, double sb, double sc, double sd, double mg, double L, double t)
{
    double Fa = t;
    double Fd = mg / 2.0 - t;
    
    double denominator = L + sb - sc;
    
    double Fc = (t * (L - sa - sd) + (mg / 2.0) * (sb + sd)) / denominator;
    
    double Fb = mg / 2.0 - Fc;
    
    return Eigen::Vector4d(Fa, Fb, Fc, Fd);
}

Eigen::Vector4d distribute_forces_(double sa, double sd, double mg, double L, double t)
{
    double Fa = t;
    double Fd = mg / 2.0 - t;
    double Fb = (mg / 2.0) * (L - sa - sd) / L - Fa;
    double Fc = mg / 2.0 - Fb;
    
    return Eigen::Vector4d(Fa, Fb, Fc, Fd);
}


int main(int argc, char **argv) {

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceControl"), "Simulation Stay Experiment Starts\n");
    
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("imp_sim_stay");
    auto imp_cmd_pub = nh.advertise<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 1000);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    auto motor_state_sub = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    auto imu_sub = nh.subscribe<sensor_msgs::msg::Imu>("imu", 1000, imu_cb);
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    int exp_case = 2;  // G=0, L=1, U=2

    double mg = -19.5*9.81;
    double F_init = mg/4.0;

    bool sim = true;
    LegModel legmodel(sim);

    Eigen::Vector4d forces;
    double sa = 0;
    double sb = 0;
    double sc = 0;
    double sd = 0;

    Eigen::Quaterniond robot_ang = Eigen::Quaterniond::Identity();
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    // robot weight ~= 220 N
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Mx = 0;
        cmd->My = 0;
        cmd->Bx = 100;
        cmd->By = 100;
        cmd->Kx = 2000;
        cmd->Ky = 2000;
    }

    for (int i=0; i<2000; i++){
        rclcpp::spin_some(node);
        if (exp_case == 0) {
            imp_cmd_modules[0]->theta += 63/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->theta += 63/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->theta += 63/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->theta += 63/2000.0/180.0*M_PI;
        }
        if (exp_case == 1) {
            imp_cmd_modules[0]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->theta += 23/2000.0/180.0*M_PI;
            imp_cmd_modules[0]->beta += 25/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->beta -= 25/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->beta -= 25/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->beta += 25/2000.0/180.0*M_PI;
        }
        if (exp_case == 2) {
            imp_cmd_modules[0]->theta += 43/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->theta += 43/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->theta += 43/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->theta += 43/2000.0/180.0*M_PI;
            imp_cmd_modules[0]->beta += 80/2000.0/180.0*M_PI;
            imp_cmd_modules[1]->beta -= 80/2000.0/180.0*M_PI;
            imp_cmd_modules[2]->beta -= 80/2000.0/180.0*M_PI;
            imp_cmd_modules[3]->beta += 80/2000.0/180.0*M_PI;
        }

        legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta);
        sa = legmodel.contact_p[0];

        legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta);
        sb = legmodel.contact_p[0];

        legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta);
        sc = legmodel.contact_p[0];

        legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta);
        sd = legmodel.contact_p[0];

        // forces = distribute_forces(sa, sb, sc, sd, mg, 0.444, F_init);
        forces = distribute_forces_(-sa/2.0, -sd/2.0, mg, 0.222, F_init);
        
        std::cout << forces[0] << ", " << forces[1] << ", " << forces[2] << ", " << forces[3] << std::endl << std::endl;

        for (int i=0; i<4; i++) {
            imp_cmd_modules[i]->Fy = forces[i];
        }

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        if (trigger){
            int loop_count = 0;
            while (rclcpp::ok()) {
                // Stay
                if (loop_count < 2000) {
                }
                else if (loop_count < 3000) {
                    F_init -= 0.03;
                }
                else if (loop_count < 4000) {
                }
                else if (loop_count < 5000) {
                    F_init += 0.03;
                }
                else if (loop_count < 6000) {
                }
                else if (loop_count < 7000) {
                    F_init -= 0.02;
                }
                else if (loop_count < 8000) {
                }
                else if (loop_count < 9000) {
                    F_init += 0.02;
                }
                else if (loop_count < 10000) {
                }
                else if (loop_count < 11000) {
                    F_init -= 0.01;
                }
                else if (loop_count < 12000) {
                }
                else if (loop_count < 13000) {
                    F_init += 0.01;
                }
                else if (loop_count < 14000) {
                }
                else {
                    break;
                }

                robot_ang.x() = imu.orientation.x;
                robot_ang.y() = imu.orientation.y;
                robot_ang.z() = imu.orientation.z;
                robot_ang.w() = imu.orientation.w;

                quaternion_to_euler(robot_ang, roll, pitch, yaw);

                legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta+pitch);
                sa = legmodel.contact_p[0];
        
                legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta-pitch);
                sb = legmodel.contact_p[0];
        
                legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta-pitch);
                sc = legmodel.contact_p[0];
        
                legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta+pitch);
                sd = legmodel.contact_p[0];
        
                std::cout << sa << ", " << sb << ", " << sc << ", " << sd << std::endl << std::endl;

                // forces = distribute_forces(sa, sb, sc, sd, mg, 0.444, F_init);
                forces = distribute_forces_(-sa/2.0, -sd/2.0, mg, 0.222, F_init);

                std::cout << forces[0] << ", " << forces[1] << ", " << forces[2] << ", " << forces[3] << std::endl << std::endl;
        
                for (int i=0; i<4; i++) {
                    imp_cmd_modules[i]->Fy = forces[i];
                }

                if (loop_count > 2000 && loop_count < 13000) {
                    imp_cmd_modules[0]->Fx = 30 * sin(loop_count / 1000.0 * M_PI);
                    imp_cmd_modules[1]->Fx = 30 * sin(loop_count / 1000.0 * M_PI);
                    imp_cmd_modules[2]->Fx = -30 * sin(loop_count / 1000.0 * M_PI);
                    imp_cmd_modules[3]->Fx = -30 * sin(loop_count / 1000.0 * M_PI);
                }


                imp_cmd.header.seq = loop_count;

                imp_cmd_pub.publish(imp_cmd);

                loop_count++;

                rate.sleep();
            }

            ros::shutdown();
            
            return 0;
        }

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}