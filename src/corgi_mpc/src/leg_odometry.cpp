#include "mpc.hpp"
#include "force_estimation.hpp"

bool trigger = false;
corgi_msgs::msg::ContactStateStamped contact_state;
std::vector<std::array<double, 2>> eta_modules = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
std::vector<std::array<double, 2>> eta_prev_modules = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped msg){
    for (int i=0; i<4; i++){
        eta_prev_modules[i][0] = eta_modules[i][0];
        eta_prev_modules[i][1] = eta_modules[i][1];
    }

    eta_modules[0][0] = msg.module_a.theta;
    eta_modules[0][1] = msg.module_a.beta;
    eta_modules[1][0] = msg.module_b.theta;
    eta_modules[1][1] = msg.module_b.beta;
    eta_modules[2][0] = msg.module_c.theta;
    eta_modules[2][1] = msg.module_c.beta;
    eta_modules[3][0] = msg.module_d.theta;
    eta_modules[3][1] = msg.module_d.beta;
}

void contact_state_cb(const corgi_msgs::msg::ContactStateStamped msg){
    contact_state = msg;
}

// Copied from leg_model.cpp
std::array<double, 2> objective(const std::array<double, 2>& guessed_q, const std::array<double, 2>& current_q, int contact_rim) {
    using namespace std::complex_literals;
    // std::array<double, 2> guessed_q = {current_q[0] + d_q[0], current_q[1] + d_q[1]};
    
    std::complex<double> current_F_exp, current_G_exp, current_U_exp, current_L_exp, 
                        guessed_F_exp, guessed_G_exp, guessed_U_exp, guessed_L_exp;
    std::array<double, 2> guessed_hip;
    if (contact_rim == 1) {
        // Left upper rim 
        current_F_exp = ( F_l_poly[0](current_q[0])+1i*F_l_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        current_U_exp = ( U_l_poly[0](current_q[0])+1i*U_l_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_F_exp = ( F_l_poly[0](guessed_q[0])+1i*F_l_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_U_exp = ( U_l_poly[0](guessed_q[0])+1i*U_l_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_F_exp - guessed_U_exp) ) - std::arg( -1i/(current_F_exp - current_U_exp) );
        double roll_d = d_alpha * legmodel.radius;
        std::array<double, 2> next_U = {current_U_exp.real() + roll_d, current_U_exp.imag()};
        guessed_hip = {next_U[0] - guessed_U_exp.real(), next_U[1] - guessed_U_exp.imag()}; // next_U - guessed_U
    } else if (contact_rim == 2) {
        // Left lower rim 
        current_G_exp = 1i*G_poly[1](current_q[0]) *std::exp( std::complex<double>(0, current_q[1]) );
        current_L_exp = ( L_l_poly[0](current_q[0])+1i*L_l_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_G_exp = 1i*G_poly[1](guessed_q[0]) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_L_exp = ( L_l_poly[0](guessed_q[0])+1i*L_l_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_G_exp - guessed_L_exp) ) - std::arg( -1i/(current_G_exp - current_L_exp) );
        double roll_d = d_alpha * legmodel.radius;
        std::array<double, 2> next_L = {current_L_exp.real() + roll_d, current_L_exp.imag()};
        guessed_hip = {next_L[0] - guessed_L_exp.real(), next_L[1] - guessed_L_exp.imag()}; // next_L - guessed_L
    } else if (contact_rim == 3) {
        // G
        current_G_exp = 1i*G_poly[1](current_q[0]) *std::exp( std::complex<double>(0, current_q[1]) );
        current_L_exp = ( L_r_poly[0](current_q[0])+1i*L_r_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_G_exp = 1i*G_poly[1](guessed_q[0]) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_L_exp = ( L_r_poly[0](guessed_q[0])+1i*L_r_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_G_exp - guessed_L_exp) ) - std::arg( -1i/(current_G_exp - current_L_exp) );
        double roll_d = d_alpha * legmodel.r;
        std::array<double, 2> next_G = {current_G_exp.real() + roll_d, current_G_exp.imag()};
        guessed_hip = {next_G[0] - guessed_G_exp.real(), next_G[1] - guessed_G_exp.imag()}; // next_G - guessed_G
    } else if (contact_rim == 4) {
        // Right lower rim 
        current_G_exp = 1i*G_poly[1](current_q[0]) *std::exp( std::complex<double>(0, current_q[1]) );
        current_L_exp = ( L_r_poly[0](current_q[0])+1i*L_r_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_G_exp = 1i*G_poly[1](guessed_q[0]) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_L_exp = ( L_r_poly[0](guessed_q[0])+1i*L_r_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_G_exp - guessed_L_exp) ) - std::arg( -1i/(current_G_exp - current_L_exp) );
        double roll_d = d_alpha * legmodel.radius;
        std::array<double, 2> next_L = {current_L_exp.real() + roll_d, current_L_exp.imag()};
        guessed_hip = {next_L[0] - guessed_L_exp.real(), next_L[1] - guessed_L_exp.imag()}; // next_L - guessed_L
    } else if (contact_rim == 5) {
        // Right upper rim 
        current_F_exp = ( F_r_poly[0](current_q[0])+1i*F_r_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        current_U_exp = ( U_r_poly[0](current_q[0])+1i*U_r_poly[1](current_q[0]) ) *std::exp( std::complex<double>(0, current_q[1]) );
        guessed_F_exp = ( F_r_poly[0](guessed_q[0])+1i*F_r_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        guessed_U_exp = ( U_r_poly[0](guessed_q[0])+1i*U_r_poly[1](guessed_q[0]) ) *std::exp( std::complex<double>(0, guessed_q[1]) );
        double d_alpha = std::arg( -1i/(guessed_F_exp - guessed_U_exp) ) - std::arg( -1i/(current_F_exp - current_U_exp) );
        double roll_d = d_alpha * legmodel.radius;
        std::array<double, 2> next_U = {current_U_exp.real() + roll_d, current_U_exp.imag()};
        guessed_hip = {next_U[0] - guessed_U_exp.real(), next_U[1] - guessed_U_exp.imag()}; // next_U - guessed_U
    } else {
        throw std::runtime_error("The leg doesn't contact ground.");
    }//end if else
    
    // Return the result of the objective function
    return {guessed_hip[0], guessed_hip[1]};
}//end objective


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Corgi Leg Odom Starts");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_leg_odom");
    auto motor_state_sub = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("motor/state", 1, motor_state_cb);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1, trigger_cb);
    auto contact_sub = nh.subscribe<corgi_msgs::msg::ContactStateStamped>("odometry/contact", 1, contact_state_cb);
    auto odom_pos_pub = nh.advertise<geometry_msgs::msg::Vector3>("odometry/position", 1);
    auto odom_vel_pub = nh.advertise<geometry_msgs::msg::Vector3>("odometry/velocity", 1);
    
    int freq = 1000;
    rclcpp::Rate rate(freq);

    geometry_msgs::msg::Vector3 odom_pos;
    geometry_msgs::msg::Vector3 odom_vel;

    odom_pos.x = 0.0;
    odom_pos.y = 0.0;
    odom_pos.z = 0.0;

    odom_vel.x = 0.0;
    odom_vel.y = 0.0;
    odom_vel.z = 0.0;

    std::vector<corgi_msgs::msg::ContactState*> contact_state_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
    };

    std::array<double,2> ds_each;
    std::array<double,2> ds_avg;
    int contact_count;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                ds_avg[0] = 0.0;
                ds_avg[1] = 0.0;
                contact_count = 0;
                for (int i=0; i<4; i++) {
                    if (contact_state_modules[i]->contact) {
                        legmodel.contact_map(eta_prev_modules[i][0], eta_prev_modules[i][1]);
                        ds_each = objective(eta_modules[i], eta_prev_modules[i], legmodel.rim);

                        if (i == 0 || i == 3) {
                            ds_each[0] *= -1;
                        }

                        ds_avg[0] += ds_each[0];
                        ds_avg[1] += ds_each[1];
                        contact_count++;
                    }
                }

                if (contact_count > 0) {
                    odom_pos.x += ds_avg[0] / contact_count;
                    odom_pos.z += ds_avg[1] / contact_count;

                    odom_vel.x = ds_avg[0] / contact_count * freq;
                    odom_vel.z = ds_avg[1] / contact_count * freq;
                }

                odom_pos_pub.publish(odom_pos);
                odom_vel_pub.publish(odom_vel);

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}
