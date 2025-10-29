#include "wlw.hpp"

double deg2rad(double degrees) {
    return degrees * PI / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / PI;
}



int main(int argc, char **argv) { 
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("wlw_pub");
    auto motor_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1);
    corgi_msgs::msg::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::msg::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    rclcpp::Rate rate(1000);

    bool sim = true;
    LegModel leg_model(sim);

    // double new_theta_beta[2] = {deg2rad(50),deg2rad(0)};
    std::array<double, 2> new_theta_beta;
    double pos[2] = {0,-0.159+leg_model.r};
    new_theta_beta = leg_model.inverse(pos, "G");
    leg_model.forward(new_theta_beta[0], new_theta_beta[1],true);
    leg_model.contact_map(new_theta_beta[0], new_theta_beta[1],0);

    // Check and update theta, beta
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->theta = new_theta_beta[0];
        if (i==1 || i==2) {
            motor_cmd_modules[i]->beta  = new_theta_beta[1];
        } else {
            motor_cmd_modules[i]->beta  = -new_theta_beta[1];
        }
        motor_cmd_modules[i]->kp_r = 150;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 150;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for
    for (int i=0; i<1000; i++) {
        motor_pub.publish(motor_cmd);
        rate.sleep();  
    }
    
    std::cout << "move" << std::endl;
    std::array<double, 2> result_eta;
    for (int j = 0; j <100; ++j) {
        // read the motor_cmd_modules[k] just send 
        for (int i=0; i<4; i++){
            new_theta_beta[0] = motor_cmd_modules[i]->theta;
            if (i==1 || i==2) {
                new_theta_beta[1] = motor_cmd_modules[i]->beta;
            } else {
                new_theta_beta[1] = -motor_cmd_modules[i]->beta;
            }
            // if (i==0 || i==1){
                result_eta = leg_model.move(new_theta_beta[0], new_theta_beta[1], {-0.001, 0}, 0.349);
                new_theta_beta[0] = result_eta[0];
                new_theta_beta[1] = result_eta[1];
                leg_model.contact_map(new_theta_beta[0], new_theta_beta[1], 0.349);
            // }
            // else{
            //     result_eta = leg_model.move(new_theta_beta[0], new_theta_beta[1], {-0.001, 0}, 0);
            //     new_theta_beta[0] = result_eta[0];
            //     new_theta_beta[1] = result_eta[1];
            //     leg_model.contact_map(new_theta_beta[0], new_theta_beta[1], 0);
            // }

            motor_cmd_modules[i]->theta = new_theta_beta[0];
            if (i==1 || i==2) {
                motor_cmd_modules[i]->beta  = new_theta_beta[1];
            } else {
                motor_cmd_modules[i]->beta  = -new_theta_beta[1];
            }
            motor_cmd_modules[i]->kp_r = 150;
            motor_cmd_modules[i]->ki_r = 0;
            motor_cmd_modules[i]->kd_r = 1.75;
            motor_cmd_modules[i]->kp_l = 150;
            motor_cmd_modules[i]->ki_l = 0;
            motor_cmd_modules[i]->kd_l = 1.75;
            motor_cmd_modules[i]->torque_r = 0;
            motor_cmd_modules[i]->torque_l = 0;
        }
        motor_pub.publish(motor_cmd);
        rate.sleep();
    }
    std::cout << "end" << std::endl;
    // cout the thata and beta of o and 1
    for(int i =0;i<2;i++){
        new_theta_beta[0] = motor_cmd_modules[i]->theta;
        if (i==1 || i==2) {
            new_theta_beta[1] = motor_cmd_modules[i]->beta;
        } else {
            new_theta_beta[1] = -motor_cmd_modules[i]->beta;
        }
        std::cout << "motor " << i << ": " << new_theta_beta[0] << ", " << new_theta_beta[1] << std::endl;
        leg_model.contact_map(new_theta_beta[0], new_theta_beta[1], 0);
    }
    return 0;

    // store all the data with "theta, beta, x, y, "swing"


    // test 1+ ss posion 
    

    // test with slope

    
}