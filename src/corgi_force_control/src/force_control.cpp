#include "force_estimation.hpp"

corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::MotorCmdStamped motor_cmd;
sensor_msgs::msg::Imu imu;


void imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped cmd){
    imp_cmd = cmd;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped state){
    force_state = state;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped state){
    motor_state = state;
}

void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr &msg){
    imu = *msg;
}

void force_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, Eigen::MatrixXd phi_vel_prev_, corgi_msgs::msg::MotorState* motor_state_, corgi_msgs::msg::ForceState* force_state_, corgi_msgs::msg::MotorCmd* motor_cmd_, double pitch){
    // force command
    Eigen::MatrixXd force_des(2, 1);
    force_des << imp_cmd_->Fx, imp_cmd_->Fy;

    Eigen::MatrixXd force_err(2, 1);
    force_err << force_des(0, 0)-force_state_->Fx, force_des(1, 0)-force_state_->Fy;

    // position command and state
    Eigen::MatrixXd pos_des(2, 1);

    legmodel.contact_map(imp_cmd_->theta, imp_cmd_->beta);

    int target_rim = legmodel.rim;
    int target_alpha = legmodel.alpha;

    pos_des << legmodel.contact_p[0], legmodel.contact_p[1];

    Eigen::MatrixXd pos_fb(2, 1);

    legmodel.forward(motor_state_->theta, motor_state_->beta + pitch);
    if      (target_rim == 1 && (target_alpha > -M_PI*1/2.0)) { pos_fb << legmodel.U_l[0], legmodel.U_l[1] - legmodel.radius; }
    else if (target_rim == 2) { pos_fb << legmodel.L_l[0], legmodel.L_l[1] - legmodel.radius; }
    else if (target_rim == 3) { pos_fb << legmodel.G[0]  , legmodel.G[1]   - legmodel.r;      }
    else if (target_rim == 4) { pos_fb << legmodel.L_r[0], legmodel.L_r[1] - legmodel.radius; }
    else if (target_rim == 5 && (target_alpha < M_PI*1/2.0)) { pos_fb << legmodel.U_r[0], legmodel.U_r[1] - legmodel.radius; }
    else {
        motor_cmd_->theta = imp_cmd_->theta;
        motor_cmd_->beta = imp_cmd_->beta;
        motor_cmd_->kp_r = 50;
        motor_cmd_->kp_l = 50;
        motor_cmd_->kd_r = 1;
        motor_cmd_->kd_l = 1;
        motor_cmd_->torque_r = 0;
        motor_cmd_->torque_l = 0;
        return;
    }

    Eigen::MatrixXd pos_err(2, 1);
    pos_err = pos_des - pos_fb;
    
    // std::cout << "force_des: " << force_des(0, 0) << ", " << force_des(1, 0) << std::endl;
    // std::cout << "force_fb : " << force_state_->Fx << ", " << force_state_->Fy << std::endl;
    // std::cout << "force_err: " << force_err(0, 0) << ", " << force_err(1, 0) << std::endl << std::endl;
    // std::cout << "pos_des: " << pos_des(0, 0) << ", " << pos_des(1, 0) << std::endl;
    // std::cout << "pos_fb : " << pos_fb(0, 0)  << ", " << pos_fb(1, 0)  << std::endl;
    // std::cout << "pos_err: " << pos_err(0, 0) << ", " << pos_err(1, 0) << std::endl << std::endl;

    // calculate jacobian
    Eigen::MatrixXd P_poly = Eigen::MatrixXd::Zero(2, 8);
    Eigen::MatrixXd P_poly_deriv = Eigen::MatrixXd::Zero(2, 7);
    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    legmodel.contact_map(motor_state_->theta, motor_state_->beta + pitch);
    P_poly = calculate_P_poly(legmodel.rim, legmodel.alpha);
    
    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(motor_state_->theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(motor_state_->theta, i); 
    
    Eigen::MatrixXd J_fb(2, 2);
    J_fb = calculate_jacobian(P_theta, P_theta_deriv, motor_state_->beta + pitch);

    // std::cout << "J_fb: " << std::endl
    //           << J_fb(0, 0) << ", " << J_fb(0, 1) << std::endl
    //           << J_fb(1, 0) << ", " << J_fb(1, 1) << std::endl << std::endl;

    Eigen::MatrixXd phi_vel(2, 1);
    phi_vel << motor_state_->velocity_r, motor_state_->velocity_l;

    Eigen::MatrixXd vel_fb = J_fb.transpose() * phi_vel;
    Eigen::MatrixXd acc_fb = (vel_fb - J_fb.transpose() * phi_vel_prev_) * 1000;

    // std::cout << "vel_fb : " << vel_fb(0, 0)  << ", " << vel_fb(1, 0)  << std::endl;
    // std::cout << "acc_fb : " << acc_fb(0, 0)  << ", " << acc_fb(1, 0)  << std::endl << std::endl;

    // impedance control
    Eigen::MatrixXd M(2, 2);
    Eigen::MatrixXd B(2, 2);
    Eigen::MatrixXd K(2, 2);
    Eigen::MatrixXd eta_cmd(2, 1);
    Eigen::MatrixXd trq_cmd(2, 1);
    Eigen::MatrixXd kp_cmd(2, 1);
    Eigen::MatrixXd kd_cmd(2, 1);

    M << imp_cmd_->Mx, 0, 0, imp_cmd_->My;
    B << imp_cmd_->Bx, 0, 0, imp_cmd_->By;
    K << imp_cmd_->Kx, 0, 0, imp_cmd_->Ky;

    eta_cmd << imp_cmd_->theta, imp_cmd_->beta;
    trq_cmd = J_fb.transpose() * (force_des + M*(-acc_fb));
    // std::cout << "trq_cmd : " << trq_cmd(0, 0) << ", " << trq_cmd(1, 0) << std::endl << std::endl;

    // calculate phi command
    Eigen::MatrixXd phi_des(2, 1);
    phi_des << eta_cmd(1, 0) - eta_cmd(0, 0) + 17/180.0*M_PI,
               eta_cmd(1, 0) + eta_cmd(0, 0) - 17/180.0*M_PI;

    Eigen::MatrixXd phi_fb(2, 1);
    phi_fb << motor_state_->beta + pitch - motor_state_->theta + 17/180.0*M_PI,
              motor_state_->beta + pitch + motor_state_->theta - 17/180.0*M_PI;

    Eigen::MatrixXd phi_err = phi_des-phi_fb;

    // kp compensate
    kp_cmd << J_fb(0, 0) * J_fb(0, 0) * K(0, 0) + J_fb(1, 0) * J_fb(1, 0) * K(1, 1),
              J_fb(0, 1) * J_fb(0, 1) * K(0, 0) + J_fb(1, 1) * J_fb(1, 1) * K(1, 1);

    trq_cmd(0, 0) += (J_fb(0, 0) * J_fb(0, 1) * K(0, 0) + J_fb(1, 0) * J_fb(1, 1) * K(1, 1)) * phi_err(1, 0);
    trq_cmd(1, 0) += (J_fb(0, 0) * J_fb(0, 1) * K(0, 0) + J_fb(1, 0) * J_fb(1, 1) * K(1, 1)) * phi_err(0, 0);

    // kd compensate
    kd_cmd << J_fb(0, 0) * J_fb(0, 0) * B(0, 0) + J_fb(1, 0) * J_fb(1, 0) * B(1, 1),
              J_fb(0, 1) * J_fb(0, 1) * B(0, 0) + J_fb(1, 1) * J_fb(1, 1) * B(1, 1);

    trq_cmd(0, 0) += (J_fb(0, 0) * J_fb(0, 1) * B(0, 0) + J_fb(1, 0) * J_fb(1, 1) * B(1, 1)) * (-phi_vel(1, 0));
    trq_cmd(1, 0) += (J_fb(0, 0) * J_fb(0, 1) * B(0, 0) + J_fb(1, 0) * J_fb(1, 1) * B(1, 1)) * (-phi_vel(0, 0));
    
    // torque version impedance control
    // trq_cmd << J_fb.transpose() * (force_des + M*(-acc_fb) + B*(-vel_fb) + K*pos_err);
    // kp_cmd << 0, 0;
    // kd_cmd << 0, 0;

    // std::cout << (force_des)(0, 0) << ", " << force_des(1, 0) << std::endl;
    // std::cout << (K*pos_err)(0, 0) << ", " << (K*pos_err)(1, 0) << std::endl;
    // std::cout << (B*(-vel_fb))(0, 0) << ", " << (B*(-vel_fb))(1, 0) << std::endl;
    // std::cout << (M*(-acc_fb))(0, 0) << ", " << (M*(-acc_fb))(1, 0) << std::endl;


    // std::cout << "kp_cmd  : " << kp_cmd(0, 0) << ", " << kp_cmd(1, 0) << std::endl;
    // std::cout << "phi_err : " << phi_err(0, 0) << ", " << phi_err(1, 0) << std::endl << std::endl;

    // std::cout << "kd_cmd  : " << kd_cmd(0, 0) << ", " << kd_cmd(1, 0) << std::endl;
    // std::cout << "phi_vel : " << phi_vel(0, 0) << ", " << phi_vel(1, 0) << std::endl << std::endl;

    // std::cout << "eta_cmd: " << eta_cmd(0, 0) << ", " << eta_cmd(1, 0) << std::endl;
    // std::cout << "trq_kp  : " << kp_cmd(0, 0)*phi_err(0, 0) << ", " << kp_cmd(1, 0)*phi_err(1, 0) << std::endl;
    // std::cout << "trq_kd  : " << -kd_cmd(0, 0)*phi_vel(0, 0) << ", " << -kd_cmd(1, 0)*phi_vel(1, 0) << std::endl;
    // std::cout << "trq_cmd: " << trq_cmd(0, 0) << ", " << trq_cmd(1, 0) << std::endl;

    // send to motor command
    motor_cmd_->theta = eta_cmd(0, 0);
    motor_cmd_->beta = eta_cmd(1, 0);
    motor_cmd_->kp_r = kp_cmd(0, 0); //std::min(200.0, std::max(10.0, kp_cmd(0, 0)));
    motor_cmd_->kp_l = kp_cmd(1, 0); //std::min(200.0, std::max(10.0, kp_cmd(1, 0)));
    motor_cmd_->kd_r = kd_cmd(0, 0); //std::min(5.0,   std::max(0.05, kd_cmd(0, 0)));
    motor_cmd_->kd_l = kd_cmd(1, 0); //std::min(5.0,   std::max(0.05, kd_cmd(1, 0)));
    motor_cmd_->torque_r = trq_cmd(0, 0);
    motor_cmd_->torque_l = trq_cmd(1, 0);
}

void position_control(corgi_msgs::msg::ImpedanceCmd* imp_cmd_, corgi_msgs::msg::MotorCmd* motor_cmd_) {
    motor_cmd_->theta = imp_cmd_->theta;
    motor_cmd_->beta = imp_cmd_->beta;
    motor_cmd_->kp_r = 50;
    motor_cmd_->kp_l = 50;
    motor_cmd_->kd_r = 1;
    motor_cmd_->kd_l = 1;
    motor_cmd_->torque_r = 0;
    motor_cmd_->torque_l = 0;
}


int main(int argc, char **argv) {

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceControl"), "Force Control Starts\n");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("force_control");
    auto imp_cmd_sub = nh.subscribe<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 1000, imp_cmd_cb);
    auto force_state_sub = nh.subscribe<corgi_msgs::msg::ForceStateStamped>("force/state", 1000, force_state_cb);
    auto motor_state_sub = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    auto imu_sub = nh.subscribe<sensor_msgs::msg::Imu>("imu", 1000, imu_cb);
    auto motor_cmd_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    rclcpp::Rate rate(1000);

    Eigen::Quaterniond body_angle_quat;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<Eigen::MatrixXd> phi_vel_prev_modules = {
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1)
    };

    std::vector<Eigen::MatrixXd> phi_prev_modules = {
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1),
        Eigen::MatrixXd::Zero(2, 1)
    };

    double phi_r = 0;
    double phi_l = 0;

    // AR, AL, BR, BL, CR, CL, DR, DL
    // std::vector<double> kt = {2.018, 2.126, 2.141, 2.176, 1.927, 2.072, 2.098, 2.143};
    std::vector<double> friction = {0.625, 0.44, 0.662, 0.499, 0.623, 0.409, 0.677, 0.356};  // already include kt

    int loop_count = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // body_angle_quat = {imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z};
        // quaternion_to_euler(body_angle_quat, roll, pitch, yaw);

        for (int i=0; i<4; i++){
            if (imp_cmd_modules[i]->Kx == 0 && imp_cmd_modules[i]->Ky == 0 && imp_cmd_modules[i]->Bx == 0 && imp_cmd_modules[i]->By == 0) {
                position_control(imp_cmd_modules[i], motor_cmd_modules[i]);
            }
            else {
                if (imp_cmd_modules[i]->theta < 17/180.0*M_PI) { imp_cmd_modules[i]->theta = 17/180.0*M_PI; }

                if (i == 1 || i == 2) {
                    force_control(imp_cmd_modules[i], phi_vel_prev_modules[i], motor_state_modules[i], force_state_modules[i], motor_cmd_modules[i], -pitch);
                }
                else {
                    force_control(imp_cmd_modules[i], phi_vel_prev_modules[i], motor_state_modules[i], force_state_modules[i], motor_cmd_modules[i], pitch);
                }
                
            }
            phi_vel_prev_modules[i] << motor_state_modules[i]->velocity_r, motor_state_modules[i]->velocity_l;
        }

        // dynamic friction compensation
        if (!sim){
            for (int i=0; i<4; i++) {
                phi_r = motor_state_modules[i]->theta + motor_state_modules[i]->beta - 17/180.0*M_PI;
                phi_l = motor_state_modules[i]->beta - motor_state_modules[i]->theta + 17/180.0*M_PI;

                if (phi_r > phi_prev_modules[i](0, 0)){
                    motor_cmd_modules[i]->torque_r -= friction[2*i];
                }
                else {
                    motor_cmd_modules[i]->torque_r += friction[2*i];
                }

                if (phi_l > phi_prev_modules[i](1, 0)){
                    motor_cmd_modules[i]->torque_l -= friction[2*i+1];
                }
                else {
                    motor_cmd_modules[i]->torque_l += friction[2*i+1];
                }

                phi_prev_modules[i] << phi_r, phi_l;
            }
        }

        // std::cout << "= = = = = = = = = = =" << std::endl << std::endl;

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = rclcpp::Time::now();
        
        motor_cmd_pub.publish(motor_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}