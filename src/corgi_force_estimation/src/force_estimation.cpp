#include "force_estimation.hpp"


corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::ContactStateStamped contact_state;
sensor_msgs::msg::Imu imu;


void motor_state_cb(const corgi_msgs::msg::MotorStateStamped state){
    motor_state = state;
}

void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr &msg){
    imu = *msg;
}

void contact_state_cb(const corgi_msgs::msg::ContactStateStamped state) {
    contact_state = state;
}

Eigen::MatrixXd calculate_P_poly(int rim, double alpha){
    for (int i=0; i<8; i++){
        H_l_coef(0, i) = H_x_coef[i];
        H_l_coef(1, i) = H_y_coef[i];

        H_r_coef(0, i) = -H_x_coef[i];
        H_r_coef(1, i) = H_y_coef[i];

        U_l_coef(0, i) = U_x_coef[i];
        U_l_coef(1, i) = U_y_coef[i];

        U_r_coef(0, i) = -U_x_coef[i];
        U_r_coef(1, i) = U_y_coef[i];
        
        L_l_coef(0, i) = L_x_coef[i];
        L_l_coef(1, i) = L_y_coef[i];
        
        L_r_coef(0, i) = -L_x_coef[i];
        L_r_coef(1, i) = L_y_coef[i];
        
        F_l_coef(0, i) = F_x_coef[i];
        F_l_coef(1, i) = F_y_coef[i];
        
        F_r_coef(0, i) = -F_x_coef[i];
        F_r_coef(1, i) = F_y_coef[i];
        
        G_coef(0, i) = 0;
        G_coef(1, i) = G_y_coef[i];
    }

    Eigen::MatrixXd P_poly(2, 8);

    double scaled_radius = legmodel.radius / legmodel.R;

    if (rim == 1 && alpha > -M_PI*2.0/3.0) {
    // if (rim == 1) {
        Eigen::Rotation2D<double> rotation(alpha+M_PI);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (H_l_coef-U_l_coef) * scaled_radius + U_l_coef;
    }
    else if (rim == 2) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef-L_l_coef) * scaled_radius + L_l_coef;
    }
    else if (rim == 3) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef-L_l_coef) * legmodel.r / legmodel.R + G_coef;
    }
    else if (rim == 4) {
        Eigen::Rotation2D<double> rotation(alpha);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (G_coef-L_r_coef) * scaled_radius + L_r_coef;
    }
    else if (rim == 5 && alpha < M_PI*2.0/3.0) {
    // else if (rim == 5) {
        Eigen::Rotation2D<double> rotation(alpha-M_PI);
        Eigen::Matrix2d rot_alpha = rotation.toRotationMatrix();
        P_poly = rot_alpha * (H_r_coef-U_r_coef) * scaled_radius + U_r_coef;
    }
    else {
        P_poly = Eigen::MatrixXd::Zero(2, 8);
    }
    
    return P_poly;
}

Eigen::MatrixXd calculate_jacobian(Eigen::MatrixXd P_theta, Eigen::MatrixXd P_theta_deriv, double beta){
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);

    double dtheta_dphiR = -0.5;
    double dtheta_dphiL =  0.5;
    double dbeta_dphiR  =  0.5;
    double dbeta_dphiL  =  0.5;

    double dPx_dtheta = P_theta_deriv(0, 0)*cos_beta - P_theta_deriv(1, 0)*sin_beta;
    double dPy_dtheta = P_theta_deriv(0, 0)*sin_beta + P_theta_deriv(1, 0)*cos_beta;
    double dPx_dbeta  = P_theta(0, 0)*(-sin_beta) - P_theta(1, 0)*cos_beta;
    double dPy_dbeta  = P_theta(0, 0)*cos_beta + P_theta(1, 0)*(-sin_beta);

    double J11 = dPx_dtheta * dtheta_dphiR + dPx_dbeta * dbeta_dphiR;
    double J12 = dPx_dtheta * dtheta_dphiL + dPx_dbeta * dbeta_dphiL;
    double J21 = dPy_dtheta * dtheta_dphiR + dPy_dbeta * dbeta_dphiR;
    double J22 = dPy_dtheta * dtheta_dphiL + dPy_dbeta * dbeta_dphiL;

    Eigen::MatrixXd jacobian(2, 2);
    jacobian << J11, J12, J21, J22;

    return jacobian;
}

Eigen::MatrixXd estimate_force(double theta, double beta, double torque_r, double torque_l){
    legmodel.contact_map(theta, beta);

    Eigen::MatrixXd P_poly = calculate_P_poly(legmodel.rim, legmodel.alpha);
    Eigen::MatrixXd P_poly_deriv(2, 7);

    for (int i=0; i<7; i++) P_poly_deriv.col(i) = P_poly.col(i+1)*(i+1);

    Eigen::MatrixXd P_theta = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P_theta_deriv = Eigen::MatrixXd::Zero(2, 1);

    for (int i=0; i<8; i++) P_theta += P_poly.col(i) * pow(theta, i); 
    for (int i=0; i<7; i++) P_theta_deriv += P_poly_deriv.col(i) * pow(theta, i); 
    
    Eigen::MatrixXd jacobian(2, 2);
    jacobian = calculate_jacobian(P_theta, P_theta_deriv, beta);
    
    Eigen::MatrixXd force_est(2, 1);

    if (jacobian.isZero(1e-6)) {
        force_est.setZero();
    } else {
        Eigen::MatrixXd torque(2, 1);
        torque << torque_r, torque_l;
        force_est = jacobian.inverse().transpose() * torque;
    }

    return force_est;
}

void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    Eigen::Quaterniond q_norm = q.normalized();

    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}


int main(int argc, char **argv) {

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Force Estimation Starts\n");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("force_estimation");
    auto motor_state_sub = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    auto imu_sub = nh.subscribe<sensor_msgs::msg::Imu>("imu", 1000, imu_cb);
    auto contact_sub = nh.subscribe<corgi_msgs::msg::ContactStateStamped>("odometry/contact", 1000, contact_state_cb);
    auto force_state_pub = nh.advertise<corgi_msgs::msg::ForceStateStamped>("force/state", 1000);
    rclcpp::Rate rate(1000);

    Eigen::Quaterniond body_angle_quat;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::msg::ForceState*> force_state_modules = {
        &force_state.module_a,
        &force_state.module_b,
        &force_state.module_c,
        &force_state.module_d
    };

    std::vector<corgi_msgs::msg::ContactState*> contact_state_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
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

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        body_angle_quat = {imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z};
        quaternion_to_euler(body_angle_quat, roll, pitch, yaw);

        if (!sim){
            pitch *= -1;
            yaw *= -1;

            // dynamic friction compensation
            for (int i=0; i<4; i++) {
                phi_r = motor_state_modules[i]->theta + motor_state_modules[i]->beta - 17/180.0*M_PI;
                phi_l = motor_state_modules[i]->beta - motor_state_modules[i]->theta + 17/180.0*M_PI;

                if (phi_r > phi_prev_modules[i](0, 0)){
                    motor_state_modules[i]->torque_r += friction[2*i];
                }
                else {
                    motor_state_modules[i]->torque_r -= friction[2*i];
                }

                if (phi_l > phi_prev_modules[i](1, 0)){
                    motor_state_modules[i]->torque_l += friction[2*i+1];
                }
                else {
                    motor_state_modules[i]->torque_l -= friction[2*i+1];
                }

                phi_prev_modules[i] << phi_r, phi_l;
            }
        }

        for (int i=0; i<4; i++){
            Eigen::MatrixXd force_est;

            if (i == 1 || i == 2) {
                force_est = estimate_force(motor_state_modules[i]->theta, motor_state_modules[i]->beta-pitch,
                                           motor_state_modules[i]->torque_r, motor_state_modules[i]->torque_l);
            }
            else {
                force_est = estimate_force(motor_state_modules[i]->theta, motor_state_modules[i]->beta+pitch,
                                           motor_state_modules[i]->torque_r, motor_state_modules[i]->torque_l);
            }

            if (i == 1 || i == 2) { force_state_modules[i]->Fx = -force_est(0, 0); }
            else { force_state_modules[i]->Fx = force_est(0, 0); }
            
            force_state_modules[i]->Fy = -force_est(1, 0)+0.68*9.81;
        }

        force_state.header.seq = motor_state.header.seq;
        force_state.header.stamp = rclcpp::Time::now();
        
        force_state_pub.publish(force_state);

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}