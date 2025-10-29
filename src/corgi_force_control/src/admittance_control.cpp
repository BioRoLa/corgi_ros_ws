#include "force_estimation.hpp"

corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
corgi_msgs::msg::MotorCmdStamped motor_cmd;

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped state){
    motor_state = state;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped state){
    force_state = state;
}

void imp_cmd_cb(const corgi_msgs::msg::ImpedanceCmdStamped cmd){
    imp_cmd = cmd;
}

Eigen::MatrixXd admittance_control(const corgi_msgs::msg::ImpedanceCmd* imp_cmd_, const corgi_msgs::msg::MotorState* motor_state_,
                                   const corgi_msgs::msg::ForceState* force_state_, Eigen::MatrixXd& pos_err_hist, Eigen::MatrixXd& force_err_hist){
    
    Eigen::MatrixXd eta_cmd(2, 1);
    eta_cmd << imp_cmd_->theta, imp_cmd_->beta;
    

    Eigen::MatrixXd pos_des(2, 1);
    Eigen::MatrixXd pos_cmd(2, 1);
    Eigen::MatrixXd pos_err(2, 1);
    Eigen::MatrixXd force_err(2, 1);
    int target_rim;

    legmodel.contact_map(imp_cmd_->theta, imp_cmd_->beta);
    pos_des << legmodel.contact_p[0], legmodel.contact_p[1];
    target_rim = legmodel.rim;

    force_err << imp_cmd_->Fx-force_state_->Fx, imp_cmd_->Fy-force_state_->Fy;


    double T = 0.001;

    Eigen::MatrixXd M(2, 2);
    Eigen::MatrixXd B(2, 2);
    Eigen::MatrixXd K(2, 2);

    M << imp_cmd_->Mx, 0, 0, imp_cmd_->My;
    B << imp_cmd_->Bx, 0, 0, imp_cmd_->By;
    K << imp_cmd_->Kx, 0, 0, imp_cmd_->Ky;

    Eigen::MatrixXd a0 =  4*M + 2*B*T + K*T*T;
    Eigen::MatrixXd a1 = -8*M + 2*K*T*T;
    Eigen::MatrixXd a2 =  4*M - 2*B*T + K*T*T;
    Eigen::MatrixXd b0 = Eigen::MatrixXd::Identity(2, 2) * T * T;
    Eigen::MatrixXd b1 = Eigen::MatrixXd::Identity(2, 2) * 2 * T * T;
    Eigen::MatrixXd b2 = Eigen::MatrixXd::Identity(2, 2) * T * T;

    std::cout << "a1: " << a1(0, 0) << ", " << a1(1, 1) << std::endl;
    std::cout << "pos_err_hist_1: " << pos_err_hist(0, 0) << ", " << pos_err_hist(1, 0) << std::endl << std::endl;

    std::cout << "a2: " << a2(0, 0) << ", " << a2(1, 1) << std::endl;
    std::cout << "pos_err_hist_2: " << pos_err_hist(0, 1) << ", " << pos_err_hist(1, 1) << std::endl << std::endl;

    std::cout << "force_cmd: " << imp_cmd_->Fx << ", " << imp_cmd_->Fy << std::endl;
    std::cout << "force_state: " << force_state_->Fx << ", " << force_state_->Fy << std::endl << std::endl;
    std::cout << "force_err: " << force_err(0, 0) << ", " << force_err(1, 0) << std::endl;
    std::cout << "force_err_2: " << force_err_hist(0, 0) << ", " << force_err_hist(1, 0) << std::endl;
    std::cout << "force_err_3: " << force_err_hist(0, 1) << ", " << force_err_hist(1, 1) << std::endl << std::endl;
    
    pos_err = a0.inverse()*(b0*force_err+b1*force_err_hist.col(0)+b2*force_err_hist.col(1)-a1*pos_err_hist.col(0)-a2*pos_err_hist.col(1));
    pos_cmd = pos_des - pos_err;

    if (pos_cmd.array().isNaN().any()) { return eta_cmd; }

    std::cout << "pos_des: " << pos_des(0, 0) << ", " << pos_des(1, 0) << std::endl;
    std::cout << "pos_err: " << pos_err(0, 0) << ", " << pos_err(1, 0) << std::endl;
    std::cout << "pos_cmd: " << pos_cmd(0, 0) << ", " << pos_cmd(1, 0) << std::endl << std::endl;

    legmodel.contact_map(motor_state_->theta, motor_state_->beta);
    pos_err_hist.col(1) = pos_err_hist.col(0);
    pos_err_hist.col(0) << pos_des(0, 0)-pos_cmd(0, 0), pos_des(1, 0)-pos_cmd(1, 0);
    force_err_hist.col(1) = force_err_hist.col(0);
    force_err_hist.col(0) = force_err;

    std::array<double, 2> eta;
    double contact_center[2];

    switch (target_rim)
    {
    case 1:
        contact_center[0] = pos_cmd(0, 0);
        contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
        eta = legmodel.inverse(contact_center, "U_l");
        break;
    case 2:
        contact_center[0] = pos_cmd(0, 0);
        contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
        eta = legmodel.inverse(contact_center, "L_l");
        break;
    case 3:
        contact_center[0] = pos_cmd(0, 0);
        contact_center[1] = pos_cmd(1, 0) + legmodel.r;
        eta = legmodel.inverse(contact_center, "G");
        break;
    case 4:
        contact_center[0] = pos_cmd(0, 0);
        contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
        eta = legmodel.inverse(contact_center, "L_r");
        break;
    case 5:
        contact_center[0] = pos_cmd(0, 0);
        contact_center[1] = pos_cmd(1, 0) + legmodel.radius;
        eta = legmodel.inverse(contact_center, "U_r");
        break;
    default:
        break;
    }

    eta_cmd << eta[0], eta[1];
    std::cout << "target_rim: " << target_rim << std::endl;
    std::cout << "contact_center: " << contact_center[0] << ", " << contact_center[1] << std::endl;
    std::cout << "eta_cmd: " << eta_cmd(0, 0)/M_PI*180 << ", " << eta_cmd(1, 0)/M_PI*180 << std::endl << std::endl;

    return eta_cmd;
}

int main(int argc, char **argv) {

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceControl"), "Admittance Control Starts\n");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("admittance_control");
    auto motor_state_sub = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    auto force_state_sub = nh.subscribe<corgi_msgs::msg::ForceStateStamped>("force/state", 1000, force_state_cb);
    auto imp_cmd_sub = nh.subscribe<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 1000, imp_cmd_cb);
    auto motor_cmd_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    rclcpp::Rate rate(1000);

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

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    std::vector<Eigen::MatrixXd> pos_err_hist_modules = {
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2)
    };

    std::vector<Eigen::MatrixXd> force_err_hist_modules = {
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2),
        Eigen::MatrixXd::Zero(2, 2)
    };

    Eigen::MatrixXd eta_cmd = Eigen::MatrixXd::Zero(2, 1);

    int loop_count = 0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        for (int i=0; i<1; i++){
            eta_cmd = admittance_control(imp_cmd_modules[i], motor_state_modules[i], force_state_modules[i],
                                         pos_err_hist_modules[i], force_err_hist_modules[i]);

            motor_cmd_modules[i]->kp_r = 90;
            motor_cmd_modules[i]->kp_l = 90;
            motor_cmd_modules[i]->kd_r = 1.75;
            motor_cmd_modules[i]->kd_l = 1.75;
            
            motor_cmd_modules[i]->theta = eta_cmd(0, 0);
            motor_cmd_modules[i]->beta = eta_cmd(1, 0);
        }

        std::cout << "= = = = =" << std::endl << std::endl;

        motor_cmd.header.seq = loop_count;
        motor_cmd.header.stamp = rclcpp::Time::now();
        
        motor_cmd_pub.publish(motor_cmd);

        loop_count++;

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}