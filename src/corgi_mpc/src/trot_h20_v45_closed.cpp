#include "force_estimation.hpp"
#include "mpc.hpp"


bool trigger = false;
corgi_msgs::msg::SimDataStamped sim_data;
corgi_msgs::msg::ForceStateStamped force_state;
corgi_msgs::msg::MotorStateStamped motor_state;
geometry_msgs::msg::Vector3 odom_pos;
geometry_msgs::msg::Vector3 odom_vel;
double odom_z;
sensor_msgs::msg::Imu imu;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}

void sim_data_cb(const corgi_msgs::msg::SimDataStamped data){
    sim_data = data;
}

void force_state_cb(const corgi_msgs::msg::ForceStateStamped msg){
    force_state = msg;
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped msg){
    motor_state = msg;
}

void odom_pos_cb(const geometry_msgs::msg::Vector3::ConstSharedPtr &msg){
    odom_pos = *msg;
}

void odom_vel_cb(const geometry_msgs::msg::Vector3::ConstSharedPtr &msg){
    odom_vel = *msg;
}

void odom_z_cb(const std_msgs::msg::Float64::ConstSharedPtr &msg){
    odom_z = msg->data;
}

void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr &msg){
    imu = *msg;
}

void convert_force_to_local(double *f_global, const Eigen::Matrix3d& R_T) {
    Eigen::Vector3d f_global_vec(f_global[0], f_global[1], f_global[2]);
    Eigen::Vector3d f_local = R_T * f_global_vec;
    f_global[0] = f_local(0);
    f_global[1] = f_local(1);
    f_global[2] = f_local(2);
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Corgi MPC Starts");

    ModelPredictiveController mpc;
    mpc.load_config();
    mpc.target_loop = 600;

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_mpc");
    auto imp_cmd_pub = nh.advertise<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 1000);
    auto contact_pub = nh.advertise<corgi_msgs::msg::ContactStateStamped>("odometry/contact", 1000);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    auto sim_data_sub = nh.subscribe<corgi_msgs::msg::SimDataStamped>("sim/data", 1000, sim_data_cb);
    auto force_state_sub = nh.subscribe<corgi_msgs::msg::ForceStateStamped>("force/state", 1000, force_state_cb);
    auto motor_state_sub = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000, motor_state_cb);
    auto odom_pos_sub = nh.subscribe<geometry_msgs::msg::Vector3>("odometry/position", 1000, odom_pos_cb);
    auto odom_vel_sub = nh.subscribe<geometry_msgs::msg::Vector3>("odometry/velocity", 1000, odom_vel_cb);
    auto odom_z_sub = nh.subscribe<std_msgs::msg::Float64>("odometry/z_position_hip", 1000, odom_z_cb);
    auto imu_sub = nh.subscribe<sensor_msgs::msg::Imu>("imu", 1000, imu_cb);

    rclcpp::Rate rate(mpc.freq);

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;
    corgi_msgs::msg::ContactStateStamped contact_state;

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    std::vector<corgi_msgs::msg::ContactState*> contact_state_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
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

    double init_eta[8];

    if (sim) {
        double tmp[8] = {1.2103127664260622,0.1973955598498808,1.1767508136634728,0.04995839572194273,1.1767508136634728,-0.04995839572194273,1.2103127664260622,-0.1973955598498808};
        for (int i = 0; i < 8; ++i) init_eta[i] = tmp[i];
    } else {
        double tmp[8] = {1.1487944075380767,0.20429200223822921,1.113877306709937,0.051749335773739046,1.113877306709937,-0.051749335773739046,1.1487944075380767,-0.20429200223822921};
        for (int i = 0; i < 8; ++i) init_eta[i] = tmp[i];
    }

    mpc.target_pos_z = 0.2;
    
    TrotGait trot_gait(sim, 0, mpc.freq);
    double velocity = 0.45;

    trot_gait.stand_height = mpc.target_pos_z;
    trot_gait.velocity = velocity;
    trot_gait.step_length = 0.15;
    trot_gait.step_height = 0.06;

    trot_gait.initialize(init_eta);
    trot_gait.set_velocity(mpc.target_vel_x);

    bool touched[4] = {true, true, true, true};
    bool selection_matrix[4] = {true, true, true, true};

    // Initialize impedance command
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Fy = -mpc.m*mpc.gravity/4.0;

        cmd->Mx = mpc.Mx;
        cmd->My = mpc.My;
        cmd->Bx = mpc.Bx_swing;
        cmd->By = mpc.By_swing;
        cmd->Kx = mpc.Kx_swing;
        cmd->Ky = mpc.Ky_swing;
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Wait For Force Control Node ...\n");
    
    if (!sim) {
        for (int i=0; i<int(3*mpc.freq); i++) {
            rate.sleep();
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Transform Starts\n");

    for (int i=0; i<int(3*mpc.freq); i++) {
        for (int j=0; j<4; j++) {
            imp_cmd_modules[j]->theta += (init_eta[2*j]-17/180.0*M_PI)/(3*mpc.freq);
            imp_cmd_modules[j]->beta += init_eta[2*j+1]/(3*mpc.freq);
        }
        imp_cmd.header.seq = -1;
        imp_cmd_pub.publish(imp_cmd);
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Transform Finished\n");

    // stay
    for (int i=0; i<int(2*mpc.freq); i++) {
        rclcpp::spin_some(node);
        imp_cmd.header.seq = -1;
        imp_cmd_pub.publish(imp_cmd);
        rate.sleep();
    }
    

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "Wait For Odometry Node Initializing ...\n");

            if (!sim) {
                for (int i=0; i<int(3*mpc.freq); i++) {
                    for (auto& state: contact_state_modules) {
                        state->contact = true;
                    }
                    contact_pub.publish(contact_state);
                    rate.sleep();
                }
            }
            else {
                for (int i=0; i<int(1*mpc.freq); i++) {
                    for (auto& state: contact_state_modules) {
                        state->contact = true;
                    }
                    contact_pub.publish(contact_state);
                    rate.sleep();
                }
            }
            
            for (auto& cmd : imp_cmd_modules){
                cmd->Bx = mpc.Bx_stance;
                cmd->By = mpc.By_stance;
                cmd->Kx = mpc.Kx_stance;
                cmd->Ky = mpc.Ky_stance;
            }

            RCLCPP_INFO(rclcpp::get_logger("CorgiMpc"), "MPC Controller Starts ...\n");

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // update target vel and pos
                // if (loop_count < int(1*mpc.freq)) {
                //     mpc.target_vel_x += velocity/(1*mpc.freq);
                //     trot_gait.set_velocity(mpc.target_vel_x);
                // }
                // else if (loop_count > mpc.target_loop-int(1*mpc.freq) && loop_count < mpc.target_loop) {
                //     mpc.target_vel_x -= velocity/(1*mpc.freq);
                //     trot_gait.set_velocity(mpc.target_vel_x);
                // }

                mpc.target_vel_x = velocity;
                trot_gait.set_velocity(mpc.target_vel_x);

                mpc.target_pos_x += mpc.target_vel_x * mpc.dt;

                // get next eta
                mpc.eta_list = trot_gait.step();

                for (int i=0; i<4; i++) {
                    imp_cmd_modules[i]->theta = mpc.eta_list[0][i];
                    imp_cmd_modules[i]->beta = (i == 1 || i == 2) ? mpc.eta_list[1][i] : -mpc.eta_list[1][i];
                    if (trot_gait.get_swing_phase()[i] == 1 && touched[i]) {
                        selection_matrix[i] = false;
                        touched[i] = false;
                        imp_cmd_modules[i]->By = mpc.By_swing;
                        imp_cmd_modules[i]->Ky = mpc.Ky_swing;
                        
                        // check_contact_state(i, contact_state_modules);
                    }
                    else if (trot_gait.get_swing_phase()[i] == 0 && !touched[i]) {
                        selection_matrix[i] = true;
                        touched[i] = true;
                        imp_cmd_modules[i]->By = mpc.By_stance;
                        imp_cmd_modules[i]->Ky = mpc.Ky_stance;
                    }

                    if (trot_gait.get_duty()[i] <= 0.5 && trot_gait.get_duty()[i] >= 0) {
                        contact_state_modules[i]->contact = true;
                    }
                    else {
                        contact_state_modules[i]->contact = false;
                    }
                }

                // update state
                mpc.robot_vel[0] = odom_vel.x;
                mpc.robot_vel[1] = odom_vel.y;
                mpc.robot_vel[2] = odom_vel.z;
                
                mpc.robot_pos[0] = odom_pos.x;
                mpc.robot_pos[1] = odom_pos.y;
                // mpc.robot_pos[2] = odom_pos.z;
                mpc.robot_pos[2] = odom_z;

                // mpc.robot_vel[0] = (sim_data.position.x-mpc.robot_pos[0])/mpc.dt;
                // mpc.robot_vel[1] = (sim_data.position.y-mpc.robot_pos[1])/mpc.dt;
                // mpc.robot_vel[2] = (sim_data.position.z-mpc.robot_pos[2])/mpc.dt;
                
                // mpc.robot_pos[0] = sim_data.position.x;
                // mpc.robot_pos[1] = sim_data.position.y;
                // mpc.robot_pos[2] = sim_data.position.z;

                mpc.robot_ang.x() = imu.orientation.x;
                mpc.robot_ang.y() = imu.orientation.y;
                mpc.robot_ang.z() = imu.orientation.z;
                mpc.robot_ang.w() = imu.orientation.w;
                
                mpc.robot_ang_vel[0] = imu.angular_velocity.x;
                mpc.robot_ang_vel[1] = imu.angular_velocity.y;
                mpc.robot_ang_vel[2] = imu.angular_velocity.z;

                quaternion_to_euler(mpc.robot_ang, mpc.roll, mpc.pitch, mpc.yaw);
                if (!sim) {
                    mpc.pitch *= -1;
                    mpc.yaw *= -1;
                }

                Eigen::VectorXd x(mpc.n_x);
                x << mpc.roll,             mpc.pitch,            mpc.yaw,
                     mpc.robot_pos[0],     mpc.robot_pos[1],     mpc.robot_pos[2],
                     mpc.robot_ang_vel[0], mpc.robot_ang_vel[1], mpc.robot_ang_vel[2],
                     mpc.robot_vel[0],     mpc.robot_vel[1],     mpc.robot_vel[2],
                     -mpc.gravity;

                Eigen::VectorXd x_ref = Eigen::VectorXd::Zero((mpc.N-1) * mpc.n_x);
                for (int i = 0; i < mpc.N-1; ++i) {
                    x_ref.segment(i * mpc.n_x, mpc.n_x) << 0,                0,                0,
                                                           mpc.target_pos_x, 0, mpc.target_pos_z,
                                                           0,                0,                0,
                                                           mpc.target_vel_x, 0,                0,
                                                           -mpc.gravity;
                }

                // model predictive control
                legmodel.contact_map(motor_state_modules[0]->theta, motor_state_modules[0]->beta);
                double ra[3] = {-legmodel.contact_p[0]+0.222, 0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[1]->theta, motor_state_modules[1]->beta);
                double rb[3] = {legmodel.contact_p[0]+0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[2]->theta, motor_state_modules[2]->beta);
                double rc[3] = {legmodel.contact_p[0]-0.222, -0.2, legmodel.contact_p[1]};
                legmodel.contact_map(motor_state_modules[3]->theta, motor_state_modules[3]->beta);
                double rd[3] = {-legmodel.contact_p[0]-0.222, 0.2, legmodel.contact_p[1]};

                mpc.init_matrices(ra, rb, rc, rd);

                Eigen::VectorXd force = mpc.step(x, x_ref, selection_matrix, force_state_modules);

                double force_A[3] = {force(0), force(1), force(2)};
                double force_B[3] = {force(3), force(4), force(5)};
                double force_C[3] = {force(6), force(7), force(8)};
                double force_D[3] = {force(9), force(10), force(11)};

                Eigen::Matrix3d R_T = mpc.robot_ang.toRotationMatrix().transpose();
                if (!sim) {
                    R_T(1, 2) *= -1;
                    R_T(2, 1) *= -1;
                }
                
                convert_force_to_local(force_A, R_T);
                convert_force_to_local(force_B, R_T);
                convert_force_to_local(force_C, R_T);
                convert_force_to_local(force_D, R_T);


                imp_cmd_modules[0]->Fx = -force_A[0];
                imp_cmd_modules[0]->Fy = -force_A[2];
                imp_cmd_modules[1]->Fx =  force_B[0];
                imp_cmd_modules[1]->Fy = -force_B[2];
                imp_cmd_modules[2]->Fx =  force_C[0];
                imp_cmd_modules[2]->Fy = -force_C[2];
                imp_cmd_modules[3]->Fx = -force_D[0];
                imp_cmd_modules[3]->Fy = -force_D[2];

                imp_cmd.header.seq = loop_count;
                imp_cmd_pub.publish(imp_cmd);

                contact_state.header.seq = loop_count;
                contact_pub.publish(contact_state);

                std::cout << std::fixed << std::setprecision(3);
                std::cout << "Ref Pos = [" << x_ref[3] << ", " << x_ref[4] << ", " << x_ref[5] << "]" << std::endl << std::endl;
                std::cout << "Odom Pos = [" << mpc.robot_pos[0] << ", " << mpc.robot_pos[1] << ", " << mpc.robot_pos[2] << "]" << std::endl;
                std::cout << "Odom Vel = [" << mpc.robot_vel[0] << ", " << mpc.robot_vel[1] << ", " << mpc.robot_vel[2] << "]" << std::endl;
                std::cout << "Odom Ang (deg) = [" << mpc.roll/M_PI*180 << ", " << mpc.pitch/M_PI*180 << ", " << mpc.yaw/M_PI*180 << "]" << std::endl << std::endl;

                std::cout << "Force A: [" << force_A[0] << ", " << force_A[2] << "]" << std::endl;
                std::cout << "State A: [" << force_state_modules[0]->Fx << ", " << force_state_modules[0]->Fy << "]" << std::endl << std::endl;
                std::cout << "Force B: [" << force_B[0] << ", " << force_B[2] << "]" << std::endl;
                std::cout << "State B: [" << force_state_modules[1]->Fx << ", " << force_state_modules[1]->Fy << "]" << std::endl << std::endl;
                std::cout << "Force C: [" << force_C[0] << ", " << force_C[2] << "]" << std::endl;
                std::cout << "State C: [" << force_state_modules[2]->Fx << ", " << force_state_modules[2]->Fy << "]" << std::endl << std::endl;
                std::cout << "Force D: [" << force_D[0] << ", " << force_D[2] << "]" << std::endl;
                std::cout << "State D: [" << force_state_modules[3]->Fx << ", " << force_state_modules[3]->Fy << "]" << std::endl << std::endl;

                std::cout << "= = = = = = = = = =" << std::endl << std::endl;

                loop_count++;
                if (loop_count >= mpc.target_loop) break;

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}