#include "force_estimation.hpp"
#include "trot_gait.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Corgi Trot Starts");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_trot");
    auto motor_cmd_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1000);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::MotorCmdStamped motor_cmd;

    std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };

    // initialize gait
    // sim, h20 sl0.2
    // double init_eta[8] = {1.2376248201301099,0.26060239174734107,1.2376248201301099,0.26060239174734107,1.2376248201301099,-0.26060239174734107,1.2376248201301099,-0.26060239174734107};
    
    // sim, h20 sl0.15
    // double init_eta[8] = {1.2103127664260622,0.1973955598498808,1.2103127664260622,0.1973955598498808,1.2103127664260622,-0.1973955598498808,1.2103127664260622,-0.1973955598498808};
    
    // sim, h20 sl0.1
    // double init_eta[8] = {1.190510669055021,0.132551532296674,1.190510669055021,0.132551532296674,1.190510669055021,-0.132551532296674,1.190510669055021,-0.132551532296674};
    
    // sim, h25 sl0.2
    double init_eta[8] = {1.6951516460296583,0.20749622643520274,1.6951516460296583,0.20749622643520274,1.6951516460296583,-0.20749622643520274,1.6951516460296583,-0.20749622643520274};
    
    // sim, h25 sl0.15
    // double init_eta[8] = {1.6742169285081276,0.15660187698201541,1.6742169285081276,0.15660187698201541,1.6742169285081276,-0.15660187698201541,1.6742169285081276,-0.15660187698201541};
    
    // sim, h25 sl0.1
    // double init_eta[8] = {1.659128421025593,0.10487693873023388,1.659128421025593,0.10487693873023388,1.659128421025593,-0.10487693873023388,1.659128421025593,-0.10487693873023388};
    

    // real, h20 sl0.2
    // double init_eta[8] = {1.1771845578593103,0.269521537734618,1.1771845578593103,0.269521537734618,1.1771845578593103,-0.269521537734618,1.1771845578593103,-0.269521537734618};

    // real, h20 sl0.15
    // double init_eta[8] = {1.1487944075380767,0.20429200223822921,1.1487944075380767,0.20429200223822921,1.1487944075380767,-0.20429200223822921,1.1487944075380767,-0.20429200223822921};
    
    // real, h20 sl0.1
    // double init_eta[8] = {1.1281967376059958,0.13725312513994078,1.1281967376059958,0.13725312513994078,1.1281967376059958,-0.13725312513994078,1.1281967376059958,-0.13725312513994078};
    
    // real, h25 sl0.2
    // double init_eta[8] = {1.6363085991866155,0.2131618881919698,1.6363085991866155,0.2131618881919698,1.6363085991866155,-0.2131618881919698,1.6363085991866155,-0.2131618881919698};
    
    // real, h25 sl0.15
    // double init_eta[8] = {1.614734905606272,0.16093373925617896,1.614734905606272,0.16093373925617896,1.614734905606272,-0.16093373925617896,1.614734905606272,-0.16093373925617896};
    
    // real, h25 sl0.1
    // double init_eta[8] = {1.59917444120165,0.1078055178775231,1.59917444120165,0.1078055178775231,1.59917444120165,-0.1078055178775231,1.59917444120165,-0.1078055178775231};
    
    
    TrotGait walk_gait(sim, 0, 1000);

    double velocity        = 0.4;  // 0.4, 0.45
    double stand_height    = 0.25;  // 0.2, 0.25
    double step_length     = 0.2;
    double step_height     = 0.04;
    
    walk_gait.set_velocity(velocity);
    walk_gait.set_stand_height(stand_height);
    walk_gait.set_step_length(step_length);
    walk_gait.set_step_height(step_height);
    walk_gait.initialize(init_eta);

    std::array<std::array<double, 4>, 2> eta_list = {{{17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI, 17.0/180.0*M_PI}, {0, 0, 0, 0}}};
    int target_loop = 5000;

    // initialize motor command
    for (auto& cmd : motor_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->kp_r = 90;
        cmd->kp_l = 90;
        cmd->ki_r = 0;
        cmd->ki_l = 0;
        if (sim) {
            cmd->kd_r = 1;
            cmd->kd_l = 1;
        }
        else {
            cmd->kd_r = 1.75;
            cmd->kd_l = 1.75;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Transform Starts\n");

    // transform
    for (int i=0; i<3000; i++) {
        for (int j=0; j<4; j++) {
            motor_cmd_modules[j]->theta += (init_eta[2*j]-17/180.0*M_PI)/3000.0;
            motor_cmd_modules[j]->beta += init_eta[2*j+1]/3000.0;
        }
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Transform Finished\n");

    // stay
    for (int i=0; i<2000; i++) {
        motor_cmd.header.seq = -1;
        motor_cmd_pub.publish(motor_cmd);
        rate.sleep();
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (trigger){
            RCLCPP_INFO(rclcpp::get_logger("CorgiForceEstimation"), "Controller Starts ...\n");

            int loop_count = 0;
            while (rclcpp::ok()) {
                rclcpp::spin_some(node);

                // if (loop_count > target_loop-3000 && loop_count < target_loop) {
                //     velocity -= 0.2/3000.0;
                //     walk_gait.set_velocity(velocity);
                // }

                // get next eta
                eta_list = walk_gait.step();

                for (int i=0; i<4; i++) {
                    if (eta_list[0][i] > M_PI*160.0/180.0) {
                        std::cout << "Exceed upper bound." << std::endl;
                    }
                    if (eta_list[0][i] < M_PI*17.0/180.0) {
                        std::cout << "Exceed lower bound." << std::endl;
                    }
                    motor_cmd_modules[i]->theta = eta_list[0][i];
                    motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
                }

                motor_cmd.header.seq = loop_count;
                motor_cmd_pub.publish(motor_cmd);

                loop_count++;
                if (loop_count >= target_loop) break;

                rate.sleep();
            }
            break;
        }
        rate.sleep();
    }
    return 0;
}