#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/impedance_cmd_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include "force_estimation.hpp"

bool trigger = false;

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;
}

int main(int argc, char **argv) {

    RCLCPP_INFO(rclcpp::get_logger("CorgiForceControl"), "Impedance Command Publisher Starts\n");
    
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("imp_cmd_pub");
    auto imp_cmd_pub = nh.advertise<corgi_msgs::msg::ImpedanceCmdStamped>("impedance/command", 1000);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, trigger_cb);
    rclcpp::Rate rate(1000);

    corgi_msgs::msg::ImpedanceCmdStamped imp_cmd;

    std::vector<corgi_msgs::msg::ImpedanceCmd*> imp_cmd_modules = {
        &imp_cmd.module_a,
        &imp_cmd.module_b,
        &imp_cmd.module_c,
        &imp_cmd.module_d
    };

    // robot weight ~= 220 N
    for (auto& cmd : imp_cmd_modules){
        cmd->theta = 17/180.0*M_PI;
        cmd->beta = 0/180.0*M_PI;
        cmd->Fx = 0;
        cmd->Fy = 0;
        cmd->Mx = 0;
        cmd->My = 0;
        cmd->Bx = 30;
        cmd->By = 10;
        cmd->Kx = 500;
        cmd->Ky = 200;
    }

    std::array<double, 2> eta;
    double ds = 0.0;
    double mg = 195.0;
    
    for (int i=0; i<2000; i++){
        eta = legmodel.move(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta, {-0.08/2000.0, 0.15/2000.0});
        
        imp_cmd_modules[0]->theta = eta[0];
        imp_cmd_modules[1]->theta = eta[0];
        imp_cmd_modules[2]->theta = eta[0];
        imp_cmd_modules[3]->theta = eta[0];

        imp_cmd_modules[0]->beta = -eta[1];
        imp_cmd_modules[1]->beta = eta[1];
        imp_cmd_modules[2]->beta = eta[1];
        imp_cmd_modules[3]->beta = -eta[1];
        
        legmodel.contact_map(eta[0], eta[1]);
        double s_front = 0.222+legmodel.contact_p[0];
        double f_hind = -mg/2.0*(s_front/0.444);
        imp_cmd_modules[0]->Fy = -mg/2.0 - f_hind;
        imp_cmd_modules[1]->Fy = -mg/2.0 - f_hind;
        imp_cmd_modules[2]->Fy = f_hind;
        imp_cmd_modules[3]->Fy = f_hind;

        imp_cmd.header.seq = -1;

        imp_cmd_pub.publish(imp_cmd);

        rate.sleep();
    }
        
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        if (trigger){
            int loop_count = 0;
            while (rclcpp::ok()) {
                if (loop_count < 2000) {

                }
                else if (loop_count < 22000) {
                    ds = 0.08*M_PI/2000.0*sin((loop_count-2000)*M_PI/2000.0);
                    eta = legmodel.move(imp_cmd_modules[1]->theta, imp_cmd_modules[1]->beta, {ds, 0.0});

                    imp_cmd_modules[0]->theta = eta[0];
                    imp_cmd_modules[1]->theta = eta[0];
                    imp_cmd_modules[2]->theta = eta[0];
                    imp_cmd_modules[3]->theta = eta[0];

                    imp_cmd_modules[0]->beta = -eta[1];
                    imp_cmd_modules[1]->beta = eta[1];
                    imp_cmd_modules[2]->beta = eta[1];
                    imp_cmd_modules[3]->beta = -eta[1];

                    legmodel.contact_map(eta[0], eta[1]);
                    double s_front = 0.222+legmodel.contact_p[0];
                    double f_hind = -mg/2.0*(s_front/0.444);
                    imp_cmd_modules[0]->Fy = -mg/2.0 - f_hind;
                    imp_cmd_modules[1]->Fy = -mg/2.0 - f_hind;
                    imp_cmd_modules[2]->Fy = f_hind;
                    imp_cmd_modules[3]->Fy = f_hind;

                    if (loop_count > 12000) {
                        imp_cmd_modules[0]->Fy += 10 * sin((loop_count-2000)/1000.0*M_PI);
                        imp_cmd_modules[1]->Fy -= 10 * sin((loop_count-2000)/1000.0*M_PI);
                        imp_cmd_modules[2]->Fy += 10 * sin((loop_count-2000)/1000.0*M_PI);
                        imp_cmd_modules[3]->Fy -= 10 * sin((loop_count-2000)/1000.0*M_PI);
                    }
                }
                else {
                    break;
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