#include <iostream>
#include <signal.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <corgi_sim/srv/set_int.hpp>
#include <corgi_sim/srv/set_float.hpp>
#include <corgi_sim/srv/field_get_string.hpp>
#include <corgi_sim/srv/get_uint64.hpp>
#include <corgi_sim/srv/motor_set_control_pid.hpp>
#include <corgi_sim/srv/node_enable_contact_points_tracking.hpp>
#include <corgi_sim/srv/node_get_contact_points.hpp>
#include <corgi_sim/srv/node_get_def.hpp>
#include <corgi_sim/srv/node_get_field.hpp>
#include <corgi_sim/srv/node_get_position.hpp>
#include <corgi_sim/srv/node_get_orientation.hpp>
#include <corgi_sim/srv/supervisor_get_from_id.hpp>
#include <corgi_sim/msg/float64_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <corgi_msgs/msg/sim_contact_point.hpp>
#include <corgi_msgs/msg/sim_data_stamped.hpp>


corgi_msgs::msg::MotorCmdStamped motor_cmd;
corgi_msgs::msg::MotorStateStamped motor_state;
corgi_msgs::msg::TriggerStamped trigger;
corgi_msgs::msg::SimDataStamped sim_data;
sensor_msgs::msg::Imu imu;
sensor_msgs::msg::Imu imu_filtered;

double AR_phi = 0.0;
double AL_phi = 0.0;
double BR_phi = 0.0;
double BL_phi = 0.0;
double CR_phi = 0.0;
double CL_phi = 0.0;
double DR_phi = 0.0;
double DL_phi = 0.0;

double AR_phi_dot = 0.0;
double AL_phi_dot = 0.0;
double BR_phi_dot = 0.0;
double BL_phi_dot = 0.0;
double CR_phi_dot = 0.0;
double CL_phi_dot = 0.0;
double DR_phi_dot = 0.0;
double DL_phi_dot = 0.0;

corgi_sim::srv::set_float AR_motor_trq_srv;
corgi_sim::srv::set_float AL_motor_trq_srv;
corgi_sim::srv::set_float BR_motor_trq_srv;
corgi_sim::srv::set_float BL_motor_trq_srv;
corgi_sim::srv::set_float CR_motor_trq_srv;
corgi_sim::srv::set_float CL_motor_trq_srv;
corgi_sim::srv::set_float DR_motor_trq_srv;
corgi_sim::srv::set_float DL_motor_trq_srv;

corgi_sim::srv::set_int time_step_srv;

corgi_sim::srv::get_uint64 node_value_srv;
corgi_sim::srv::node_get_position node_pos_srv;
corgi_sim::srv::node_get_orientation node_orien_srv;

corgi_sim::srv::node_enable_contact_points_tracking cp_enable_srv;
corgi_sim::srv::node_get_contact_points cp_srv;

rosgraph_msgs::msg::Clock simulationClock;

void motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped cmd) { motor_cmd = cmd; }

void AR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { AR_phi_dot = (phi.data - AR_phi)*1000; AR_phi = phi.data; }
void AL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { AL_phi_dot = (phi.data - AL_phi)*1000; AL_phi = phi.data; }
void BR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { BR_phi_dot = (phi.data - BR_phi)*1000; BR_phi = phi.data; }
void BL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { BL_phi_dot = (phi.data - BL_phi)*1000; BL_phi = phi.data; }
void CR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { CR_phi_dot = (phi.data - CR_phi)*1000; CR_phi = phi.data; }
void CL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { CL_phi_dot = (phi.data - CL_phi)*1000; CL_phi = phi.data; }
void DR_encoder_cb(corgi_sim::msg::Float64Stamped phi) { DR_phi_dot = (phi.data - DR_phi)*1000; DR_phi = phi.data; }
void DL_encoder_cb(corgi_sim::msg::Float64Stamped phi) { DL_phi_dot = (phi.data - DL_phi)*1000; DL_phi = phi.data; }

void gyro_cb(sensor_msgs::msg::Imu values) { imu.orientation = values.orientation; }
void ang_vel_cb(sensor_msgs::msg::Imu values) { imu.angular_velocity = values.angular_velocity; }
void imu_cb(sensor_msgs::msg::Imu values) { imu.linear_acceleration = values.linear_acceleration; }

double find_closest_phi(double phi_ref, double phi_fb) {
    double diff = fmod(phi_ref - phi_fb + M_PI, 2 * M_PI);
    if (diff < 0) diff += 2 * M_PI;
    return phi_fb + diff - M_PI;
}

void phi2tb(double phi_r, double phi_l, double &theta, double &beta){
    theta = (phi_l - phi_r) / 2.0 + 17 / 180.0 * M_PI;
    beta  = (phi_l + phi_r) / 2.0;
}

void tb2phi(corgi_msgs::msg::MotorCmd motor_cmd, double &trq_r, double &trq_l, double phi_r_fb, double phi_l_fb, double phi_r_dot_fb, double phi_l_dot_fb){
    if (motor_cmd.theta == 0) {
        motor_cmd.kp_r = 90;
        motor_cmd.kp_l = 90;
        motor_cmd.kd_r = 1.75;
        motor_cmd.kd_l = 1.75;
    }
    
    double theta_0 = 17 / 180.0 * M_PI;
    if (motor_cmd.theta < theta_0) {motor_cmd.theta = theta_0;}
    
    double phi_r = find_closest_phi(motor_cmd.beta - motor_cmd.theta + theta_0, phi_r_fb);
    double phi_l = find_closest_phi(motor_cmd.beta + motor_cmd.theta - theta_0, phi_l_fb);

    trq_r = motor_cmd.kp_r*(phi_r-phi_r_fb) + motor_cmd.kd_r*(-phi_r_dot_fb) + motor_cmd.torque_r;
    trq_l = motor_cmd.kp_l*(phi_l-phi_l_fb) + motor_cmd.kd_l*(-phi_l_dot_fb) + motor_cmd.torque_l;
}

std::string get_lastest_input() {
    std::string input;
    auto start = std::chrono::steady_clock::now();
    while (true) {
        std::getline(std::cin, input);
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= 100) { return input; }
    }
}

void signal_handler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiSim"), "Interrupt received.");
    ros::shutdown();
    exit(signum);
}


int main(int argc, char **argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiSim"), "Corgi Simulation Starts\n");

    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_sim");

    auto clock_pub = nh.advertise<rosgraph_msgs::msg::Clock>("/clock", 1);
    
    ros::service::waitForService("/robot/time_step");
    rclcpp::spin_some(node);

    ros::ServiceClient time_step_client = nh.serviceClient<corgi_sim::srv::set_int>("robot/time_step");

    ros::ServiceClient node_pos_client = nh.serviceClient<corgi_sim::srv::node_get_position>("supervisor/node/get_position");
    ros::ServiceClient node_orient_client = nh.serviceClient<corgi_sim::srv::node_get_orientation>("supervisor/node/get_orientation");

    ros::ServiceClient AR_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("lf_left_motor/set_torque");
    ros::ServiceClient AL_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("lf_right_motor/set_torque");
    ros::ServiceClient BR_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("rf_left_motor/set_torque");
    ros::ServiceClient BL_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("rf_right_motor/set_torque");
    ros::ServiceClient CR_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("rh_left_motor/set_torque");
    ros::ServiceClient CL_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("rh_right_motor/set_torque");
    ros::ServiceClient DR_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("lh_left_motor/set_torque");
    ros::ServiceClient DL_motor_trq_client = nh.serviceClient<corgi_sim::srv::set_float>("lh_right_motor/set_torque");
    
    ros::ServiceClient cp_enable_client = nh.serviceClient<corgi_sim::srv::node_enable_contact_points_tracking>("supervisor/node/enable_contact_points_tracking");
    ros::ServiceClient cp_get_client = nh.serviceClient<corgi_sim::srv::node_get_contact_points>("supervisor/node/get_contact_points");

    auto AR_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("lf_left_motor_sensor/value" , 1, AR_encoder_cb);
    auto AL_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("lf_right_motor_sensor/value", 1, AL_encoder_cb);
    auto BR_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("rf_left_motor_sensor/value" , 1, BR_encoder_cb);
    auto BL_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("rf_right_motor_sensor/value", 1, BL_encoder_cb);
    auto CR_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("rh_left_motor_sensor/value" , 1, CR_encoder_cb);
    auto CL_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("rh_right_motor_sensor/value", 1, CL_encoder_cb);
    auto DR_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("lh_left_motor_sensor/value" , 1, DR_encoder_cb);
    auto DL_encoder_sub = nh.subscribe<corgi_sim::msg::Float64Stamped>("lh_right_motor_sensor/value", 1, DL_encoder_cb);
    
    auto gyro_sub = nh.subscribe<sensor_msgs::msg::Imu>("gyro/quaternion", 1, gyro_cb);
    auto ang_vel_sub = nh.subscribe<sensor_msgs::msg::Imu>("ang_vel/values", 1, ang_vel_cb);
    auto imu_sub = nh.subscribe<sensor_msgs::msg::Imu>("imu/values", 1, imu_cb);

    auto motor_cmd_sub = nh.subscribe<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1, motor_cmd_cb);
    auto motor_state_pub = nh.advertise<corgi_msgs::msg::MotorStateStamped>("motor/state", 1000);
    auto imu_pub = nh.advertise<sensor_msgs::msg::Imu>("imu", 1000);
    auto trigger_pub = nh.advertise<corgi_msgs::msg::TriggerStamped>("trigger", 1000);
    auto sim_data_pub = nh.advertise<corgi_msgs::msg::SimDataStamped>("sim/data", 1000);
    
    ros::WallRate rate(1000);

    node_value_srv.request.ask = true;
    ros::service::call("/supervisor/get_self", node_value_srv);

    node_pos_srv.request.node = node_value_srv.response.value;
    node_orien_srv.request.node = node_value_srv.response.value;
    
    cp_enable_srv.request.node = node_value_srv.response.value;
    cp_enable_srv.request.include_descendants = true;

    cp_srv.request.node = node_value_srv.response.value;
    cp_srv.request.include_descendants = true;

    signal(SIGINT, signal_handler);

    trigger.enable = true;
    time_step_srv.request.value = 1;

    std::cout << "\nInput the output filename and press Enter to start the simulation: ";
    trigger.output_filename = get_lastest_input();

    int loop_counter = 0;
    while (rclcpp::ok() && time_step_client.call(time_step_srv)){
        rclcpp::spin_some(node);
        
        node_pos_client.call(node_pos_srv);
        node_orient_client.call(node_orien_srv);

        sim_data.header.seq = loop_counter;
        sim_data.position = node_pos_srv.response.position;
        sim_data.orientation = node_orien_srv.response.orientation;

        tb2phi(motor_cmd.module_a, AR_motor_trq_srv.request.value, AL_motor_trq_srv.request.value, AR_phi, AL_phi, AR_phi_dot, AL_phi_dot);
        tb2phi(motor_cmd.module_b, BR_motor_trq_srv.request.value, BL_motor_trq_srv.request.value, BR_phi, BL_phi, BR_phi_dot, BL_phi_dot);
        tb2phi(motor_cmd.module_c, CR_motor_trq_srv.request.value, CL_motor_trq_srv.request.value, CR_phi, CL_phi, CR_phi_dot, CL_phi_dot);
        tb2phi(motor_cmd.module_d, DR_motor_trq_srv.request.value, DL_motor_trq_srv.request.value, DR_phi, DL_phi, DR_phi_dot, DL_phi_dot);
        
        AR_motor_trq_client.call(AR_motor_trq_srv);
        AL_motor_trq_client.call(AL_motor_trq_srv);
        BR_motor_trq_client.call(BR_motor_trq_srv);
        BL_motor_trq_client.call(BL_motor_trq_srv);
        CR_motor_trq_client.call(CR_motor_trq_srv);
        CL_motor_trq_client.call(CL_motor_trq_srv);
        DR_motor_trq_client.call(DR_motor_trq_srv);
        DL_motor_trq_client.call(DL_motor_trq_srv);
        
        phi2tb(AR_phi, AL_phi, motor_state.module_a.theta, motor_state.module_a.beta);
        phi2tb(BR_phi, BL_phi, motor_state.module_b.theta, motor_state.module_b.beta);
        phi2tb(CR_phi, CL_phi, motor_state.module_c.theta, motor_state.module_c.beta);
        phi2tb(DR_phi, DL_phi, motor_state.module_d.theta, motor_state.module_d.beta);

        motor_state.header.seq = loop_counter;
        
        motor_state.module_a.torque_r = AR_motor_trq_srv.request.value;
        motor_state.module_a.torque_l = AL_motor_trq_srv.request.value;
        motor_state.module_b.torque_r = BR_motor_trq_srv.request.value;
        motor_state.module_b.torque_l = BL_motor_trq_srv.request.value;
        motor_state.module_c.torque_r = CR_motor_trq_srv.request.value;
        motor_state.module_c.torque_l = CL_motor_trq_srv.request.value;
        motor_state.module_d.torque_r = DR_motor_trq_srv.request.value;
        motor_state.module_d.torque_l = DL_motor_trq_srv.request.value;

        motor_state.module_a.velocity_r = AR_phi_dot;
        motor_state.module_a.velocity_l = AL_phi_dot;
        motor_state.module_b.velocity_r = BR_phi_dot;
        motor_state.module_b.velocity_l = BL_phi_dot;
        motor_state.module_c.velocity_r = CR_phi_dot;
        motor_state.module_c.velocity_l = CL_phi_dot;
        motor_state.module_d.velocity_r = DR_phi_dot;
        motor_state.module_d.velocity_l = DL_phi_dot;

        cp_get_client.call(cp_srv);
        size_t n = cp_srv.response.contact_points.size();
        sim_data.contact.clear();
        if (n > 0) {
            for (int m=0; m<n; m++) {
                const auto& p = cp_srv.response.contact_points[m];

                corgi_sim::srv::supervisor_get_from_id node_get_id_srv;
                node_get_id_srv.request.id = p.node_id;
                ros::service::call("supervisor/get_from_id", node_get_id_srv);

                corgi_sim::srv::node_get_field field_get_srv;
                field_get_srv.request.node = node_get_id_srv.response.node;
                field_get_srv.request.fieldName = "name";
                ros::service::call("supervisor/node/get_field", field_get_srv);

                corgi_sim::srv::field_get_string name_get_srv;
                name_get_srv.request.field = field_get_srv.response.field;
                ros::service::call("supervisor/field/get_string", name_get_srv);

                corgi_sim::srv::node_get_def def_get_srv;
                def_get_srv.request.node = node_get_id_srv.response.node;
                ros::service::call("supervisor/node/get_def", def_get_srv);

                corgi_msgs::msg::SimContactPoint contact;
                contact.def_name = def_get_srv.response.name;
                contact.name = name_get_srv.response.value;
                contact.point = p.point;
                sim_data.contact.push_back(contact);
            }
        }

        Eigen::Quaterniond orientation(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
        Eigen::Vector3d linear_acceleration(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        Eigen::Vector3d gravity_global(0, 0, 9.81);
        Eigen::Vector3d gravity_body = orientation.inverse() * gravity_global;

        linear_acceleration -= gravity_body;
        
        imu_filtered.header.seq = loop_counter;
        imu_filtered.orientation.w = imu.orientation.w;
        imu_filtered.orientation.x = imu.orientation.x;
        imu_filtered.orientation.y = imu.orientation.y;
        imu_filtered.orientation.z = imu.orientation.z;
        imu_filtered.angular_velocity.x = imu.angular_velocity.x;
        imu_filtered.angular_velocity.y = imu.angular_velocity.y;
        imu_filtered.angular_velocity.z = imu.angular_velocity.z; 
        imu_filtered.linear_acceleration.x = linear_acceleration(0);
        imu_filtered.linear_acceleration.y = linear_acceleration(1);
        imu_filtered.linear_acceleration.z = linear_acceleration(2);

        motor_state_pub.publish(motor_state);
        trigger_pub.publish(trigger);
        imu_pub.publish(imu_filtered);
        sim_data_pub.publish(sim_data);

        double clock = loop_counter*0.001;
        simulationClock.clock.sec = (int)clock;
        simulationClock.clock.nsec = round(1000 * (clock - simulationClock.clock.sec)) * 1.0e+6;
        clock_pub.publish(simulationClock);

        loop_counter++;

        rate.sleep();
    }

    trigger.enable = false;
    trigger_pub.publish(trigger);

    ros::shutdown();

    return 0;
}
