#include <iostream>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

#include "NodeHandler.h"
#include "Config.pb.h"
#include "Log.pb.h"
#include "Motor.pb.h"
#include "Power.pb.h"
#include "Robot.pb.h"
#include "Steering.pb.h"

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <corgi_msgs/msg/config_stamped.hpp>
#include <corgi_msgs/msg/log_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/power_state_stamped.hpp>
#include <corgi_msgs/msg/robot_cmd_stamped.hpp>
#include <corgi_msgs/msg/robot_state_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>

std::mutex mutex_grpc_motor_cmd;
std::mutex mutex_grpc_robot_cmd;
std::mutex mutex_grpc_steer_cmd;
std::mutex mutex_grpc_config_cmd;

std::mutex mutex_ros_motor_state;
std::mutex mutex_ros_power_state;
std::mutex mutex_ros_robot_state;
std::mutex mutex_ros_steer_state;
std::mutex mutex_ros_config_state;

std::mutex mutex_ros_log;

corgi_msgs::msg::MotorCmdStamped         ros_motor_cmd;
corgi_msgs::msg::RobotCmdStamped         ros_robot_cmd;
corgi_msgs::msg::SteeringCmdStamped      ros_steer_cmd;
corgi_msgs::msg::ConfigStamped           ros_config_cmd;
corgi_msgs::msg::MotorStateStamped       ros_motor_state;
corgi_msgs::msg::PowerStateStamped       ros_power_state;
corgi_msgs::msg::RobotStateStamped       ros_robot_state;
corgi_msgs::msg::SteeringStateStamped    ros_steer_state;
corgi_msgs::msg::ConfigStamped           ros_config_state;
corgi_msgs::msg::LogStamped              ros_log;

motor_msg::MotorCmdStamped          grpc_motor_cmd;
robot_msg::RobotCmdStamped          grpc_robot_cmd;
steering_msg::SteeringCmdStamped    grpc_steer_cmd;
config_msg::ConfigStamped           grpc_config_cmd;
motor_msg::MotorStateStamped        grpc_motor_state;
power_msg::PowerStateStamped        grpc_power_state;
robot_msg::RobotStateStamped        grpc_robot_state;
steering_msg::SteeringStateStamped  grpc_steer_state;
config_msg::ConfigStamped           grpc_config_state;
log_msg::LogEntry                   grpc_log;

rclcpp::Publisher<corgi_msgs::msg::MotorStateStamped>::SharedPtr ros_motor_state_pub; 
rclcpp::Publisher<corgi_msgs::msg::PowerStateStamped>::SharedPtr ros_power_state_pub;
rclcpp::Publisher<corgi_msgs::msg::RobotStateStamped>::SharedPtr ros_robot_state_pub; 
rclcpp::Publisher<corgi_msgs::msg::SteeringStateStamped>::SharedPtr ros_steer_state_pub;
rclcpp::Publisher<corgi_msgs::msg::ConfigStamped>::SharedPtr ros_config_state_pub;
rclcpp::Publisher<corgi_msgs::msg::LogStamped>::SharedPtr ros_log_pub;

core::Publisher<motor_msg::MotorCmdStamped>         *grpc_motor_cmd_pub;
core::Publisher<robot_msg::RobotCmdStamped>         *grpc_robot_cmd_pub;
core::Publisher<steering_msg::SteeringCmdStamped>   *grpc_steer_cmd_pub;
core::Publisher<config_msg::ConfigStamped>          *grpc_config_cmd_pub;

void ros_motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped cmd){
    std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);

    ros_motor_cmd = cmd;

    std::vector<motor_msg::MotorCmd*> grpc_motor_modules = {
        grpc_motor_cmd.mutable_module_a(),
        grpc_motor_cmd.mutable_module_b(),
        grpc_motor_cmd.mutable_module_c(),
        grpc_motor_cmd.mutable_module_d()
    };

    std::vector<corgi_msgs::msg::MotorCmd> ros_motor_modules = {
        ros_motor_cmd.module_a,
        ros_motor_cmd.module_b,
        ros_motor_cmd.module_c,
        ros_motor_cmd.module_d
    };

    for (int i = 0; i < 4; i++) {
        grpc_motor_modules[i]->set_theta(std::min(std::max(ros_motor_modules[i].theta, 17/180.0*M_PI), 160/180.0*M_PI));
        grpc_motor_modules[i]->set_beta(ros_motor_modules[i].beta);
        grpc_motor_modules[i]->set_kp_r(ros_motor_modules[i].kp_r);
        grpc_motor_modules[i]->set_kp_l(ros_motor_modules[i].kp_l);
        grpc_motor_modules[i]->set_ki_r(ros_motor_modules[i].ki_r);
        grpc_motor_modules[i]->set_ki_l(ros_motor_modules[i].ki_l);
        grpc_motor_modules[i]->set_kd_r(ros_motor_modules[i].kd_r);
        grpc_motor_modules[i]->set_kd_l(ros_motor_modules[i].kd_l);
        grpc_motor_modules[i]->set_torque_r(ros_motor_modules[i].torque_r);
        grpc_motor_modules[i]->set_torque_l(ros_motor_modules[i].torque_l);
    }

    grpc_motor_cmd.mutable_header()->set_seq(ros_motor_cmd.header.seq);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(ros_motor_cmd.header.stamp.sec);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(ros_motor_cmd.header.stamp.nanosec / 1000);

    grpc_motor_cmd_pub->publish(grpc_motor_cmd);
}

void ros_steer_cmd_cb(const corgi_msgs::msg::SteeringCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_steer_cmd);

    ros_steer_cmd = cmd;

    grpc_steer_cmd.set_voltage(ros_steer_cmd.voltage);
    grpc_steer_cmd.set_angle(ros_steer_cmd.angle);

    grpc_steer_cmd.mutable_header()->set_seq(ros_steer_cmd.header.seq);
    grpc_steer_cmd.mutable_header()->mutable_stamp()->set_sec(ros_steer_cmd.header.stamp.sec);
    grpc_steer_cmd.mutable_header()->mutable_stamp()->set_usec(ros_steer_cmd.header.stamp.nanosec / 1000);

    grpc_steer_cmd_pub->publish(grpc_steer_cmd);
}

void ros_robot_cmd_cb(const corgi_msgs::msg::RobotCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_robot_cmd);

    ros_robot_cmd = cmd;

    grpc_robot_cmd.set_request_robot_mode(static_cast<robot_msg::ROBOTMODE>(ros_robot_cmd.request_robot_mode));

    grpc_robot_cmd.mutable_header()->set_seq(ros_robot_cmd.header.seq);
    grpc_robot_cmd.mutable_header()->mutable_stamp()->set_sec(ros_robot_cmd.header.stamp.sec);
    grpc_robot_cmd.mutable_header()->mutable_stamp()->set_usec(ros_robot_cmd.header.stamp.nanosec / 1000);

    grpc_robot_cmd_pub->publish(grpc_robot_cmd);
}

void ros_config_cmd_cb(const corgi_msgs::msg::ConfigStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_config_cmd);

    ros_config_cmd = cmd;

    // Map ROS message fields to gRPC proto fields according to Config.proto
    grpc_config_cmd.set_transmit(ros_config_cmd.transmit);
    grpc_config_cmd.set_module(static_cast<config_msg::Module>(ros_config_cmd.module));
    grpc_config_cmd.set_motor(static_cast<config_msg::Motor>(ros_config_cmd.motor));
    grpc_config_cmd.set_mode(static_cast<config_msg::ConfigMode>(ros_config_cmd.mode));
    grpc_config_cmd.set_type(static_cast<config_msg::ConfigType>(ros_config_cmd.type));
    grpc_config_cmd.set_address(ros_config_cmd.address);
    grpc_config_cmd.set_value_f(ros_config_cmd.value_f);
    grpc_config_cmd.set_value_i(ros_config_cmd.value_i);
    grpc_config_cmd.set_error_code(ros_config_cmd.error_code);

    grpc_config_cmd.mutable_header()->set_seq(ros_config_cmd.header.seq);
    grpc_config_cmd.mutable_header()->mutable_stamp()->set_sec(ros_config_cmd.header.stamp.sec);
    grpc_config_cmd.mutable_header()->mutable_stamp()->set_usec(ros_config_cmd.header.stamp.nanosec / 1000);

    grpc_config_cmd_pub->publish(grpc_config_cmd);
}

void grpc_motor_state_cb(const motor_msg::MotorStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_motor_state);

    grpc_motor_state = state;

    std::vector<const motor_msg::MotorState*> grpc_motor_modules = {
        &grpc_motor_state.module_a(),
        &grpc_motor_state.module_b(),
        &grpc_motor_state.module_c(),
        &grpc_motor_state.module_d()
    };

    std::vector<corgi_msgs::msg::MotorState*> ros_motor_modules = {
        &ros_motor_state.module_a,
        &ros_motor_state.module_b,
        &ros_motor_state.module_c,
        &ros_motor_state.module_d
    };

    for (int i = 0; i < 4; i++) {
        ros_motor_modules[i]->theta = grpc_motor_modules[i]->theta();
        ros_motor_modules[i]->beta = grpc_motor_modules[i]->beta();
        ros_motor_modules[i]->velocity_r = grpc_motor_modules[i]->velocity_r();
        ros_motor_modules[i]->velocity_l = grpc_motor_modules[i]->velocity_l();
        ros_motor_modules[i]->torque_r = grpc_motor_modules[i]->torque_r();
        ros_motor_modules[i]->torque_l = grpc_motor_modules[i]->torque_l();
    }

    ros_motor_state.motor_mode = static_cast<int32_t>(grpc_motor_state.motor_mode());
    ros_motor_state.header.seq = grpc_motor_state.header().seq();
    ros_motor_state.header.stamp.sec = grpc_motor_state.header().stamp().sec();
    ros_motor_state.header.stamp.nanosec = grpc_motor_state.header().stamp().usec() * 1000;

    ros_motor_state_pub->publish(ros_motor_state);
}

void grpc_power_state_cb(const power_msg::PowerStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_power_state);

    grpc_power_state = state;

    ros_power_state.digital = grpc_power_state.digital();
    ros_power_state.signal = grpc_power_state.signal();
    ros_power_state.power = grpc_power_state.power();
    ros_power_state.clean = grpc_power_state.clean();
    // ros_power_state.robot_mode = grpc_power_state.robot_mode();
    ros_power_state.v_0 = grpc_power_state.v_0();
    ros_power_state.i_0 = grpc_power_state.i_0();
    ros_power_state.v_1 = grpc_power_state.v_1();
    ros_power_state.i_1 = grpc_power_state.i_1();
    ros_power_state.v_2 = grpc_power_state.v_2();
    ros_power_state.i_2 = grpc_power_state.i_2();
    ros_power_state.v_3 = grpc_power_state.v_3();
    ros_power_state.i_3 = grpc_power_state.i_3();
    ros_power_state.v_4 = grpc_power_state.v_4();
    ros_power_state.i_4 = grpc_power_state.i_4();
    ros_power_state.v_5 = grpc_power_state.v_5();
    ros_power_state.i_5 = grpc_power_state.i_5();
    ros_power_state.v_6 = grpc_power_state.v_6();
    ros_power_state.i_6 = grpc_power_state.i_6();
    ros_power_state.v_7 = grpc_power_state.v_7();
    ros_power_state.i_7 = grpc_power_state.i_7();
    ros_power_state.v_8 = grpc_power_state.v_8();
    ros_power_state.i_8 = grpc_power_state.i_8();
    ros_power_state.v_9 = grpc_power_state.v_9();
    ros_power_state.i_9 = grpc_power_state.i_9();
    ros_power_state.v_10 = grpc_power_state.v_10();
    ros_power_state.i_10 = grpc_power_state.i_10();
    ros_power_state.v_11 = grpc_power_state.v_11();
    ros_power_state.i_11 = grpc_power_state.i_11();

    ros_power_state.header.seq = grpc_power_state.header().seq();
    ros_power_state.header.stamp.sec = grpc_power_state.header().stamp().sec();
    ros_power_state.header.stamp.nanosec = grpc_power_state.header().stamp().usec() * 1000;

    ros_power_state_pub->publish(ros_power_state);
}

void grpc_steer_state_cb(const steering_msg::SteeringStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_steer_state);

    grpc_steer_state = state;

    ros_steer_state.current_angle = grpc_steer_state.current_angle();
    ros_steer_state.current_state = grpc_steer_state.current_state();
    ros_steer_state.cmd_finish = grpc_steer_state.cmd_finish();
    ros_steer_state.steering_cali = grpc_steer_state.steering_cali();

    ros_steer_state.header.seq = grpc_steer_state.header().seq();
    ros_steer_state.header.stamp.sec = grpc_steer_state.header().stamp().sec();
    ros_steer_state.header.stamp.nanosec = grpc_steer_state.header().stamp().usec() * 1000;

    ros_steer_state_pub->publish(ros_steer_state);
}

void grpc_robot_state_cb(const robot_msg::RobotStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_robot_state);

    grpc_robot_state = state;

    ros_robot_state.robot_mode = grpc_robot_state.robot_mode();

    ros_robot_state.header.seq = grpc_robot_state.header().seq();
    ros_robot_state.header.stamp.sec = grpc_robot_state.header().stamp().sec();
    ros_robot_state.header.stamp.nanosec = grpc_robot_state.header().stamp().usec() * 1000;

    ros_robot_state_pub->publish(ros_robot_state);
}

void grpc_config_state_cb(const config_msg::ConfigStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_config_state);

    grpc_config_state = state;

    // Map gRPC proto fields to ROS message fields according to Config.proto
    ros_config_state.transmit = grpc_config_state.transmit();
    ros_config_state.module = static_cast<int32_t>(grpc_config_state.module());
    ros_config_state.motor = static_cast<int32_t>(grpc_config_state.motor());
    ros_config_state.mode = static_cast<int32_t>(grpc_config_state.mode());
    ros_config_state.type = static_cast<int32_t>(grpc_config_state.type());
    ros_config_state.address = grpc_config_state.address();
    ros_config_state.value_f = grpc_config_state.value_f();
    ros_config_state.value_i = grpc_config_state.value_i();
    ros_config_state.error_code = grpc_config_state.error_code();

    ros_config_state.header.seq = grpc_config_state.header().seq();
    ros_config_state.header.stamp.sec = grpc_config_state.header().stamp().sec();
    ros_config_state.header.stamp.nanosec = grpc_config_state.header().stamp().usec() * 1000;

    ros_config_state_pub->publish(ros_config_state);
}

void grpc_log_cb(const log_msg::LogEntry log) {
    std::lock_guard<std::mutex> lock(mutex_ros_log);

    grpc_log = log;

    ros_log.level = grpc_log.level();
    ros_log.node_name = grpc_log.node_name();
    ros_log.message = grpc_log.message();

    ros_log.header.seq = grpc_log.header().seq();
    ros_log.header.stamp.sec = grpc_log.header().stamp().sec();
    ros_log.header.stamp.nanosec = grpc_log.header().stamp().usec() * 1000;

    ros_log_pub->publish(ros_log);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_ros_bridge");
    RCLCPP_INFO(node->get_logger(), "Corgi ROS Bridge Starts");

    bool debug_mode = false;
    if (argc >= 2 && argv[1] != nullptr) {
        if (strcmp(argv[1], "log") == 0) {
            debug_mode = true;
        }
    }

    auto ros_motor_cmd_sub = node->create_subscription<corgi_msgs::msg::MotorCmdStamped>("motor/command", 1, ros_motor_cmd_cb);
    auto ros_steer_cmd_sub = node->create_subscription<corgi_msgs::msg::SteeringCmdStamped>("steer/command", 1, ros_steer_cmd_cb);
    auto ros_robot_cmd_sub = node->create_subscription<corgi_msgs::msg::RobotCmdStamped>("robot/command", 1, ros_robot_cmd_cb);
    auto ros_config_cmd_sub = node->create_subscription<corgi_msgs::msg::ConfigStamped>("config/command", 1, ros_config_cmd_cb);
    ros_motor_state_pub = node->create_publisher<corgi_msgs::msg::MotorStateStamped>("motor/state", 1);
    ros_power_state_pub = node->create_publisher<corgi_msgs::msg::PowerStateStamped>("power/state", 1);
    ros_steer_state_pub = node->create_publisher<corgi_msgs::msg::SteeringStateStamped>("steer/state", 1);
    ros_robot_state_pub = node->create_publisher<corgi_msgs::msg::RobotStateStamped>("robot/state", 1);
    ros_config_state_pub = node->create_publisher<corgi_msgs::msg::ConfigStamped>("config/state", 1);
    ros_log_pub = node->create_publisher<corgi_msgs::msg::LogStamped>("log", 1);

    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStateStamped> &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>("motor/state", 1000, grpc_motor_state_cb);
    core::Subscriber<power_msg::PowerStateStamped> &grpc_power_state_sub = nh_.subscribe<power_msg::PowerStateStamped>("power/state", 1000, grpc_power_state_cb);
    core::Subscriber<steering_msg::SteeringStateStamped> &grpc_steer_state_sub = nh_.subscribe<steering_msg::SteeringStateStamped>("steer/state", 1000, grpc_steer_state_cb);
    core::Subscriber<robot_msg::RobotStateStamped> &grpc_robot_state_sub = nh_.subscribe<robot_msg::RobotStateStamped>("robot/state", 1000, grpc_robot_state_cb);
    core::Subscriber<config_msg::ConfigStamped> &grpc_config_state_sub = nh_.subscribe<config_msg::ConfigStamped>("motor/config/reply", 1000, grpc_config_state_cb, 100);
    core::Subscriber<log_msg::LogEntry> &grpc_log_sub = nh_.subscribe<log_msg::LogEntry>("/log", 100, grpc_log_cb, 100);
    grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));
    grpc_steer_cmd_pub = &(nh_.advertise<steering_msg::SteeringCmdStamped>("steer/command"));
    grpc_robot_cmd_pub = &(nh_.advertise<robot_msg::RobotCmdStamped>("robot/command"));
    grpc_config_cmd_pub = &(nh_.advertise<config_msg::ConfigStamped>("motor/config/request"));

    core::Rate rate(1000);

    int loop_counter = 0;
    while (rclcpp::ok()) {
        if (debug_mode) RCLCPP_INFO(node->get_logger(), "Loop Count: %d", loop_counter);

        rclcpp::spin_some(node);
        core::spinOnce();

        if (debug_mode) RCLCPP_INFO(node->get_logger()," ");

        loop_counter++;
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger("CorgiRosBridge"), "Corgi ROS Bridge is killed");

    rclcpp::shutdown();
    
    return 0;
}
