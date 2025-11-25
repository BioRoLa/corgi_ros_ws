#include <iostream>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"
#include "Steering.pb.h"

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/power_cmd_stamped.hpp>
#include <corgi_msgs/msg/power_state_stamped.hpp>
#include <corgi_msgs/msg/steering_cmd_stamped.hpp>
#include <corgi_msgs/msg/steering_state_stamped.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>


std::mutex mutex_ros_motor_state;
std::mutex mutex_ros_power_state;
std::mutex mutex_ros_steer_state;
std::mutex mutex_grpc_motor_cmd;
std::mutex mutex_grpc_power_cmd;
std::mutex mutex_grpc_steer_cmd;

corgi_msgs::msg::MotorCmdStamped         ros_motor_cmd;
corgi_msgs::msg::PowerCmdStamped         ros_power_cmd;
corgi_msgs::msg::SteeringCmdStamped      ros_steer_cmd;
corgi_msgs::msg::MotorStateStamped       ros_motor_state;
corgi_msgs::msg::PowerStateStamped       ros_power_state;
corgi_msgs::msg::SteeringStateStamped    ros_steer_state;

motor_msg::MotorCmdStamped          grpc_motor_cmd;
power_msg::PowerCmdStamped          grpc_power_cmd;
steering_msg::SteeringCmdStamped    grpc_steer_cmd;
motor_msg::MotorStateStamped        grpc_motor_state;
power_msg::PowerStateStamped        grpc_power_state;
steering_msg::SteeringStateStamped  grpc_steer_state;

rclcpp::Publisher<corgi_msgs::msg::MotorStateStamped>::SharedPtr ros_motor_state_pub; 
rclcpp::Publisher<corgi_msgs::msg::PowerStateStamped>::SharedPtr ros_power_state_pub; 
rclcpp::Publisher<corgi_msgs::msg::SteeringStateStamped>::SharedPtr ros_steer_state_pub;
core::Publisher<motor_msg::MotorCmdStamped>         *grpc_motor_cmd_pub;
core::Publisher<power_msg::PowerCmdStamped>         *grpc_power_cmd_pub;
core::Publisher<steering_msg::SteeringCmdStamped>   *grpc_steer_cmd_pub;

bool ros_trigger = false;


void ros_trigger_cb(const corgi_msgs::msg::TriggerStamped trigger){
    ros_trigger = trigger.enable;
}

void ros_motor_cmd_cb(const corgi_msgs::msg::MotorCmdStamped cmd) {
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
        grpc_motor_modules[i]->set_torque_r(-ros_motor_modules[i].torque_r);
        grpc_motor_modules[i]->set_torque_l(-ros_motor_modules[i].torque_l);
    }

    grpc_motor_cmd.mutable_header()->set_seq(ros_motor_cmd.header.seq);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(ros_motor_cmd.header.stamp.sec);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(ros_motor_cmd.header.stamp.nanosec);

    grpc_motor_cmd_pub->publish(grpc_motor_cmd);
}

void ros_power_cmd_cb(const corgi_msgs::msg::PowerCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);

    ros_power_cmd = cmd;

    grpc_power_cmd.set_digital(ros_power_cmd.digital);
    grpc_power_cmd.set_signal(ros_power_cmd.signal);
    grpc_power_cmd.set_power(ros_power_cmd.power);
    grpc_power_cmd.set_clean(false);
    grpc_power_cmd.set_trigger(ros_trigger);
    grpc_power_cmd.set_robot_mode((power_msg::ROBOTMODE)ros_power_cmd.robot_mode);
    grpc_power_cmd.set_steering_cali(ros_power_cmd.steering_cali);

    grpc_power_cmd.mutable_header()->set_seq(ros_power_cmd.header.seq);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_sec(ros_power_cmd.header.stamp.sec);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_usec(ros_power_cmd.header.stamp.nanosec);
    
    grpc_power_cmd_pub->publish(grpc_power_cmd);
}

void ros_steer_cmd_cb(const corgi_msgs::msg::SteeringCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_grpc_steer_cmd);

    ros_steer_cmd = cmd;

    grpc_steer_cmd.set_voltage(ros_steer_cmd.voltage);
    grpc_steer_cmd.set_angle(ros_steer_cmd.angle);

    grpc_steer_cmd.mutable_header()->set_seq(ros_steer_cmd.header.seq);
    grpc_steer_cmd.mutable_header()->mutable_stamp()->set_sec(ros_steer_cmd.header.stamp.sec);
    grpc_steer_cmd.mutable_header()->mutable_stamp()->set_usec(ros_steer_cmd.header.stamp.nanosec);

    grpc_steer_cmd_pub->publish(grpc_steer_cmd);
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
        ros_motor_modules[i]->torque_r = -grpc_motor_modules[i]->torque_r();
        ros_motor_modules[i]->torque_l = -grpc_motor_modules[i]->torque_l();
    }

    ros_motor_state.header.seq = grpc_motor_state.header().seq();
    ros_motor_state.header.stamp.sec = grpc_motor_state.header().stamp().sec();
    ros_motor_state.header.stamp.nanosec = grpc_motor_state.header().stamp().usec() / 1000;

    ros_motor_state_pub->publish(ros_motor_state);
}

void grpc_power_state_cb(const power_msg::PowerStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_power_state);

    grpc_power_state = state;

    ros_power_state.digital = grpc_power_state.digital();
    ros_power_state.signal = grpc_power_state.signal();
    ros_power_state.power = grpc_power_state.power();
    ros_power_state.clean = grpc_power_state.clean();
    ros_power_state.robot_mode = grpc_power_state.robot_mode();

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
    ros_power_state.header.stamp.nanosec = grpc_power_state.header().stamp().usec() / 1000;

    ros_power_state_pub->publish(ros_power_state);
}

void grpc_steer_state_cb(const steering_msg::SteeringStateStamped state) {
    std::lock_guard<std::mutex> lock(mutex_ros_steer_state);

    grpc_steer_state = state;

    ros_steer_state.current_angle = grpc_steer_state.current_angle();
    ros_steer_state.current_state = grpc_steer_state.current_state();
    ros_steer_state.cmd_finish = grpc_steer_state.cmd_finish();

    ros_steer_state.header.seq = grpc_steer_state.header().seq();
    ros_steer_state.header.stamp.sec = grpc_steer_state.header().stamp().sec();
    ros_steer_state.header.stamp.nanosec = grpc_steer_state.header().stamp().usec() / 1000;

    ros_steer_state_pub->publish(ros_steer_state);
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
    auto ros_power_cmd_sub = node->create_subscription<corgi_msgs::msg::PowerCmdStamped>("power/command", 1, ros_power_cmd_cb);
    auto ros_steer_cmd_sub = node->create_subscription<corgi_msgs::msg::SteeringCmdStamped>("steer/command", 1, ros_steer_cmd_cb);
    auto ros_trigger_sub = node->create_subscription<corgi_msgs::msg::TriggerStamped>("trigger", 1, ros_trigger_cb);
    ros_motor_state_pub = node->create_publisher<corgi_msgs::msg::MotorStateStamped>("motor/state", 1);
    ros_power_state_pub = node->create_publisher<corgi_msgs::msg::PowerStateStamped>("power/state", 1);
    ros_steer_state_pub = node->create_publisher<corgi_msgs::msg::SteeringStateStamped>("steer/state", 1);

    core::NodeHandler nh_;
    core::Subscriber<motor_msg::MotorStateStamped> &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>("motor/state", 1000, grpc_motor_state_cb);
    core::Subscriber<power_msg::PowerStateStamped> &grpc_power_state_sub = nh_.subscribe<power_msg::PowerStateStamped>("power/state", 1000, grpc_power_state_cb);
    core::Subscriber<steering_msg::SteeringStateStamped> &grpc_steer_state_sub = nh_.subscribe<steering_msg::SteeringStateStamped>("steer/state", 1000, grpc_steer_state_cb);
    grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));
    grpc_power_cmd_pub = &(nh_.advertise<power_msg::PowerCmdStamped>("power/command"));
    grpc_steer_cmd_pub = &(nh_.advertise<steering_msg::SteeringCmdStamped>("steer/command"));

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
