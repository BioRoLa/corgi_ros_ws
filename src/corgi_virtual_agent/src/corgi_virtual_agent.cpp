#include <iostream>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"
#include "Steering.pb.h"


std::mutex mutex_motor_state;
std::mutex mutex_power_state;
std::mutex mutex_steer_state;

motor_msg::MotorCmdStamped          motor_cmd;
power_msg::PowerCmdStamped          power_cmd;
steering_msg::SteeringCmdStamped    steer_cmd;
motor_msg::MotorStateStamped        motor_state;
power_msg::PowerStateStamped        power_state;
steering_msg::SteeringStateStamped  steer_state;


void motor_cmd_cb(const motor_msg::MotorCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_motor_state);

    std::vector<motor_msg::MotorState*> motor_states = {
        motor_state.mutable_module_a(),
        motor_state.mutable_module_b(),
        motor_state.mutable_module_c(),
        motor_state.mutable_module_d()
    };

    std::vector<const motor_msg::MotorCmd*> motor_cmds = {
        &cmd.module_a(),
        &cmd.module_b(),
        &cmd.module_c(),
        &cmd.module_d()
    };

    std::cout << "TB_A: (" << motor_cmds[0]->theta() << ", " << motor_cmds[0]->beta() << "); " << std::endl
              << "TB_B: (" << motor_cmds[1]->theta() << ", " << motor_cmds[1]->beta() << "); " << std::endl
              << "TB_C: (" << motor_cmds[2]->theta() << ", " << motor_cmds[2]->beta() << "); " << std::endl
              << "TB_D: (" << motor_cmds[3]->theta() << ", " << motor_cmds[3]->beta() << "); " << std::endl << std::endl;

    for (int i = 0; i < 4; i++) {
        motor_states[i]->set_theta(motor_cmds[i]->theta());
        motor_states[i]->set_beta(motor_cmds[i]->beta());
        motor_states[i]->set_velocity_r(1);
        motor_states[i]->set_velocity_l(1);
        motor_states[i]->set_torque_r(1);
        motor_states[i]->set_torque_l(1);
    }

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    motor_state.mutable_header()->set_seq(cmd.header().seq());
    motor_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    motor_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}

void power_cmd_cb(const power_msg::PowerCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_power_state);

    power_state.set_digital(cmd.digital());
    power_state.set_signal(cmd.signal());
    power_state.set_power(cmd.power());
    power_state.set_robot_mode((power_msg::ROBOTMODE)cmd.robot_mode());


    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    power_state.mutable_header()->set_seq(cmd.header().seq());
    power_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    power_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}

void steer_cmd_cb(const steering_msg::SteeringCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_steer_state);

    steer_state.set_current_angle(cmd.angle());
    steer_state.set_current_state(cmd.voltage());

    std::cout << cmd.angle() << std::endl;

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    steer_state.mutable_header()->set_seq(cmd.header().seq());
    steer_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    steer_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_virtual_agent");

    core::NodeHandler nh_;
    core::Publisher<motor_msg::MotorStateStamped> &motor_state_pub = nh_.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Publisher<power_msg::PowerStateStamped> &power_state_pub = nh_.advertise<power_msg::PowerStateStamped>("power/state");
    core::Publisher<steering_msg::SteeringStateStamped> &steer_state_pub = nh_.advertise<steering_msg::SteeringStateStamped>("steer/state");
    core::Subscriber<motor_msg::MotorCmdStamped> &motor_cmd_sub = nh_.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);
    core::Subscriber<power_msg::PowerCmdStamped> &power_cmd_sub = nh_.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_cmd_cb);
    core::Subscriber<steering_msg::SteeringCmdStamped> &steer_cmd_sub = nh_.subscribe<steering_msg::SteeringCmdStamped>("steer/command", 1000, steer_cmd_cb);

    core::Rate rate(1000);

    while (rclcpp::ok()) {
        core::spinOnce();

        {
            std::lock_guard<std::mutex> lock(mutex_motor_state);
            motor_state_pub.publish(motor_state);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_power_state);
            power_state_pub.publish(power_state);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_steer_state);
            steer_state_pub.publish(steer_state);
        }

        rate.sleep();
    }

    rclcpp::shutdown();


    return 0;
}