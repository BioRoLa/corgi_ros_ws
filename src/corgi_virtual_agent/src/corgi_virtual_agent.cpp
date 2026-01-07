#include <iostream>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"
#include "Steering.pb.h"

#include "Config.pb.h"
#include "Robot.pb.h"
#include "Log.pb.h"

std::mutex mutex_motor_state;
std::mutex mutex_power_state;
std::mutex mutex_steer_state;
std::mutex mutex_robot_state;
std::mutex mutex_config_state;
std::mutex mutex_log;

motor_msg::MotorCmdStamped          motor_cmd;
// power_msg::PowerCmdStamped          power_cmd;
robot_msg::RobotCmdStamped          robot_cmd;
steering_msg::SteeringCmdStamped    steer_cmd;
config_msg::ConfigStamped           config_cmd;
motor_msg::MotorStateStamped        motor_state;
power_msg::PowerStateStamped        power_state;
robot_msg::RobotStateStamped        robot_state;
steering_msg::SteeringStateStamped  steer_state;
config_msg::ConfigStamped           config_state;
log_msg::LogEntry                   log_entry;


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

// void power_cmd_cb(const power_msg::PowerCmdStamped cmd) {
//     std::lock_guard<std::mutex> lock(mutex_power_state);

//     power_state.set_digital(cmd.digital());
//     power_state.set_signal(cmd.signal());
//     power_state.set_power(cmd.power());
//     power_state.set_robot_mode((power_msg::ROBOTMODE)cmd.robot_mode());


//     timeval currentTime;
//     gettimeofday(&currentTime, nullptr);
//     power_state.mutable_header()->set_seq(cmd.header().seq());
//     power_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
//     power_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
// }

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

void robot_cmd_cb(const robot_msg::RobotCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_robot_state);

    std::cout << "Received robot command! Mode: " << cmd.request_robot_mode() << std::endl;

    robot_state.set_robot_mode((robot_msg::ROBOTMODE)cmd.request_robot_mode());

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    robot_state.mutable_header()->set_seq(cmd.header().seq());
    robot_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    robot_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
    
    std::cout << "Robot state updated. Publishing mode: " << robot_state.robot_mode() << std::endl;
}

void config_cmd_cb(const config_msg::ConfigStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_config_state);

    std::cout << "Received config command! "
              << "Module: " << cmd.module() 
              << ", Motor: " << cmd.motor()
              << ", Mode: " << (cmd.mode() == config_msg::READ ? "READ" : "WRITE")
              << ", Type: " << (cmd.type() == config_msg::INT ? "INT" : "FLOAT")
              << ", Address: " << cmd.address()
              << ", Value_i: " << cmd.value_i()
              << ", Value_f: " << cmd.value_f()
              << std::endl;

    // Echo back the config request as a reply
    config_state.set_transmit(cmd.transmit());
    config_state.set_module(cmd.module());
    config_state.set_motor(cmd.motor());
    config_state.set_mode(cmd.mode());
    config_state.set_type(cmd.type());
    config_state.set_address(cmd.address());
    
    // For simulation, echo back the values or return dummy values
    if (cmd.mode() == config_msg::WRITE) {
        // Echo write values
        config_state.set_value_i(cmd.value_i());
        config_state.set_value_f(cmd.value_f());
    } else {
        // READ mode - return dummy values for simulation
        if (cmd.type() == config_msg::INT) {
            config_state.set_value_i(123);  // Dummy int value
            config_state.set_value_f(0.0);
        } else {
            config_state.set_value_i(0);
            config_state.set_value_f(45.67);  // Dummy float value
        }
    }
    
    config_state.set_error_code(0);  // No error

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    config_state.mutable_header()->set_seq(cmd.header().seq());
    config_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    config_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
    
    std::cout << "Config reply prepared. Error code: " << config_state.error_code() << std::endl;
}

void log_cb(const log_msg::LogEntry entry) {
    std::lock_guard<std::mutex> lock(mutex_log);

    log_entry.set_level(entry.level());
    log_entry.set_node_name(entry.node_name());
    log_entry.set_message(entry.message());

    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    log_entry.mutable_header()->set_seq(entry.header().seq());
    log_entry.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    log_entry.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
    std::cout << "Log Entry - Level: " << log_entry.level() << ", Node: " << log_entry.node_name() << ", Message: " << log_entry.message() << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_virtual_agent");

    core::NodeHandler nh_;
    core::Publisher<motor_msg::MotorStateStamped> &motor_state_pub = nh_.advertise<motor_msg::MotorStateStamped>("motor/state");
    core::Publisher<power_msg::PowerStateStamped> &power_state_pub = nh_.advertise<power_msg::PowerStateStamped>("power/state");
    core::Publisher<robot_msg::RobotStateStamped> &robot_state_pub = nh_.advertise<robot_msg::RobotStateStamped>("robot/state");
    core::Publisher<steering_msg::SteeringStateStamped> &steer_state_pub = nh_.advertise<steering_msg::SteeringStateStamped>("steer/state");
    core::Publisher<config_msg::ConfigStamped> &config_state_pub = nh_.advertise<config_msg::ConfigStamped>("motor/config/reply");
    core::Publisher<log_msg::LogEntry> &log_pub = nh_.advertise<log_msg::LogEntry>("log");

    core::Subscriber<motor_msg::MotorCmdStamped> &motor_cmd_sub = nh_.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);
    // core::Subscriber<power_msg::PowerCmdStamped> &power_cmd_sub = nh_.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_cmd_cb);
    core::Subscriber<steering_msg::SteeringCmdStamped> &steer_cmd_sub = nh_.subscribe<steering_msg::SteeringCmdStamped>("steer/command", 1000, steer_cmd_cb);
    core::Subscriber<robot_msg::RobotCmdStamped> &robot_cmd_sub = nh_.subscribe<robot_msg::RobotCmdStamped>("robot/command", 1000, robot_cmd_cb);
    core::Subscriber<config_msg::ConfigStamped> &config_cmd_sub = nh_.subscribe<config_msg::ConfigStamped>("motor/config/request", 1000, config_cmd_cb);
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
        
        {
            std::lock_guard<std::mutex> lock(mutex_robot_state);
            robot_state_pub.publish(robot_state);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_config_state);
            config_state_pub.publish(config_state);
        }
        
        {
            std::lock_guard<std::mutex> lock(mutex_log);
            log_pub.publish(log_entry);
        }

        rate.sleep();
    }

    rclcpp::shutdown();


    return 0;
}