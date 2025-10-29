#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <corgi_msgs/msg/motor_state.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>

// Global shared variables and a mutex for thread safety
std::mutex input_mutex;
int direction = 0;      // Active direction: -1 (CCW), 0 (paused), 1 (CW)
int last_direction = 0; // Stores the last non-zero direction
double velocity = 0.5;  // Velocity value

// Global messages for motor state and command
corgi_msgs::msg::MotorStateStamped current_motor_state_;
corgi_msgs::msg::MotorCmdStamped current_motor_cmd_;

// Create vectors of pointers to each module
std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
    &current_motor_state_.module_a,
    &current_motor_state_.module_b,
    &current_motor_state_.module_c,
    &current_motor_state_.module_d
};

std::vector<corgi_msgs::msg::MotorCmd*> motor_cmds = {
    &current_motor_cmd_.module_a,
    &current_motor_cmd_.module_b,
    &current_motor_cmd_.module_c,
    &current_motor_cmd_.module_d
};

// Callback to update the current motor state
void motorsStateCallback(const corgi_msgs::msg::MotorStateStamped::ConstSharedPtr& msg){
    current_motor_state_ = *msg;
}

// This thread continuously reads keyboard input and updates shared variables.
void keyboardInputThread() {
    while (rclcpp::ok()) {
        char input_char;
        std::cout << "[d] Direction; [v] Velocity; [p] Pause; [r] Resume; [q] Quit: ";
        std::cin >> input_char;
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            switch (input_char) {
                case 'd': {
                    int dir;
                    std::cout << "Enter direction (-1 for CCW, 1 for CW): ";
                    std::cin >> dir;
                    if (dir == -1 || dir == 1) {
                        direction = dir;
                        last_direction = dir;  // Store the last valid direction
                        std::cout << (dir == -1 ? "CCW" : "CW") << std::endl;
                    } else {
                        std::cout << "Invalid command." << std::endl;
                    }
                    break;
                }
                case 'v': {
                    double new_velocity;
                    std::cout << "Enter new velocity (positive value): ";
                    std::cin >> new_velocity;
                    if (new_velocity >= 0) {
                        velocity = new_velocity;
                        std::cout << "Velocity updated to " << velocity << std::endl;
                    } else {
                        std::cout << "Invalid velocity. Must be non-negative." << std::endl;
                    }
                    break;
                }
                case 'p':
                    std::cout << "Paused" << std::endl;
                    direction = 0;  // Pause by setting active direction to zero
                    break;
                case 'r': {
                    if (last_direction != 0) {
                        direction = last_direction;  // Resume with the last known direction
                        std::cout << "Resumed with direction " << (direction == -1 ? "CCW" : "CW") << std::endl;
                    } else {
                        std::cout << "No valid previous direction found. Please set direction first using 'd'." << std::endl;
                    }
                    break;
                }
                case 'q':
                    std::cout << "Quitting..." << std::endl;
                    ros::shutdown();
                    break;
                default:
                    std::cout << "Unrecognized input, keeping previous settings." << std::endl;
                    break;
            }
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("test");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto motor_cmd_pub_ = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("/motor/command", 1000);
    auto motor_state_sub_ = nh.subscribe("/motor/state", 1000, motorsStateCallback);
    rclcpp::Rate rate(1000);

    // Start the keyboard input thread
    std::thread input_thread(keyboardInputThread);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Get the latest direction and velocity (protected by a mutex)
        int current_direction;
        double current_velocity;
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            current_direction = direction;
            current_velocity = velocity;
        }

        // Calculate beta adjustment based on the current direction and velocity
        double beta_adjustment = (current_velocity / 0.119) * (M_PI / 180.0);
        if (current_direction == -1) {
            beta_adjustment = -beta_adjustment;
        } else if (current_direction == 0) {
            beta_adjustment = 0;
        }

        // Update motor commands for each module
        current_motor_cmd_.header.stamp = rclcpp::Time::now();
        for (int i = 0; i < 4; ++i) {
            motor_cmds[i]->theta = 17 * (M_PI / 180.0);
            motor_cmds[i]->beta = motor_state_modules[i]->beta + beta_adjustment;
            motor_cmds[i]->kp_r = 90;
            motor_cmds[i]->ki_r = 0;
            motor_cmds[i]->kd_r = 1.75;
            motor_cmds[i]->kp_l = 90;
            motor_cmds[i]->ki_l = 0;
            motor_cmds[i]->kd_l = 1.75;
        }

        // Publish the command continuously
        motor_cmd_pub_.publish(current_motor_cmd_);
        rate.sleep();
    }

    // Join the input thread before exiting
    if (input_thread.joinable()) {
        input_thread.join();
    }

    
    return 0;
}
