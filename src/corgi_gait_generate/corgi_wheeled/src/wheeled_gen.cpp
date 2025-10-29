# include "wheeled_gen.hpp"

/* Wheeled model class */
/* 
mode
    1. FW/BW
    2. L/R
    3. CW/CCW
control method (wheeled_cmd.cpp)
    1. joystick
    2. teleop
    3. pure code 
TDL
    1. add ratio of turning    
*/

Wheeled::Wheeled(rclcpp::Node& nh) : GaitSelector(nh)
{
    // Initialize parameters
    beta_adjustment = 0.0;
    beta_adjustment_l = 0.0;
    beta_adjustment_r = 0.0;

    // Initialize subscribers
    wheel_cmd_sub_ = nh.subscribe("/wheel_cmd", 1000, &Wheeled::wheelCmdCallback, this);
    steer_cmd_sub_ = nh.subscribe("/steer/command", 1000, &Wheeled::steerCmdCallback, this);
}

void Wheeled::wheelCmdCallback(const corgi_msgs::msg::WheelCmd::ConstSharedPtr& msg)
{
    current_wheel_cmd_ = *msg;
    velocity = current_wheel_cmd_.velocity;
    motor_cmd.header.stamp = rclcpp::Time::now();
    // Compute beta adjustments based on whether we're in linear or angular mode.
    if (current_steer_cmd_.angle == 0.0) {
        // Linear motion
        MoveLinear(current_wheel_cmd_.direction);
        // If stop command, force adjustment to zero.
        if (current_wheel_cmd_.stop){
            beta_adjustment = 0;
        }
        // Use the same adjustment for both sides.
        beta_adjustment_l = beta_adjustment_r = beta_adjustment;
    }
    else { 
        // Angular motion: the boolean indicates turning direction.
        MoveAngular(current_steer_cmd_.angle > 0.0);
        if (current_wheel_cmd_.stop)
            beta_adjustment_l = beta_adjustment_r = 0;
    }
    
    // Update all motor commands
    for (int i = 0; i < 4; ++i) {
        motor_cmd_modules[i]->theta = 17 * (M_PI / 180.0);
        // For modules 1 & 2 subtract right adjustment; for 0 & 3 add left adjustment.
        if(current_wheel_cmd_.ground_rotate==0.0){
            motor_cmd_modules[i]->beta = (i == 1 || i == 2) ?
                              (motor_state_modules[i]->beta - beta_adjustment_r) :
                              (motor_state_modules[i]->beta + beta_adjustment_l);
        }
        else{
            //Rotate in place
            motor_cmd_modules[i]->beta = motor_state_modules[i]->beta + beta_adjustment;
        }
        
        // Set PID parameters for both right and left controllers.
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
    }
    
    motor_cmd.header.seq = motor_state.header.seq + 1;
}

void Wheeled::steerCmdCallback(const corgi_msgs::msg::SteeringCmdStamped::ConstSharedPtr& msg)
{
    current_steer_cmd_ = *msg;
}


void Wheeled::MoveLinear(bool dir){
    beta_adjustment = (velocity / 0.119) * (M_PI / 180.0);
    if (!dir){
        //backward
        beta_adjustment = -beta_adjustment; // Reverse adjustment if direction is 0
    }
}

void Wheeled::MoveAngular(bool dir){
    // Determine multipliers based on turn direction (right or left)
    float leftMultiplier  = dir ? 1.5f : 0.5f;
    float rightMultiplier = dir ? 0.5f : 1.5f;

    // Calculate beta adjustments using the multipliers
    beta_adjustment_l = (velocity * leftMultiplier / 0.119f) * (M_PI / 180.0f);
    beta_adjustment_r = (velocity * rightMultiplier / 0.119f) * (M_PI / 180.0f);

    // Reverse adjustments if the direction flag is false
    if (!current_wheel_cmd_.direction) {
        beta_adjustment_l = -beta_adjustment_l;
        beta_adjustment_r = -beta_adjustment_r;
    }

}

void Wheeled::Roll(int pub_time, int do_pub, bool dir, bool ground_rotate, int voltage, float angle){
    if (angle == 0.0){
        MoveLinear(dir);
        beta_adjustment_l = beta_adjustment_r = beta_adjustment;
    }
    else{
        MoveAngular(angle > 0.0);
    }

    // read current steering state(whether pub or not)
    if (current_steering_state_.current_angle!= angle){
        //publish steering command
        steering_cmd_.angle = angle;
        steering_cmd_.voltage = voltage;
    }
    else {
        //publish steering command
        steering_cmd_.voltage = 0;
    }
    
    // Update all motor commands
    for (int i = 0; i < 4; ++i) {
    motor_cmd_modules[i]->theta = 17 * (M_PI / 180.0);
    // For modules 1 & 2 subtract right adjustment; for 0 & 3 add left adjustment.
    if(ground_rotate==0.0){
        motor_cmd_modules[i]->beta = (i == 1 || i == 2) ?
                            (motor_state_modules[i]->beta - beta_adjustment_r) :
                            (motor_state_modules[i]->beta + beta_adjustment_l);
    }
    else{
        //Rotate in place
        motor_cmd_modules[i]->beta = motor_state_modules[i]->beta + beta_adjustment;
    }
    
    // Set PID parameters for both right and left controllers.
    motor_cmd_modules[i]->kp_r = 90;
    motor_cmd_modules[i]->ki_r = 0;
    motor_cmd_modules[i]->kd_r = 1.75;
    motor_cmd_modules[i]->kp_l = 90;
    motor_cmd_modules[i]->ki_l = 0;
    motor_cmd_modules[i]->kd_l = 1.75;
    }
    
    motor_cmd.header.seq = motor_state.header.seq + 1;
    for (int i=0;i<4;i++){
        eta[i][0] = 17*M_PI/180;
        if(i==1 | i==2){
            eta[i][1] = -1 * motor_cmd_modules[i]->beta;
        }
        else{
            eta[i][1] = motor_cmd_modules[i]->beta;
        }
    }   
}


