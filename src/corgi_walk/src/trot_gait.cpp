#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <chrono>

#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>
#include "trot_gait.hpp"
#include "leg_model.hpp"
#include "bezier.hpp"

TrotGait::TrotGait(bool sim, double CoM_bias, int rate, double BL, double BW, double BH) : 
    /* Initializer List */
    leg_model(sim),
    CoM_bias(CoM_bias),
    rate(rate), 
    BL(BL),
    BW(BW),
    BH(BH)
{
    // Initial dS & incre_duty
    dS = velocity / rate;
    incre_duty = dS / step_length;
}//end TrotGait

void TrotGait::initialize(double init_eta[8]) {
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4]  = {-init_eta[1], init_eta[3], init_eta[5], -init_eta[7]};
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    int current_rim = 0;
    for (int i=0; i<4; i++) {
        leg_model.contact_map(init_theta[i], init_beta[i]);
        current_rim = leg_model.rim;
        leg_model.forward(init_theta[i], init_beta[i]);
        if (current_rim == 1) {
            relative_foothold[i][0] = leg_model.U_l[0];
        } else if (current_rim == 2) {
            relative_foothold[i][0] = leg_model.L_l[0];
        } else if (current_rim == 3) {
            relative_foothold[i][0] = leg_model.G[0];
        } else if (current_rim == 4) {
            relative_foothold[i][0] = leg_model.L_r[0];
        } else if (current_rim == 5) {
            relative_foothold[i][0] = leg_model.U_r[0];
        } else {
            std::cout << "Leg cannot contact ground if use the given initial theta/beta." << std::endl;
        }//end if else
        relative_foothold[i][1] = -stand_height;
    }//end for
    // Get initial leg duty  
    int first_swing_leg = 0;
    for (int i=1; i<4; i++) {
        if (relative_foothold[i][0] < relative_foothold[first_swing_leg][0]) {
            first_swing_leg = i;
        }//end if
    }//end for 
    if (first_swing_leg == 0) {
        duty = {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
    } else if (first_swing_leg == 1) {
        duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
    } else if (first_swing_leg == 2) {
        duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
    } else if (first_swing_leg == 3) {
        duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
    }//end if else
    // Get foothold in world coordinate
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;
    // Initial leg configuration
    for (int i=0; i<4; i++) {
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0], next_hip[i][1] + relative_foothold[i][1]};
    }//end for
    // Initial theta/beta
    for (int i=0; i<4; i++) {
        theta[i] = init_theta[i];
        beta[i]  = init_beta[i];
    }//end for
}//end initialize

std::array<std::array<double, 4>, 2> TrotGait::step() {
    touchdown = false;
    for (int i=0; i<4; i++) {
        next_hip[i][0] += dS + sign_diff[i]*diff_dS;
        duty[i] += incre_duty;
    }//end for 
    for (int i=0; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (duty[i] < 0){
            duty[i] += 1.0;
        }//end if
        /* Calculate next foothold if entering swing phase */
        if ((duty[i] > (1 - swing_time)) && swing_phase[i] == 0) {
            swing_phase[i] = 1;
            double total_step_length; // step length considering differential
            double swing_hip_move_d; // hip moving distance during swing phase 
            // change to new step length when front leg start to swing
            if ( ((direction == 1) && (i==0 || i==1)) || ((direction == -1) && (i==2 || i==3)) ) {  // front leg swing
                // apply new step length and differential
                next_step_length[i] = new_step_length;   
                double rest_time = (1.0 - 4*swing_time) / 2;    // time during swing of front leg and next hind leg 
                total_step_length = step_length + sign_diff[i]*diff_step_length;
                swing_hip_move_d = direction * swing_time * total_step_length;
                foothold[i] = {next_hip[i][0] + direction*((1-swing_time)/2)*(new_step_length + sign_diff[i]*new_diff_step_length) + swing_hip_move_d + direction*(rest_time*(step_length - new_step_length)) + CoM_bias, 0};    // half distance between leave and touch-down position (in hip coordinate) + distance hip traveled during swing phase + hip travel difference during rest time because different incre_duty caused by change of step length + CoM_bias.
                diff_step_length = new_diff_step_length;
            } else {    // hind leg swing
                int last_leg = (i+2) % 4;   // Contralateral front leg
                step_length = current_step_length[last_leg];
                next_step_length[i] = step_length;    // apply hind step length corresponding to the front leg's.
                total_step_length = step_length + sign_diff[i]*diff_step_length;
                swing_hip_move_d = direction * swing_time * total_step_length;
                foothold[i] = {next_hip[i][0] + direction*((1-swing_time)/2)*total_step_length + swing_hip_move_d + CoM_bias, 0};
                incre_duty = dS / step_length;  // change incre_duty corresponding to new step length when hind leg start to swing.
            }//end if else
            /* Bezier curve setup */
            leg_model.forward(theta[i], beta[i]);
            p_lo = {next_hip[i][0] + leg_model.G[0], next_hip[i][1] + leg_model.G[1]};
            // calculate contact rim when touch ground
            for (int j=0; j<5; j++) {   // G, L_l, U_l
                double contact_height = j==0? leg_model.r : leg_model.radius;
                std::array<double, 2> contact_point = {foothold[i][0] - (next_hip[i][0] + swing_hip_move_d), -stand_height+contact_height};
                result_eta = leg_model.inverse(contact_point, touch_rim_list[j]);
                leg_model.contact_map(result_eta[0], result_eta[1]);
                if (leg_model.rim == touch_rim_idx[j]) {
                    current_rim = leg_model.rim;
                    break;
                }//end if
            }//end for
            // G position when touch ground
            leg_model.forward(result_eta[0], result_eta[1]);
            if (current_rim == 3) {   // G
                p_td = {foothold[i][0], foothold[i][1] + leg_model.r};
            } else if (current_rim == 2) {  // L_l
                p_td = {foothold[i][0] + leg_model.G[0]-leg_model.L_l[0], foothold[i][1] + leg_model.G[1]-leg_model.L_l[1] + leg_model.radius};
            } else if (current_rim == 4) {  // L_r
                p_td = {foothold[i][0] + leg_model.G[0]-leg_model.L_r[0], foothold[i][1] + leg_model.G[1]-leg_model.L_r[1] + leg_model.radius};
            } else if (current_rim == 1) {  // U_l
                p_td = {foothold[i][0] + leg_model.G[0]-leg_model.U_l[0], foothold[i][1] + leg_model.G[1]-leg_model.U_l[1] + leg_model.radius};
            } else if (current_rim == 5) {  // U_r
                p_td = {foothold[i][0] + leg_model.G[0]-leg_model.U_r[0], foothold[i][1] + leg_model.G[1]-leg_model.U_r[1] + leg_model.radius};
            }//end if else
            sp[i] = SwingProfile(p_lo, p_td, step_height, direction);
        } else if ((direction == 1) && (duty[i] > 1.0)) {                  // entering stance phase when velocirty > 0
            touchdown = true;
            swing_phase[i] = 0;
            duty[i] -= 1.0; // Keep duty in the range [0, 1]
            if (sp[i].getDirection() == direction){ // if the leg swing a whole swing phase, instead of swing back.
                step_count[i] += 1;
                current_step_length[i] = next_step_length[i];  
            }//end if
        } else if ((direction == -1) && (duty[i] < (1.0-swing_time))) {    // entering stance phase when velocirty < 0
            touchdown = true;
            swing_phase[i] = 0;
            if (sp[i].getDirection() == direction){ // if the leg swing a whole swing phase, instead of swing back.
                step_count[i] -= 1;
                current_step_length[i] = next_step_length[i];   
            }//end if
        }//end if else
        /* Calculate next theta, beta */
        if (swing_phase[i] == 0) { // Stance phase
            result_eta = leg_model.move(theta[i], beta[i], {next_hip[i][0]-hip[i][0], next_hip[i][1]-hip[i][1]});
        } else { // Swing phase
            if ( sp[i].getDirection()==1 ) {    // direction == 1
                swing_phase_ratio = (duty[i] - (1 - swing_time)) / swing_time;
            } else {    // direction == -1
                swing_phase_ratio = (1.0 - duty[i]) / swing_time;
            }//end if else
            curve_point_temp = sp[i].getFootendPoint(swing_phase_ratio);
            std::array<double, 2> curve_point = {curve_point_temp[0] - next_hip[i][0], curve_point_temp[1] - next_hip[i][1]};
            result_eta = leg_model.inverse(curve_point, "G");
        }//end if else
        theta[i] = result_eta[0];
        beta[i] = result_eta[1];
        hip[i] = next_hip[i];
    }//end for
    return {theta, beta};
}//end step

void TrotGait::set_velocity(double new_value){
    if (std::abs(new_value) > 1.0) {
        throw std::runtime_error("Velocity should not exceed 1.0 m/s.");
    }//end if
    velocity = new_value;
    dS = velocity / rate;
    incre_duty = dS / step_length;
    direction = velocity>=0? 1 : -1;
    // change differential dS if the robot is turning
    if (curvature != 0.0) {
        diff_dS = dS * (outer_radius - inner_radius) / (outer_radius + inner_radius);  // apply new value immediately
    }//end if 
}//end set_velocity

void TrotGait::set_stand_height(double new_value){
    if (new_value > 0.34 || new_value < 0.12+step_height) {
        throw std::runtime_error("Stand height should be between 0.12+\"step_height\" and 0.34.");
    }//end if
    stand_height = new_value;
    for (int i=0; i<4; i++) {
        next_hip[i][1] = stand_height;
    }//end for
}//end set_stand_height

void TrotGait::set_step_length(double new_value){
    if (new_value <= 0.0) {
        throw std::runtime_error("Step length should be larger than zero.");
    }//end if
    new_step_length = new_value;
    // change differential step length if the robot is turning
    if (curvature != 0.0) {
        new_diff_step_length = new_step_length * (outer_radius - inner_radius) / (outer_radius + inner_radius);    // apply new value when front leg swing
    }//end if 
}//end set_step_length

void TrotGait::set_step_height(double new_value){
    if (new_value <= 0.0) {
        throw std::runtime_error("Step height should be larger than zero.");
    }//end if
    step_height = new_value;
}//end set_step_height

void TrotGait::set_curvature(double new_value){
    curvature = new_value;    
    if (curvature == 0.0) {
        new_diff_step_length = 0.0;
        diff_dS = 0.0;
    } else {
        double turn_radius = 1.0 / std::abs(curvature);
        outer_radius = turn_radius + BW/2.0;
        inner_radius = turn_radius - BW/2.0;
        /*
        step_length + d : step_length - d  = outer_radius : inner_radius
        (step_length - d) * outer_radius = (step_length + d) * inner_radius
        d * (outer_radius + inner_radius) = step_length * (outer_radius - inner_radius)
        */
        new_diff_step_length = new_step_length * (outer_radius - inner_radius) / (outer_radius + inner_radius);    // apply new value when front leg swing
        diff_dS = dS * (outer_radius - inner_radius) / (outer_radius + inner_radius);  // apply new value immediately
        // determine increase/decrease of differential according to sign of curvature and left/right leg 
        if (curvature > 0.0) { // turn left
            sign_diff[0] = -1;
            sign_diff[1] = 1;
            sign_diff[2] = 1;
            sign_diff[3] = -1;
        } else { // turn right
            sign_diff[0] = 1;
            sign_diff[1] = -1;
            sign_diff[2] = -1;
            sign_diff[3] = 1;
        }//end if else
    }// end if else
}//end set_curvature

void TrotGait::set_eta(std::array<std::array<double, 4>, 2> eta_) {
    this->theta = eta_[0];
    this->beta  = eta_[1];
}//end set_eta

void TrotGait::set_duty(std::array<double, 4> duty_) {
    for (int i=0; i<4; i++) {
        if (duty_[i] < 0 || duty_[i] > 1) {
            throw std::runtime_error("Duty should be in the range [0, 1].");
        }//end if
    }//end for
    this->duty = duty_;
}//end set_duty

std::array<int, 4> TrotGait::get_step_count() {
    return this->step_count;
}//end get_step_count

std::array<int, 4> TrotGait::get_swing_phase() {
    return this->swing_phase;
}//end get_step_count

std::array<double, 4> TrotGait::get_duty() {
    return this->duty;
}//end get_duty

bool TrotGait::if_touchdown() {
    return this->touchdown;
}//end if_touchdown