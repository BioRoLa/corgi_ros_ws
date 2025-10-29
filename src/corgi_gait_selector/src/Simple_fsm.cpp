#include "Simple_fsm.hpp"

GaitSelector::GaitSelector( rclcpp::Node& nh, 
                            bool sim, 
                            double CoM_bias, 
                            int pub_rate, 
                            double BL, 
                            double BW, 
                            double BH): 
    leg_model(sim), 
    CoM_bias(CoM_bias), 
    BL(BL), 
    BW(BW), 
    BH(BH), 
    pub_rate(pub_rate),
    rng(rd()), 
    dist(0, 359),
    currentGait(Gait::WHEELED)
{
    motor_state_sub_ = nh.subscribe("/motor/state", 1000, &GaitSelector::motor_state_cb, this);
    motor_cmd_pub_ = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("/motor/command", pub_rate);
    marker_pub_ = nh.advertise<visualization_msgs::msg::Marker>("stable_triangle", pub_rate);
    trigger_sub_ = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1000, &GaitSelector::trigger_cb, this);
    rate_ptr = new rclcpp::Rate(pub_rate);

    // Initialize dS & incre_duty
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;  

    // in leg frame
    for (int i = 0; i < 4; ++i) {
        relative_foothold[i][0] = -stand_height;
        relative_foothold[i][1] = 0;
    }

    // in robot frame
    body = {0, 0, stand_height};
    hip = {{{body[0]+BL/2, body[1]+BW/2, body[2]},
            {body[0]+BL/2, body[1]-BW/2, body[2]},
            {body[0]-BL/2, body[1]-BW/2, body[2]},
            {body[0]-BL/2, body[1]+BW/2, body[2]}}};

    foothold = {{{hip[0][0] - relative_foothold[0][1], hip[0][1], hip[0][2] + relative_foothold[0][0]},
                 {hip[1][0] - relative_foothold[1][1], hip[1][1], hip[1][2] + relative_foothold[1][0]},
                 {hip[2][0] - relative_foothold[2][1], hip[2][1], hip[2][2] + relative_foothold[2][0]},
                 {hip[3][0] - relative_foothold[3][1], hip[3][1], hip[3][2] + relative_foothold[3][0]}}};

    

    next_body = body;
    next_hip = hip;
    next_foothold = foothold;


}

GaitSelector::~GaitSelector() {
    delete rate_ptr;
    rate_ptr = nullptr;
}


void GaitSelector::Send(){
    for(int i =0; i<4; i++){
        if (i==0 || i==3) {
            motor_cmd_modules[i]->beta  = -next_eta[i][1];            
        } else {
            motor_cmd_modules[i]->beta  =  next_eta[i][1];
        }     
        motor_cmd_modules[i]->theta = next_eta[i][0];
        motor_cmd_modules[i]->kp_r = 150;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 150;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        // Update eta to next_eta
        for(int j = 0; j < 2; j++) {
            eta[i][j] = next_eta[i][j];
        }
    }
    motor_cmd_pub_.publish(motor_cmd);
    rate_ptr->sleep();
}

void GaitSelector::motor_state_cb(const corgi_msgs::msg::MotorStateStamped state){
    motor_state = state;
}  

void GaitSelector::trigger_cb(const corgi_msgs::msg::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb


void GaitSelector::Transfer(int transfer_sec, int wait_sec){
    // next_eta = target and grep current_motor_pose -> devided to step until current_eta
    // transfer
    std::vector<std::vector<double>> theta_steps(4), beta_steps(4);
    for (int i=0; i<4; i++){
        theta_steps[i] = linspace(motor_state_modules[i]->theta, next_eta[i][0], transfer_sec*pub_rate);
        beta_steps[i]  = linspace(motor_state_modules[i]->beta,  next_eta[i][1], transfer_sec*pub_rate);
    }
    for (int step_i = 0; step_i < transfer_sec*pub_rate; step_i++) {
        for  (int i=0; i<4; i++){
            next_eta[i][0] = theta_steps[i][step_i];
            next_eta[i][1] = beta_steps[i][step_i];
        }       
        Send();
    }

    // wait
    for (int step_i = 0; step_i < wait_sec*pub_rate; step_i++) {
        Send();
    }

}

void GaitSelector::Receive(){
    for (int i=0; i<4; i++){
        std::cout << i << std::endl;
        std::cout << motor_state_modules[i]->theta << std::endl;
        std::cout << motor_state_modules[i]->beta << std::endl;
        next_eta[i][0] = motor_state_modules[i]->theta;
        if (i==0 || i==3) {
            next_eta[i][1] = -motor_state_modules[i]->beta;   
        } else {
            next_eta[i][1] = motor_state_modules[i]->beta;
        }   
        
    }
    Send();
}

std::vector<double> GaitSelector::linspace(double start, double end, int num_steps) {
    std::vector<double> result;
    if (num_steps < 1) return result; 
    
    result.resize(num_steps);
    if (num_steps == 1) {
        // Only one step -> just start
        result[0] = start;
        return result;
    }
    
    double step = (end - start) / (num_steps - 1);
    for (int i = 0; i < num_steps; ++i) {
        result[i] = start + step * i;
    }
    return result;
}

// Initialize static variables
std::array<double, 4> GaitSelector::duty = {0.0};
std::array<int, 4> GaitSelector::swing_phase = {0};

double GaitSelector::swing_time = 0.2;   
double GaitSelector::velocity = 0.1;  
double GaitSelector::stand_height = 0.17;
double GaitSelector::step_length = 0.5; 
double GaitSelector::step_height = 0.03; 

double GaitSelector::curvature = 0.0; // +: turn left, -:turn right, 0: straight

std::array<double, 4> GaitSelector::current_step_length = {step_length, step_length, step_length, step_length};
std::array<double, 4> GaitSelector::next_step_length    = {step_length, step_length, step_length, step_length};
double GaitSelector::new_step_length = step_length;

std::array<double, 4> GaitSelector::current_stand_height = {stand_height, stand_height, stand_height, stand_height};
std::array<double, 4> GaitSelector::next_stand_height    = {stand_height, stand_height, stand_height, stand_height};
double GaitSelector::new_stand_height = stand_height;

std::array<double, 4> GaitSelector::current_shift = {0.0, 0.0, 0.0, 0.0}; 
double GaitSelector::relative_foothold[4][2] = {0.0};
double GaitSelector::eta[4][2] = {0.0};
double GaitSelector::next_eta[4][2]= {0.0};
std::array<std::array<double, 3>, 4> GaitSelector::foothold= {0.0};
std::array<std::array<double, 3>, 4> GaitSelector::next_foothold=foothold;
std::array<double, 3>GaitSelector::body= {0.0};
std::array<double, 3>GaitSelector::next_body = body;
std::array<std::array<double, 3>, 4> GaitSelector::hip = {0.0};
std::array<std::array<double, 3>, 4> GaitSelector::next_hip = hip;

// For turning 
double GaitSelector::outer_radius = 0.0;
double GaitSelector::inner_radius = 0.0;
double GaitSelector::diff_step_length = 0.0;  // Differential step length 
double GaitSelector::new_diff_step_length = 0.0;  // New differential step length
double GaitSelector::diff_dS = 0.0;   // Differential dS
int GaitSelector::sign_diff[4] = {0.0};   // Differential sign

corgi_msgs::msg::MotorStateStamped GaitSelector::motor_state = corgi_msgs::msg::MotorStateStamped();

std::vector<corgi_msgs::msg::MotorState*> GaitSelector::motor_state_modules = {
    &GaitSelector::motor_state.module_a,
    &GaitSelector::motor_state.module_b,
    &GaitSelector::motor_state.module_c,
    &GaitSelector::motor_state.module_d
};

int GaitSelector::pub_time = 1;
int GaitSelector::do_pub = 1;
int GaitSelector::transfer_state = 0;
int GaitSelector::transfer_sec = 5;
int GaitSelector::wait_sec = 3;

int GaitSelector::direction = 1;

    

// add statecallback
//  bool sim=true,
// double CoM_bias=0.0,
// int pub_rate=1000,
// double BL=0.444,
// double BW=0.4,
// double BH=0.2




