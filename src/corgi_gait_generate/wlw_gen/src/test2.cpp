#include "test2.hpp"

using namespace std;
using namespace Eigen;
const double PI = M_PI;

WLWGait::WLWGait(rclcpp::Node& nh, bool sim, double CoM_bias, int pub_rate, double BL, double BW, double BH): 
    leg_model(sim), 
    CoM_bias(CoM_bias), 
    BL(BL), 
    BW(BW), 
    BH(BH), 
    pub_rate(pub_rate),
    rng(rd()), 
    dist(0, 359)
{
    motor_pub = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("/motor/command", 1000);
    motor_state_sub_ = nh.subscribe("/motor/state", 1000, &WLWGait::motorsStateCallback, this);
    rate_ptr = new rclcpp::Rate(pub_rate);
    
    // Initialize dS & incre_duty
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;    
}

WLWGait::~WLWGait() {
    delete rate_ptr;
    rate_ptr = nullptr;
}

void WLWGait::motorsStateCallback(const corgi_msgs::msg::MotorStateStamped::ConstSharedPtr& msg)
{
    current_motor_state_ = *msg;
}

void WLWGait::setCmd(std::array<double, 2> send, int index, bool dir) {
    if (dir==true){
        motor_cmd_modules[index]->beta  = -send[1];
    }
    else{
        motor_cmd_modules[index]->beta  = send[1];
    }
    motor_cmd_modules[index]->theta = send[0];
    motor_cmd_modules[index]->kp_r = 150;
    motor_cmd_modules[index]->ki_r = 0;
    motor_cmd_modules[index]->kd_r = 1.75;
    motor_cmd_modules[index]->kp_l = 150;
    motor_cmd_modules[index]->ki_l = 0;
    motor_cmd_modules[index]->kd_l = 1.75;
}

void WLWGait::publish(int freq) {
    for (int i = 0; i < freq; i++) {
        motor_pub.publish(motor_cmd);
        rate_ptr->sleep();
    }
}

std::array<double, 2> WLWGait::find_pose(double height, float shift, float steplength, double slope) {
    
    std::array<double, 2> pose;
    double pos[2] = {0, -height + leg_model.r};
    pose = leg_model.inverse(pos, "G");
    if (steplength >= 0) {
        for (double i = 0; i < shift + steplength; i += 0.001) {
            pose = leg_model.move(pose[0], pose[1], {0.001, 0}, slope);
        }
    } else {
        for (double i = 0; i > shift + steplength; i -= 0.001) {
            pose = leg_model.move(pose[0], pose[1], {-0.001, 0}, slope);
        }
    }
    return pose;
}

void WLWGait::Send(int freq){
    for(int i =0; i<4; i++){
        // std::cout << i << ": " <<current_eta[i][0]*180.0/M_PI << ", "<< current_eta[i][1]*180.0/M_PI << std::endl;
        std::array<double, 2> tmp = { current_eta[i][0], current_eta[i][1] };
        if (i==1 || i==2) {
            setCmd(tmp, i, true);
        } else {
            setCmd(tmp, i, false);
        }     
    }
    publish(freq);
}

void WLWGait::Initialize(int swing_index, int pub_time, int do_pub, int transfer_state, int transfer_sec, int wait_sec, double shift) {
    // 1>3>0>2, swing_index = who swings first
    switch (swing_index) {
        case 0:
        {
            duty = {1 - swing_time, 0.5 - swing_time, 0.5, 0.0};
            break;
        }
        case 1:
        {
            duty = {0.5 - swing_time, 1 - swing_time, 0.0, 0.5};
            break;
        }
        case 2:
        {
            duty = {0.5 - 2 * swing_time, 1 - 2 * swing_time, 1 - swing_time, 0.5 - swing_time};
            break;
        }
        case 3:
        {
            duty = {1 - 2 * swing_time, 0.5 - 2 * swing_time, 0.5 - swing_time, 1 - swing_time};
            break;
        }
        default:
            for (int k=0 ; k<4 ; k++){
                current_eta[k][0] = 0.0; 
                current_eta[k][1] = 0.0;
            }
            break;
    }
    
    for(int i =0; i<4;i++){
        if (i!=swing_index){
            auto tmp0 = find_pose(stand_height, shift, (step_length/2) - (duty[i]/(1-swing_time)) * step_length, 0);
            next_eta[i][0] = tmp0[0];
            next_eta[i][1] = tmp0[1];
        }
        else{
            auto tmp0 = find_pose(stand_height, shift, -(step_length/2), 0);
            next_eta[i][0] = tmp0[0];
            next_eta[i][1] = tmp0[1];
        }
    }
    
    // Get foothold in world coordinate
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;

    // Initial leg configuration
    for (int i=0; i<4; i++) {
        leg_model.contact_map(next_eta[i][0], next_eta[i][1],0);
        leg_model.forward(next_eta[i][0], next_eta[i][1],true);
        relative_foothold[i][0] = leg_model.contact_p[0];
        relative_foothold[i][1] = -stand_height;
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0] + CoM_bias, next_hip[i][1] + relative_foothold[i][1]};

    }  

    if (transfer_state){
        Transfer(transfer_sec, wait_sec, do_pub);
    }
    else{
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                current_eta[i][j] = next_eta[i][j];
            }
        }
        if(do_pub){
            Send(pub_time);
        }
    }
}

void WLWGait::Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg){
    double startX = relative[swing_leg][0];
    double startY = relative[swing_leg][1];

    double endX   = target[0];
    double endY   = target[1];
    while(endY < startY) {
        endY += 2.0 * PI;
    } 
    variation[0] = endX - startX;
    variation[1] = endY - startY;

    target[0] = startX;
    target[1] = startY;    
}

void WLWGait::Swing_step(std::array<double, 2> target, std::array<double, 2> variation, double eta[4][2], int swing_leg, double duty_ratio){
    double ratio = (duty_ratio-(1-swing_time))/swing_time;
    double currentX = target[0] + variation[0] * ratio;
    double currentY = target[1] + variation[1] * ratio;

    current_eta[swing_leg][0] = currentX;
    current_eta[swing_leg][1] = currentY;
}

void WLWGait::Step(int pub_time, int do_pub, double shift){
    for (int i=0; i<4; i++) {
        next_hip[i][0] += dS ;
        duty[i] += incre_duty;     
    }

    for (int i=0; i<4; i++) {
        /* Keep duty in the range [0, 1] */
        if (duty[i] < 0){ duty[i] += 1.0; }

        /* Calculate next foothold if entering swing phase(1) */
        // Enter SW (calculate swing phase traj)

        if ((duty[i] > (1 - swing_time)) && swing_phase[i] == 0) {
            swing_phase[i] = 1;
            // change to new step length when front leg start to swing
            // front leg swing (forward)
            if ( (i==0 || i==1)) {  
                // apply new step length and differential
                next_step_length[i] = new_step_length;   
                double rest_time = (1.0 - 4*swing_time) / 2;
                foothold[i] = {next_hip[i][0] + ((1-swing_time)/2)*(new_step_length ) + (swing_time)*(step_length ) + (rest_time*(step_length - new_step_length)), 0};   
                // half distance between leave and touch-down position (in hip coordinate) + distance hip traveled during swing phase + hip travel difference during rest time because different incre_duty caused by change of step length.
            } 
            // hind leg swing
            else {    
                int last_leg = (i+2) % 4;   // Contralateral front leg 對側
                // apply hind step length corresponding to the front leg's.
                step_length = current_step_length[last_leg];
                next_step_length[i] = step_length;    
                foothold[i] = {next_hip[i][0] + ((1-swing_time)/2+swing_time)*(step_length), 0};
                incre_duty = dS / step_length;  // change incre_duty corresponding to new step length when hind leg start to swing.
            }
            swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
            Swing(current_eta, swing_pose, swing_variation, i);
            
        } 
        // Enter TD
        else if ((duty[i] > 1.0)) {                  
            swing_phase[i] = 0;
            duty[i] -= 1.0; // Keep duty in the range [0, 1]
            current_step_length[i] = next_step_length[i];  
        }

        /* Calculate next eta */
        // calculate the nest Stance phase traj
        if (swing_phase[i] == 0) { 
            leg_model.forward(current_eta[i][0], current_eta[i][1],true);
            std::array<double, 2> result_eta;
            // result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, 0); 
            result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-(next_hip[i][0]-hip[i][0]), next_hip[i][1]-hip[i][1]}, 0);
            current_eta[i][0] = result_eta[0];
            current_eta[i][1] = result_eta[1];
        } 
        // read the next Swing phase traj
        else { 
            Swing_step(swing_pose, swing_variation, current_eta, i, duty[i]);
        }
        // update the hip position
        hip[i] = next_hip[i];
    }
    
    // Send eta 
    if(do_pub){
        Send(pub_time);
    }

}

double WLWGait::closer_beta(double ref_rad, int leg_index)
{
    double val0 = motor_state_modules[leg_index]->beta;
    double delta_beta;
    // current_eta[leg_index][1];
    if (leg_index ==1 | leg_index ==2){
        while (val0 < ref_rad){
            ref_rad -= 2*PI;
        }
        delta_beta = val0 - ref_rad ;
    }
    else{
        while (val0 > ref_rad){
            ref_rad += 2*PI;
        }
        delta_beta = ref_rad - val0;
    }
    return delta_beta;
}

void WLWGait::Transform(int type, int do_pub, int transfer_state, int transfer_sec, int wait_sec, double shift){    
    // tranform according to type (according duty)
    switch (type){
        case 0:
            // wheel to wlw
            for(int i = 0; i < 4; i++){
                int randomDeg = dist(rng); 
                next_eta[i][0] = 17.0 * PI / 180.0;
                // next_eta[i][1] = randomDeg * PI / 180.0;
                next_eta[i][1] = 0* PI / 180.0;
                // cout<<i<<": "<<next_eta[i][1]*180/PI<<endl;
            }

            // transfer to random wheel pose
            if (transfer_state){
                cout<< "Transfer" <<endl;
                Transfer(transfer_sec, wait_sec, do_pub);
            }
            else{
                for(int i=0;i<4;i++){
                    for(int j =0;j<2;j++){
                        current_eta[i][j] =  next_eta[i][j];
                    }
                }
                if(do_pub){
                    Send(200);
                }
            } 
            
            // cout<<"start transform"<<endl;
            // Rotate in Wheel Mode until RF/LF_beta = 45 deg
            wheel_delta_beta = velocity/(leg_model.radius * pub_rate);
            check_beta[0] = closer_beta(-45*PI/180, 0);
            check_beta[1] = closer_beta( 45*PI/180, 1);  
            
            cout<<"0: "<<check_beta[0]*180/PI<<endl;
            cout<<"1: "<<check_beta[1]*180/PI<<endl;
            if(check_beta[0]<= check_beta[1]){
                state = false;
            }
            else{
                state = true;
            }
            body_move_dist = (leg_model.radius * check_beta[state]);
            delta_time_step = int(check_beta[state]/wheel_delta_beta);
            for (int i =0;i<delta_time_step;i++){
                for (int j=0;j<4;j++){
                    current_eta[j][0] = 17 * PI/180;
                    current_eta[j][1] = current_eta[j][1] +  wheel_delta_beta;
                }
                Send(1);
            }

            // Front Transform
            body_angle = asin((stand_height - leg_model.radius) / BL);
            if(state ==0){
                check_beta[state] = closer_beta(body_angle, state);
            }
            else{
                check_beta[state] = closer_beta(-body_angle, state);
            }        
            pos = {0, -stand_height+leg_model.r};
            // (double height, float shift, float steplength, double slope)
            // cout<< (step_length/2) - (0.5/(1-swing_time)) * step_length << endl;
            // cout<< body_angle * 180/PI << endl;
            temp = find_pose(stand_height, shift, (step_length/2) - (0.5/(1-swing_time)) * step_length, -body_angle);
            // temp = find_pose(stand_height, shift, (step_length/2) - (0.5/(1-swing_time)) * step_length, 0);
            target_theta = temp[0];
            cout<< temp[1]* 180/PI << endl;
            if(state ==0){
                check_beta[state] = closer_beta(temp[1], state);
            }
            else{
                check_beta[state] = closer_beta(-temp[1], state);
            }    

            body_move_dist = (leg_model.radius * check_beta[state]);
            
            
            delta_time_step = int(check_beta[state] /wheel_delta_beta);
            target_theta = (target_theta - current_eta[state][0])/delta_time_step;

            pos = {step_length/2, -stand_height+leg_model.r};
            temp = find_pose(stand_height, shift, (step_length/2), -body_angle); 
            // temp = find_pose(stand_height, shift, (step_length/2), 0); 
            if (state ==0){
                check_beta[!state] = closer_beta(-temp[1], !state);
            }
            else{
                check_beta[!state] = closer_beta(temp[1], !state);
            }       

            delta_beta = (check_beta[!state]-wheel_delta_beta*delta_time_step*1/3)/(delta_time_step*2/3);
            delta_theta = (temp[0]-current_eta[!state][0])/(delta_time_step*2/3);

            for (int i =0;i<delta_time_step;i++){
                for (int j=0;j<4;j++){
                    if (j == state){
                        current_eta[j][0] += target_theta;
                        current_eta[j][1] = current_eta[j][1] +  wheel_delta_beta;
                    }
                    else if(j == !state && i>=(delta_time_step*1/3)){
                        current_eta[j][0] += delta_theta;
                        current_eta[j][1] = current_eta[j][1] +  delta_beta;
                    }  
                    else if(j == !state){
                        current_eta[j][0] = 17 * PI/180;
                        current_eta[j][1] = current_eta[j][1] +  wheel_delta_beta;
                    }
                    else{
                        current_eta[j][0] = 17 * PI/180;
                        current_eta[j][1] = current_eta[j][1] +  wheel_delta_beta;
                    }
                }
                Send(1);
            }
          
            // Hybrid mode 
            // find the closest to the initial pose
            if (state ==0){
                duty_temp = {0.5, 0, (0.5-swing_time)*2/3, 0.5+(0.5-swing_time)*2/3};   
            }
            else{
                duty_temp = {0, 0.5 ,0.5+(0.5-swing_time)*2/3, (0.5-swing_time)*2/3};
            }
            // cout << "duty: "<< duty_temp[0] << " , " << duty_temp[1] << " , " << duty_temp[2] << " , " << duty_temp[3] << endl; 
            cout << "still walking!" << endl;
            swing_phase_temp = {0,0,0,0};
            state = 0;
            while(state == false){
                
                for (int i=0; i<4; i++) {
                    duty_temp[i] += incre_duty;     
                }        
                for(int j =2; j<4; j++){
                    if (duty_temp[j] < 0.0){ duty_temp[j] += 1.0; }
                    if ((duty_temp[j] >= (1 - swing_time)) && swing_phase_temp[j] == 0) {
                        swing_phase_temp[j] = 1;
                    } 
                    else if ((duty_temp[j] > 1.0)) {                  
                        swing_phase_temp[j] = 0;
                        duty_temp[j] -= 1.0; 
                    }

                    current_eta[j][0] = 17 * PI/180;
                    current_eta[j][1] = abs(current_eta[j][1]) +  wheel_delta_beta;                
                }
                for (int i=0; i<2; i++) {
                    // cout<< "0: " <<duty_temp[0] << " ;1: "<<duty_temp[1]<<endl; 
                    if (duty_temp[i] < 0){ duty_temp[i] += 1.0; }
                    if ((duty_temp[i] >= (1 - swing_time)) && swing_phase_temp[i] == 0) {
                        swing_phase_temp[i] = 1;
                        swing_pose = find_pose(stand_height, shift, (step_length*3/6), -body_angle);  
                        Swing(current_eta, swing_pose, swing_variation, i);
                    } 
                    else if ((duty_temp[i] > 1.0)) {                  
                        swing_phase_temp[i] = 0;
                        duty_temp[i] -= 1.0; 
                         
                    }
            
                    if (swing_phase_temp[i] == 0) { 
                        leg_model.forward(current_eta[i][0], current_eta[i][1],true);
                        std::array<double, 2> result_eta;
                        result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, -body_angle);
                        current_eta[i][0] = result_eta[0];
                        current_eta[i][1] = result_eta[1];
                    } 
                    // read the next Swing phase traj
                    else { 
                        Swing_step(swing_pose, swing_variation, current_eta, i, duty_temp[i]);
                    }
                }
                // add when the hear leg with extend
                // 1. front legs both not in swing
                // 2. see who reach the ideal one and the other one swing to the position 
                if(swing_phase_temp[0] == 0 && swing_phase_temp[1] == 0 ){
                    if(duty_temp[2]>=duty_temp[3]){
                        // ideal hear pose
                        auto tmp0 = find_pose(stand_height, shift, (step_length/2), 0);
                        next_eta[2][0] = tmp0[0];
                        next_eta[2][1] = tmp0[1];
                        auto tmp1 = find_pose(stand_height, shift, (step_length/2) - (0.5/(1-swing_time)) * step_length, 0);
                        next_eta[3][0] = tmp1[0];
                        next_eta[3][1] = tmp1[1];
                    }
                    else{
                        auto tmp0 = find_pose(stand_height, shift, (step_length/2), 0);
                        next_eta[3][0] = tmp0[0];
                        next_eta[3][1] = tmp0[1];
                        auto tmp1 = find_pose(stand_height, shift, (step_length/2) - (0.5/(1-swing_time)) * step_length, 0);
                        next_eta[2][0] = tmp1[0];
                        next_eta[2][1] = tmp1[1];
                    }                
                    check_beta[2] = closer_beta( next_eta[2][1], 2);
                    check_beta[3] = closer_beta(-next_eta[3][1], 3);
                    for (int i = 0; i<2;i++){
                        if(duty_temp[0]>=duty_temp[1]){
                            if ((leg_model.radius *check_beta[i+2]) < (dS*(0.8-duty_temp[0])/incre_duty)){
                                state = true;
                            }
                        }
                        else{
                            if ((leg_model.radius *check_beta[i+2]) < (dS*(0.8-duty_temp[1])/incre_duty)){
                                state = true;
                            }
                        }
                    }
                }          
                Send(1);
            }
            

            // cout<< "hear_transform" <<endl;
            // cout << "duty: "<< duty_temp[0] << " , " << duty_temp[1] << " , " << duty_temp[2] << " , " << duty_temp[3] << endl; 
            // find the one that is going to change -> decide the #step
            // the other one swing to the ideal position
            // the front leg walk as original
            // calculate ( 30 the same side -)
            if(check_beta[2]<= check_beta[3]){
                state = false;
            }
            else{
                state = true;
            }
            // the hear closer leg [state+2]
            cout << "body_move_dist!" <<body_move_dist<< endl;
            cout << "delta_time_step!"<<delta_time_step << endl;
            body_move_dist = (leg_model.radius * check_beta[state+2]);
            delta_time_step = int(check_beta[state+2]/wheel_delta_beta);
            target_theta = (next_eta[(state)+2][0] - current_eta[state+2][0])/delta_time_step;
            // the hear further leg  [!state+2]
            // 最終要到達的位置
            // double mod1 = fmod(next_eta[(!state) + 2][1], 2 * PI);
            // double mod2 = fmod(current_eta[(!state) + 2][1], 2 * PI);
            // 使用 (value - PI) 的乘積來判斷兩者是否在 PI 的同一側
            if(check_beta[(!state)+2] >= 2*PI - check_beta[(!state)+2]){
                swing_dir = 0;
            }
            else{
                swing_dir = 1;
            }
            // swing_dir = (((fmod(next_eta[(!state) + 2][1], 2 * PI)) - PI) * (fmod(current_eta[(!state) + 2][1], 2 * PI) - PI) >= 0);
            cout<<(!state) + 2<<endl;
            cout<<"ideal beta = "<<fmod(next_eta[(!state) + 2][1], 2 * PI) *180/PI<<endl;
            cout<<"current beta = "<<fmod(current_eta[(!state) + 2][1], 2 * PI) *180/PI<<endl;
            cout<<"Compare result: "<<swing_dir<<endl;

            // if ((duty[(!state)+2]+incre_duty*delta_time_step)>0.8) {
            //     swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
            //     Swing(current_eta, swing_pose, swing_variation, (!state)+2);
            //     cout << "Get in" <<endl;
            //     double ratio_temp = ((duty[(!state)+2]+incre_duty*delta_time_step)-(1-swing_time))/swing_time;
            //     double currentX_temp = swing_pose[0] + swing_variation[0] * ratio_temp;
            //     double currentY_temp = swing_pose[1] + swing_variation[1] * ratio_temp;                
            //     if((!state)+2 ==3){
            //         check_beta[(!state)+2] = closer_beta(-currentY_temp, (!state)+2);
            //     }
            //     else{
            //         check_beta[(!state)+2] = closer_beta( currentY_temp, (!state)+2);
            //     }                
            //     // choose the swing (swing backward/swing forward)
            //     // swing backward
              
            //     double mod1 = fmod(currentY_temp, 2 * PI);
            //     double mod2 = fmod(current_eta[(!state) + 2][1], 2 * PI);
            //     cout<<(!state) + 2<<endl;
            //     cout<<currentY_temp<<endl;
            //     cout<<mod1<<endl;
            //     cout<<current_eta[(!state) + 2][1]<<endl;
            //     cout<<mod2<<endl;

            //     // 使用 (value - PI) 的乘積來判斷兩者是否在 PI 的同一側
            //     swing_dir = ((mod1 - PI) * (mod2 - PI) >= 0);
            // }
            if (swing_dir){
                delta_beta = (check_beta[(!state)+2] -wheel_delta_beta*delta_time_step*1/3)/(delta_time_step*2/3);
                delta_theta = (next_eta[(!state)+2][0]-current_eta[(!state)+2][0])/(delta_time_step*1/3);
            }
            else{
                delta_beta = (2*PI - check_beta[(!state)+2] -wheel_delta_beta*delta_time_step*1/3)/(delta_time_step*2/3);
                delta_theta = (next_eta[(!state)+2][0]-current_eta[(!state)+2][0])/(delta_time_step*1/3);
            }          
           
            for (int j =0;j<delta_time_step;j++){
                // cout << "duty: "<< duty_temp[0] << " , " << duty_temp[1] << " , " << duty_temp[2] << " , " << duty_temp[3] << endl; 
                for (int i=0; i<4; i++) {
                    duty_temp[i] += incre_duty;     
                }
                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (duty_temp[i] < 0){ duty_temp[i] += 1.0; }
                    /* Calculate next foothold if entering swing phase(1) */
                    // Enter SW (calculate swing phase traj)
                    if ((duty_temp[i] >= (1 - swing_time)) && swing_phase_temp[i] == 0) {
                        swing_phase_temp[i] = 1;                    
                    } 
                    // Enter TD
                    else if ((duty_temp[i] > 1.0)) {                  
                        swing_phase_temp[i] = 0;
                        duty_temp[i] -= 1.0; 
                    }
                    // calculate the nest Stance phase traj
                    // front leg  slope -> 0
                    if (i == 0 || i == 1){
                        leg_model.forward(current_eta[i][0], current_eta[i][1],true);
                        std::array<double, 2> result_eta;
                        result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, (-body_angle+body_angle*(j/delta_time_step)));
                        current_eta[i][0] = result_eta[0];
                        current_eta[i][1] = result_eta[1];
                    }
                    // hear leg 
                    else if(i == (state+2)){
                        // closer one
                        current_eta[i][0] += target_theta;
                        current_eta[i][1] = current_eta[i][1] +  wheel_delta_beta;
                    }
                    else{
                        // current_eta[i][0] = current_eta[i][0];
                        // current_eta[i][1] = current_eta[i][1] +  delta_beta;
                        if (swing_dir){
                            if(j>=(delta_time_step*1/3) && j<=(delta_time_step*2/3)){
                                current_eta[i][0] = 17 * PI/180;
                                current_eta[i][1] = current_eta[i][1] +  delta_beta;
                            }
                            else if(j>(delta_time_step*2/3)){
                                current_eta[i][0] += delta_theta;
                                current_eta[i][1] = current_eta[i][1] +  delta_beta;
                            }
                            else{
                                current_eta[i][0] = 17 * PI/180;
                                current_eta[i][1] = current_eta[i][1] + wheel_delta_beta;
                            }
                        }
                        else{
                            if(j>=(delta_time_step*1/3) && j<=(delta_time_step*2/3)){
                                current_eta[i][0] = 17 * PI/180;
                                current_eta[i][1] = current_eta[i][1] -  delta_beta;
                            }
                            else if(j>(delta_time_step*2/3)){
                                current_eta[i][0] += delta_theta;
                                current_eta[i][1] = current_eta[i][1] -  delta_beta;
                            }
                            else{
                                current_eta[i][0] = 17 * PI/180;
                                current_eta[i][1] = current_eta[i][1] - wheel_delta_beta;
                            }
                        }                        
                    }
                
                }
                Send(1);
            }
            for (int i = 0;i<4 ;i++){
                swing_phase[i] =  swing_phase_temp[i];
                duty[i] =   duty_temp[i];
                if (duty[i]>=(1-swing_time) ){
                    swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
                    Swing(current_eta, swing_pose, swing_variation, i);
                }
            }
            break;
        case 1:
            // wlw to wheel
            // variation height to wheel
            cout<<"wlw to wheel"<<endl;
            int check_theta;
            int check_phase;
            
            
            while(check_theta<4){
                check_theta=0;
                check_phase=0;
                for(int i = 0; i < 4; i++){
                    if (swing_phase[i]==0){
                        check_phase++;
                    }

                }
                if(check_phase==4){
                    if(stand_height>=leg_model.radius+0.0001){
                        change_Height(stand_height- 0.0001);
                    }   
                    Step(1, 1, -0.05);
                    for(int i = 0; i < 4; i++){
                        if(current_eta[i][0]<=18*PI/180){
                            check_theta++;
                        }
                    }
                }
                else{
                    Step(1, 1, -0.05);
                }
               
            }
            for(int i=0;i<5000;i++){
                wheel_delta_beta = velocity/(leg_model.radius * pub_rate);
                for(int i = 0; i < 4; i++){
                    current_eta[i][0] = 17.0 * PI / 180.0;
                    current_eta[i][1] = current_eta[i][1]+wheel_delta_beta;
                }
                if(do_pub){
                    Send(1);
                }
            }
            break;            
        case 2:
            // leg to wlw
            cout<<"leg to wlw"<<endl;
            /* Strategy */
            // read current duty and swing_phase
            // if get in swing phase then turn into walk pose 
            check_point=0;
            walk_transform = {0, 0, 0, 0};
            while(check_point<4){
                // cout << "Duty: "<< duty[0] << " , " << duty[1] << " , " << duty[2] << " , " << duty[3] << endl; 
            
                for (int i=0; i<4; i++) {
                    next_hip[i][0] += dS ;
                    duty[i] += incre_duty;     
                }
                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (duty[i] < 0){ duty[i] += 1.0; }
            
                    /* Calculate next foothold if entering swing phase(1) */
                    // Enter SW (calculate swing phase traj)
                    if ((duty[i] >= (1 - swing_time)) && swing_phase[i] == 0) {
                        swing_phase[i] = 1;
                        swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
                        if(walk_transform[i] ==0){
                            if (walk_transform[i] ==0){
                                walk_transform[i] = 1;
                                cout<< i << " get in hybrid swing phase" << endl;
                            }
                            // cout << "swing TD pose: "<< swing_pose_temp[0]*180/PI << " , " << swing_pose_temp[1]*180/PI << endl; 
                            // cout << "origin swing : "<< swing_pose[0]*180/PI << " , " << swing_pose[1]*180/PI << endl; 
                      
                        }
                        Swing(current_eta, swing_pose, swing_variation, i);                 
                    } 
                    // Enter TD
                    else if ((duty[i] > 1.0)) {           
                        swing_phase[i] = 0;
                        duty[i] -= 1.0; // Keep duty in the range [0, 1]
                        if(walk_transform[i]==1){
                            walk_transform[i]=2;
                            check_point++;
                            cout<< i << " start wlw" << endl;
                            cout<< "check_point add " << i << " ,current= " << check_point<< endl;
                            
                        }
                        // else if(walk_transform[i]!=2){
                        //     cout<< i << " still in walk" << endl;  
                        // }
                    }
            
                    /* Calculate next eta */
                    // calculate the nest Stance phase traj
                    if (swing_phase[i] == 0) { 
                            // cout<<"set"<<endl;
                            // leg_model.forward(current_eta[i][0], current_eta[i][1],true);
                            std::array<double, 2> result_eta;
                            result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, 0);
                            // result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {next_hip[i][0]-hip[i][0], next_hip[i][1]-hip[i][1]});
                            // cout<<"here"<<endl;
                            // cout<<result_eta[0]<<endl;
                            // cout<<result_eta[1]<<endl;

                            current_eta[i][0] = result_eta[0];
                            current_eta[i][1] = result_eta[1];                 
                    } 
                    // read the next Swing phase traj
                    else { 
                        Swing_step(swing_pose, swing_variation, current_eta, i, duty[i]);
                        
                    }
                    hip[i] = next_hip[i];
                }
                
                // Send eta 
                if(do_pub){
                    Send(1);
                }


            }

            break;
        case 3:
            // wlw to leg
            cout<<"wlw to leg"<<endl;
            /* Strategy */
            // read current duty and swing_phase
            // if get in swing phase then turn into walk pose
            check_point=0;
            walk_transform = {0, 0, 0, 0};
            dS = velocity / pub_rate;
            incre_duty = dS / step_length;   
            while(check_point<4){
                // keep walk until transform finish
                for (int i=0; i<4; i++) {
                    next_hip[i][0] += dS ;
                    duty[i] += incre_duty;     
                }
            
                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (duty[i] < 0){ duty[i] += 1.0; }
            
                    /* Calculate next foothold if entering swing phase(1) */
                    // Enter SW (calculate swing phase traj)
                    if ((duty[i] >= (1 - swing_time)) && swing_phase[i] == 0) {
                        swing_phase[i] = 1;
                        swing_pose = find_pose(stand_height, shift, (step_length*3/6), 0);  
                        // tune step length for leg mode
                        step_length = 0.2;
                        double pos[2] = {-step_length/2*(1-swing_time), -stand_height + leg_model.r};
                        swing_pose_temp = leg_model.inverse(pos, "G");
                        if(walk_transform[i] ==0){
                            if (walk_transform[i] ==0){
                                walk_transform[i] = 1;
                                cout<< i << " get in hybrid swing phase" << endl;
                            }
                            // cout << "swing TD pose: "<< swing_pose_temp[0]*180/PI << " , " << swing_pose_temp[1]*180/PI << endl; 
                            // cout << "origin swing : "<< swing_pose[0]*180/PI << " , " << swing_pose[1]*180/PI << endl; 
                      
                        }
                        Swing(current_eta, swing_pose, swing_variation, i);
                        Swing(current_eta, swing_pose_temp, swing_variation_temp, i);
                        
                        // cout << "swing_variation_temp: "<< swing_variation_temp[0]*180/PI << " , " << swing_variation_temp[1]*180/PI << endl; 
                        // cout << "current_eta: "<< current_eta[i][0]*180/PI << " , " << current_eta[i][1]*180/PI << endl; 
                        
                        
                    } 
                    // Enter TD
                    else if ((duty[i] > 1.0)) {           
                        swing_phase[i] = 0;
                        duty[i] -= 1.0; // Keep duty in the range [0, 1]
                        if(walk_transform[i]==1){
                            walk_transform[i]=2;
                            check_point++;
                            cout<< i << " start walk" << endl;
                            cout<< "check_point add " << i << " ,current= " << check_point<< endl;
                            
                        }
                        else if(walk_transform[i]!=2){
                            cout<< i << " still in wlw" << endl;  
                        }
                    }
            
                    /* Calculate next eta */
                    // calculate the nest Stance phase traj
                    if (swing_phase[i] == 0) { 
                            leg_model.forward(current_eta[i][0], current_eta[i][1],true);
                            std::array<double, 2> result_eta;
                            result_eta = leg_model.move(current_eta[i][0], current_eta[i][1], {-dS, 0}, 0);
                            current_eta[i][0] = result_eta[0];
                            current_eta[i][1] = result_eta[1];                 
                    } 
                    // read the next Swing phase traj
                    else { 
                        if (walk_transform[i]!=0){
                            // to walk
                            Swing_step(swing_pose_temp, swing_variation_temp, current_eta, i, duty[i]);
                        }
                        else{
                            // origin tranform
                            Swing_step(swing_pose, swing_variation, current_eta, i, duty[i]);
                        }
                        
                    }
                }
                // Send eta 
                if(do_pub){
                    Send(1);
                }
            }
            break;
    };   
}

std::vector<double> WLWGait::linspace(double start, double end, int num_steps) {
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

void WLWGait::Transfer(int transfer_sec, int wait_sec, int do_pub){
    // next_eta = target and grep current_motor_pose -> devided to step until current_eta
    // transfer
    std::vector<std::vector<double>> theta_steps(4), beta_steps(4);
    for (int i=0; i<4; i++){
        theta_steps[i] = linspace(motor_state_modules[i]->theta, next_eta[i][0], transfer_sec*pub_rate);
        beta_steps[i]  = linspace(motor_state_modules[i]->beta,  next_eta[i][1], transfer_sec*pub_rate);
    }
    for (int step_i = 0; step_i < transfer_sec*pub_rate; step_i++) {
        for  (int i=0; i<4; i++){
            current_eta[i][0] = theta_steps[i][step_i];
            current_eta[i][1] = beta_steps[i][step_i];
        }       
        // cout << step_i << ": " <<current_eta[1][0]*180.0/M_PI << ", "<< current_eta[1][1]*180.0/M_PI << std::endl;
        if (do_pub) {
            Send(1);
        }
    }

    // wait
    for (int step_i = 0; step_i < wait_sec*pub_rate; step_i++) {
        if (do_pub) {
            Send(1);
        }
    }

}

void WLWGait::change_Height(double new_value){
    stand_height = new_value;
    for (int i=0; i<4; i++) {
        next_hip[i][1] = stand_height;
    }
    // add limitation
}

void WLWGait::change_Step_length(double new_value){
    new_step_length = new_value;
    // add limitation
}

void WLWGait::change_Velocity(double new_value){
    velocity = new_value;
    dS = velocity / pub_rate;
    incre_duty = dS / step_length;  
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("wlw_test");

    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double CoM_bias = 0.0;
    int pub_rate = 1000;
    
    //*  wlw functions  *//
    /*  wlw initialize  */
    WLWGait wlw_gait(nh, true, CoM_bias, pub_rate);   

    /*  wlw initial pose  */
    // wlw_gait.Initialize(2, 300, 0, 0, 5, 2, 0);
    wlw_gait.Initialize(2, 10, 0, 0, 2, 2, -0.05);
    
    /*  wheel to wlw transform  */
    cout<< "-----transform to wlw------"<<endl;
    // wlw_gait.Transform(0, 1, 1, 5, 10, 0);
    wlw_gait.Transform(0, 1, 1, 1, 0, -0.05);

    /*  wlw real-time   */
    cout<< "-----wlw------"<<endl;
    cout<< "walking"<<endl;
    for (int step = 0;step<1000;step++) {
        wlw_gait.motor_cmd.header.seq = step;
        wlw_gait.motor_cmd.header.stamp = rclcpp::Time::now();
        wlw_gait.Step(1, 1, -0.05);
    }

    cout<< "----wlw-variation----"<<endl;
    cout<< "Height from 0.14 to 0.165"<<endl;
    for (int step = 0;step<10000;step++) {
        // Test change_Height
        if(step <= 8000){
            wlw_gait.change_Height(0.14 + 0.025*(step)/8000);
        }
       
       
        // Test change_Step_length
        if (step >= 5000){
            if(step == 5000){
                cout<< "SL from 0.4 to 0.3"<<endl;
                cout<< "V from 0.08 to 0.05"<<endl;
            }
            wlw_gait.change_Step_length(0.3);
            wlw_gait.change_Velocity(0.05);
            wlw_gait.motor_cmd.header.seq = step;
            wlw_gait.motor_cmd.header.stamp = rclcpp::Time::now();
            wlw_gait.Step(1, 1, -0.06);
        }
        else{
            wlw_gait.motor_cmd.header.seq = step;
            wlw_gait.motor_cmd.header.stamp = rclcpp::Time::now();
            wlw_gait.Step(1, 1, -0.05);
        }
        
        
    }
   
    cout << "swing_phase_temp: "<< wlw_gait.swing_phase[0] << " , " << wlw_gait.swing_phase[1] << " , " << wlw_gait.swing_phase[2] << " , " << wlw_gait.swing_phase[3] << endl; 
    cout << "duty: "<< wlw_gait.duty[0] << " , " << wlw_gait.duty[1] << " , " << wlw_gait.duty[2] << " , " << wlw_gait.duty[3] << endl; 
           
    /*  wlw to walk transform   */
    cout<< "-----transform to leg------"<<endl;
    wlw_gait.Transform(3, 1, 0, 3, 0, -0.06);
    
    /*  try walk real-time   */
    cout<< "-----leg------"<<endl;
    WalkGait walk_gait(true, 0.0, 1000);
    cout<< "transport eta "<<endl;
    std::array<std::array<double, 4>, 2> eta_list;
    double init_eta[8];
    for (int i=0; i<4; i++){
        init_eta[2*i] = wlw_gait.current_eta[i][0] ;
        if(i==1 | i==2){
            init_eta[2*i+1] =-wlw_gait.current_eta[i][1] ;
        }
        else{
            init_eta[2*i+1] = wlw_gait.current_eta[i][1] ;
        }
    }
    
    // double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    // double init_eta[8] = {
    //       0.8340308101719882 , -0.025117378374684307 
    //     , 1.381705425444313  , 0.779336487921992 
    //     , 1.052012758141898  , 0.5300286393524838 
    //     , 0.9164608588034946 , 0.3283090821356041};
    cout<< "initialize"<<endl;
    walk_gait.initialize(init_eta);   
    
    cout<< "transport duty "<<endl;
    for (int i=0; i<4; i++){
        walk_gait.duty[i]=wlw_gait.duty[i];
        walk_gait.swing_phase[i]=wlw_gait.swing_phase[i];
    }

    cout<< "start walk"<<endl;
    int check_walk = 0;
    for (int step = 0;step<8000;step++) {
        walk_gait.set_stand_height(0.165-0.005*(step+1)/8000);
        if (check_walk!=4){
            check_walk = 0;
            for(int phase=0;phase<4;phase++){
                if(walk_gait.swing_phase[phase]==0 && step>6000){
                    check_walk++;
                }
            }
            eta_list = walk_gait.step();
            for (int i=0; i<4; i++) {
                if (eta_list[0][i] > M_PI*159.9/180.0) {
                    RCLCPP_INFO(rclcpp::get_logger("WlwGen"), "Exceed Upper Bound.\n");
                    eta_list[0][i] = M_PI*159.9/180.0;
                }
                if (eta_list[0][i] < M_PI*16.9/180.0) {
                    RCLCPP_INFO(rclcpp::get_logger("WlwGen"), "Exceed Lower Bound.\n");
                    eta_list[0][i] = M_PI*16.9/180.0;
                }
                wlw_gait.motor_cmd_modules[i]->theta = eta_list[0][i];
                wlw_gait.motor_cmd_modules[i]->beta = (i == 1 || i == 2) ? eta_list[1][i] : -eta_list[1][i];
                wlw_gait.motor_cmd_modules[i]->kp_r = 150;
                wlw_gait.motor_cmd_modules[i]->ki_r = 0;
                wlw_gait.motor_cmd_modules[i]->kd_r = 1.75;
                wlw_gait.motor_cmd_modules[i]->kp_l = 150;
                wlw_gait.motor_cmd_modules[i]->ki_l = 0;
                wlw_gait.motor_cmd_modules[i]->kd_l = 1.75;
                wlw_gait.motor_cmd.header.seq = step;
                wlw_gait.motor_cmd.header.stamp = rclcpp::Time::now();
            }
            wlw_gait.publish(1);
        }
        else{
            cout << "Duty: "<< walk_gait.duty[0] << " , " << walk_gait.duty[1] << " , " << walk_gait.duty[2] << " , " << walk_gait.duty[3] << endl; 
            cout << "swing_phase: "<< walk_gait.swing_phase[0] << " , " << walk_gait.swing_phase[1] << " , " << walk_gait.swing_phase[2] << " , " << walk_gait.swing_phase[3] << endl; 
            break;
        }
                
    }
    
     // cout << "current_eta walk" << endl;
    // for (int i=0; i<4; i++){
    //     cout<< i <<": "<<eta_list[i][0]<<" , " <<eta_list[i][1] << endl;
    // }
  
    /*  walk to wlw transform   */
    wlw_gait.velocity     = 0.05;
    wlw_gait.stand_height = 0.160;
    wlw_gait.step_length  = 0.3;
    wlw_gait.Initialize(2, 10, 0, 0, 2, 2, -0.06);
    for (int i=0; i<4; i++){
        wlw_gait.foothold[i][0] = walk_gait.foothold[i][0];
        wlw_gait.foothold[i][0] = walk_gait.foothold[i][1];
        
        wlw_gait.hip[i][0] = walk_gait.hip[i][0];
        wlw_gait.hip[i][1] = walk_gait.hip[i][1];

        wlw_gait.next_hip[i][0] = walk_gait.next_hip[i][0];
        wlw_gait.next_hip[i][1] = walk_gait.next_hip[i][1];
        
        wlw_gait.duty[i] = walk_gait.duty[i];
        wlw_gait.swing_phase[i] = walk_gait.swing_phase[i];
        // change to read the current from topic
        wlw_gait.current_eta[i][0] = eta_list[0][i] ;
        wlw_gait.current_eta[i][1] = -eta_list[1][i];
    }        
    
    cout<< "-----transform to wlw------"<<endl;
    wlw_gait.Transform(2, 1, 0, 3, 0, -0.06);


    cout<< "-----wlw walking-----"<<endl;
    for (int step = 0;step<5000;step++) {
        wlw_gait.motor_cmd.header.seq = step;
        wlw_gait.motor_cmd.header.stamp = rclcpp::Time::now();
        wlw_gait.Step(1, 1, -0.03);
    }

    cout<< "-----transform to wheel-----"<<endl;
    wlw_gait.Transform(1, 1, 0, 3, 0, -0.06);
    // for (int step = 0;step<7500;step++) {
    //     wlw_gait.change_Height(0.165 - (0.165-0.1125)*(step)/7500);
    //     wlw_gait.change_Step_length(0.3+0.1*(step)/7500);
    //     wlw_gait.motor_cmd.header.seq = step;
    //     wlw_gait.motor_cmd.header.stamp = rclcpp::Time::now();
    //     wlw_gait.Step(1, 1, -0.05);
    // }
    // // until theta = 17+PI/180
    // for (int step = 0;step<1000;step++) {
    //     wlw_gait.change_Height(0.1125);
    //     wlw_gait.motor_cmd.header.seq = step;
    //     wlw_gait.motor_cmd.header.stamp = rclcpp::Time::now();
    //     wlw_gait.Step(1, 1, -0.05);
    // }
    

    cout<< "-----end-----"<<endl;
    ros::shutdown();
    return 0;
}

// stability
// collision
// slope
// print (draw)
// how to swing
// multi-loop
// variaty
// add reading initial pose (find the leg that should be the front)
// add transfer(like csv)
// transform
// turn

