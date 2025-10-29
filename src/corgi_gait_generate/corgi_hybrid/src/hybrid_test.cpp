#include "hybrid_gen.hpp"

#include <iostream>
using namespace std;
// enum STATES {MOVE_STABLE, SLOW_DOWN, SWING_SAME, SWING_NEXT, END};
//         STATES state;
//         STATES last_state;

int main(int argc, char** argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiHybrid"), "Hybrid mode test\n");
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("corgi_hybrid_test");

    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*     Gait Selector Setting    */ 
    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);
    auto gaitSelector = std::make_shared<GaitSelector>(nh, sim, CoM_bias, pub_rate);
    std::cout << "gaitSelector initialized" << std::endl;
    /*    Initialize of each mode   */ 
    // hybrid mode 
    Hybrid hybrid(gaitSelector); 

    /*  wlw initial pose  */
    std::cout << "hybrid" << std::endl;

    // // for 5 (0.17) swing_pose[1] += 0.1745329252;
    // gaitSelector->current_stand_height[0] = gaitSelector->current_stand_height[0]-0.444*tan(0.0872664626);
    // gaitSelector->current_stand_height[1] = gaitSelector->current_stand_height[1]-0.444*tan(0.0872664626);
    // hybrid.Initialize(1, 1);
    // for(int i = 0; i < 4; i++) {
    //     gaitSelector->next_eta[i][1] = gaitSelector->next_eta[i][1]+0.0872664626;
    // }
    // // gaitSelector->Transfer(gaitSelector->transfer_sec, gaitSelector->wait_sec);
    // for(int k=0;k<10;k++){
    //     gaitSelector->Send();
    // }
    // hybrid.update_nextFrame();
    // gaitSelector->body = gaitSelector->next_body;
    // gaitSelector->hip = gaitSelector->next_hip;
    // gaitSelector->foothold = gaitSelector->next_foothold;
    // while(1){
    //     if((int)gaitSelector->trigger_msg.enable){
    //         hybrid.Step();
    //         gaitSelector->Send();
    //         // if(gaitSelector->hip[0][0] <1){
    //         //     std::cout << gaitSelector->hip[0][0] << std::endl;
    //         // }
    //     }
    //     else{
    //         std::cout << "Waiting for trigger..." << std::endl;
    //     }
    // }

    // // for 10
    // double slope = 0.1466077; // in rad 8.4 degree
    // gaitSelector->step_length = 0.25;
    // gaitSelector->current_shift[2] = -0.05;
    // gaitSelector->current_shift[3] = -0.05;
    // gaitSelector->current_stand_height[2] = gaitSelector->leg_model.radius+0.444*tan(slope)+0.01;
    // gaitSelector->current_stand_height[3] = gaitSelector->leg_model.radius+0.444*tan(slope)+0.01;
    // gaitSelector->dS = gaitSelector->velocity / gaitSelector->pub_rate;
    // gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length; 
    // // gaitSelector->duty = {1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 0.5, 0.0};
    // gaitSelector->duty = {1 - gaitSelector->swing_time, 0.5 - gaitSelector->swing_time, 0.5, 0.0};
    // for(int i =2; i<4;i++){
    //         // std::cout << "gaitSelector->duty[" << i << "] = " << gaitSelector->duty[i] << std::endl;
    //         auto tmp0 = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], (gaitSelector->step_length) , gaitSelector->duty[i],slope);
    //         // double height, float shift, float steplength, double duty, double slope
    //         gaitSelector->next_eta[i][0] = tmp0[0];
    //         gaitSelector->next_eta[i][1] = tmp0[1]+slope;

    //     }
    // std::cout << "lo_pose: " << gaitSelector->next_eta[2][0] << ", " << gaitSelector->next_eta[2][1] << std::endl;
    // std::cout << "swing_pose: " << gaitSelector->next_eta[3][0] << ", " << gaitSelector->next_eta[3][1] << std::endl;
    // gaitSelector->next_eta[0][0] = 17*M_PI/180.0;
    // gaitSelector->next_eta[0][1] = 0;
    // gaitSelector->next_eta[1][0] = 17*M_PI/180.0;
    // gaitSelector->next_eta[1][1] = 0;
    // // hybrid.Initialize(1, 1);
    // for(int i = 2; i < 4; i++) {
    //     gaitSelector->next_eta[i][1] = gaitSelector->next_eta[i][1]+slope;
    // }
    // gaitSelector->Transfer(gaitSelector->transfer_sec, gaitSelector->wait_sec);

    // for(int k=0;k<10;k++){
    //     gaitSelector->Send();
    // }
    // hybrid.update_nextFrame();
    // gaitSelector->body = gaitSelector->next_body;
    // gaitSelector->hip = gaitSelector->next_hip;
    // gaitSelector->foothold = gaitSelector->next_foothold;
    // std::cout << gaitSelector->trigger_msg << std::endl;
    // while(1){
    //     if((int)gaitSelector->trigger_msg.enable){
    //         hybrid.Step_wheel();
    //         gaitSelector->Send();
    //     }
    //     else{
    //         std::cout << "Waiting for trigger..." << std::endl;
    //     }
    // }

    // for _/-\_  small one 
    // if successfull. add to wheel
    //  - stair height
    double stair_height = 0.03;
    double slope = 0.24497; // in rad
    double stair_length = 0.1237;// slope 
    gaitSelector->step_length = 0.3;
    gaitSelector->dS = gaitSelector->velocity / gaitSelector->pub_rate;
    gaitSelector->incre_duty = gaitSelector->dS / gaitSelector->step_length;
    gaitSelector->new_step_length = gaitSelector->step_length;
    double ideal_height = 0.17;
    for(int i = 0; i < 4; i++) {
        gaitSelector->current_stand_height[i] = ideal_height;
    }
    gaitSelector->current_shift[3] = -0.1;
    gaitSelector->current_shift[2] = -0.07;
    gaitSelector->duty = {0.0, 0.5, 0.5+gaitSelector->swing_time, gaitSelector->swing_time};
    for(int i =0; i<4;i++){
        auto tmp0 = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], (gaitSelector->step_length) , gaitSelector->duty[i],0);
        // double height, float shift, float steplength, double duty, double slope
        gaitSelector->next_eta[i][0] = tmp0[0];
        gaitSelector->next_eta[i][1] = tmp0[1];
    }
    // gaitSelector->Transfer(gaitSelector->transfer_sec, gaitSelector->wait_sec);
    for(int k=0;k<10;k++){
        gaitSelector->Send();
    }
    hybrid.update_nextFrame();
    gaitSelector->body = gaitSelector->next_body;
    gaitSelector->hip = gaitSelector->next_hip;
    gaitSelector->foothold = gaitSelector->next_foothold;
     // assume the terrain start at the contact point of the front left leg 
    double terrain_start = gaitSelector->hip[0][0] + gaitSelector->relative_foothold[0][0];
    std::cout << "terrain start = " << terrain_start << std::endl;
    std::cout << "the distance between the robot center and the terrain center= " << 1.14-( terrain_start+ stair_length*0.5*cos(slope))<< std::endl;
    
    while(1){
        if((int)gaitSelector->trigger_msg.enable){        
            // start from here
            double move_dist = gaitSelector->hip[2][0] + gaitSelector->swing_time*gaitSelector->step_length + (1-gaitSelector->swing_time-gaitSelector->duty[2])*gaitSelector->step_length;
            // hear leg swing
            std::cout << "start front leg landing" << std::endl;
            double land_position = gaitSelector->hip[1][0]+gaitSelector->swing_time*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length + (1-gaitSelector->swing_time-gaitSelector->duty[2])*gaitSelector->step_length;
            double delta_x = land_position-terrain_start;
            std::cout << gaitSelector->current_stand_height[1] << std::endl;

            double h_op = ideal_height-stair_height;
            // double h_op = (gaitSelector->current_stand_height[1]-delta_x*tan(slope))*cos(slope);
            // double h_op = 0.15;
            gaitSelector->current_stand_height[1] = h_op;
            auto land_pose = hybrid.find_pose(gaitSelector->current_stand_height[1], gaitSelector->current_shift[1], gaitSelector->step_length,0.0, slope);  
            // land_pose[1] += slope; // add slope to the landing pose 
            std::cout << "land_pose: " << land_pose[0] << ", " << land_pose[1] << std::endl;
            // front leg swing to the terrain
            while(gaitSelector->hip[1][0]<=land_position){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;  
                    if (i==1){
                        hybrid.Swing(gaitSelector->eta, land_pose, hybrid.swing_variation, i); 
                    } 
                    else{
                        auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
                        hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                    }
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(land_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }

            // 4 leg move and swing 3 (now 0,1 on slope)
            std::cout << "start 4 leg move swing 3" << std::endl;
            // gaitSelector->current_shift[3] = ;
            move_dist = gaitSelector->hip[3][0] + gaitSelector->swing_time*gaitSelector->step_length + (1-gaitSelector->swing_time-gaitSelector->duty[3])*gaitSelector->step_length;
            while(gaitSelector->hip[3][0]<=move_dist-gaitSelector->dS){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if (i==0 || i==1) {
                        // maybe for 1 is on the upper plain || i==1
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, slope);
                    } else {
                        // hind leg move
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0.0);
                    }
                    // result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(land_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }
            // swing the front leg to the plain
            std::cout << "start front leg swing to the plain" << std::endl;
            move_dist = gaitSelector->hip[0][0] + gaitSelector->swing_time*gaitSelector->step_length;
            
            gaitSelector->leg_model.forward(gaitSelector->next_eta[1][0], gaitSelector->next_eta[1][1], true);
            gaitSelector->leg_model.contact_map(gaitSelector->next_eta[1][0], -gaitSelector->next_eta[1][1], 0.0, true, true);
            std::cout << "height of the front leg:" << -gaitSelector->leg_model.contact_p[1] << std::endl;
            h_op = -gaitSelector->leg_model.contact_p[1];
            // // h_op = ideal_height - stair_height;
            gaitSelector->current_stand_height[0] = h_op;
            gaitSelector->current_stand_height[1] = h_op;
            land_pose = hybrid.find_pose(gaitSelector->current_stand_height[0], gaitSelector->current_shift[0], gaitSelector->step_length,0.0, 0.0); 
            // land_pose[1] += slope; 
            // std::cout << "land_pose: " << land_pose[0] << ", " << land_pose[1] << std::endl;

            while(gaitSelector->hip[0][0]<=move_dist){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;  
                    if (i==0){
                        hybrid.Swing(gaitSelector->eta, land_pose, hybrid.swing_variation, i); 
                    } 
                    else{
                        auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
                        hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                    }
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(land_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }

            // the front legs are on the stair and the back legs are on the floor
            std::cout << "start 4 leg move and one of the back leg swing on slope" << std::endl;
            move_dist = gaitSelector->hip[2][0] + (1-gaitSelector->swing_time-gaitSelector->duty[2])*gaitSelector->step_length;

            land_position = move_dist+gaitSelector->swing_time*gaitSelector->step_length;
            // delta_x = land_position-terrain_start;
            // h_op = (gaitSelector->current_stand_height[2]-delta_x*tan(slope))*cos(slope);
            // gaitSelector->current_stand_height[2] = h_op;
            gaitSelector->current_shift[2] = 0.01; // 可以小一點點
            auto hind_land_pose = hybrid.find_pose(gaitSelector->current_stand_height[2], gaitSelector->current_shift[2], gaitSelector->step_length,0.0, slope);  
            // hind_land_pose[1] += slope; // add slope to the landing pose 
            std::cout << "hind_land_pose: " << hind_land_pose[0] << ", " << hind_land_pose[1] << std::endl;
            // land_pose = hybrid.find_pose(gaitSelector->current_stand_height[0], -0.3, gaitSelector->step_length,0.0, 0.0); 
            // auto land_pose_2 = hybrid.find_pose(gaitSelector->current_stand_height[0], 0, gaitSelector->step_length,0.0, 0.0); 
            while(gaitSelector->hip[2][0]<land_position){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;  
                    if (i==2){
                        hybrid.Swing(gaitSelector->eta, hind_land_pose, hybrid.swing_variation, i); 
                    } 
                    else{
                        auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
                        hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                    }
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(hind_land_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }
            
            std::cout << "front leg swing (RF)"<< std::endl;
            move_dist = gaitSelector->hip[1][0] +gaitSelector->swing_time*gaitSelector->step_length;
            while(gaitSelector->hip[1][0]<move_dist){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    // auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, land_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if(i==2){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, slope);
                    
                    }
                    else{
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    }
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(land_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }
            
            std::cout << "4 leg move done and last back leg swing on slope" << std::endl;
            move_dist = gaitSelector->hip[3][0] + (1-gaitSelector->swing_time-gaitSelector->duty[3])*gaitSelector->step_length;
            land_position = move_dist+gaitSelector->swing_time*gaitSelector->step_length;

            // gaitSelector->current_stand_height[3] =gaitSelector->current_stand_height[1];
            delta_x = land_position-terrain_start;
            h_op = (gaitSelector->current_stand_height[3]-delta_x*tan(slope))*cos(slope);
            gaitSelector->current_shift[3] = 0;
            
            gaitSelector->current_stand_height[3] = h_op;
            land_pose = hybrid.find_pose(gaitSelector->current_stand_height[3], gaitSelector->current_shift[3], gaitSelector->step_length,0.0, 0.0); 
            land_pose[1] += slope; 
            while(gaitSelector->hip[3][0]<land_position-gaitSelector->dS){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    // auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, land_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if(i==2){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    
                    }
                    else{
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    }
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(land_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }
            
            std::cout << "front leg swing (LF)"<< std::endl;
            double terrain_second = terrain_start + cos(slope) * stair_length + 0.4;
            move_dist = gaitSelector->hip[0][0] +gaitSelector->swing_time*gaitSelector->step_length;
            h_op = abs(-slope*move_dist-ideal_height+stair_height+slope*terrain_second)/sqrt(1+pow(slope,2));
            std::cout << "h_op = " << h_op << std::endl;
            gaitSelector->current_stand_height[0] = h_op;
            auto new_land_pose = hybrid.find_pose(gaitSelector->current_stand_height[0], 0, gaitSelector->step_length,0.0, 0);  
            new_land_pose[1] -= slope; // add slope to the landing pose 
            while(gaitSelector->hip[0][0]<move_dist-gaitSelector->dS){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    // auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], gaitSelector->current_shift[i], gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, new_land_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if(i==2){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    
                    }
                    else if(i==1){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    }
                    else{
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, slope);
                    }
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(new_land_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }
            // std::cout << "front leg swing to the slope done" << std::endl;

            std::cout << "swing the RH leg with LH height and the front leg td on -slope" << std::endl;
            move_dist = gaitSelector->hip[2][0] +gaitSelector->swing_time*gaitSelector->step_length + (1-gaitSelector->swing_time-gaitSelector->duty[2])*gaitSelector->step_length;

            gaitSelector->leg_model.forward(gaitSelector->next_eta[3][0], gaitSelector->next_eta[3][1], true);
            gaitSelector->leg_model.contact_map(gaitSelector->next_eta[3][0], gaitSelector->next_eta[3][1], 0.0, true, true);
            std::cout << "height of the hind leg:" << -gaitSelector->leg_model.contact_p[1] << std::endl;
            h_op = -gaitSelector->leg_model.contact_p[1];
            // h_op = ideal_height - stair_height;
            gaitSelector->current_stand_height[2] = h_op;
            while(gaitSelector->hip[2][0]<move_dist-gaitSelector->dS){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i],0.1, gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if(i==0 ){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, -slope);
                    }
                    else if(i==3 ){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    }
                    else{
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    }
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(hybrid.swing_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }

            // swing the front leg to the floor (RF)
            move_dist = gaitSelector->hip[1][0] +gaitSelector->swing_time*gaitSelector->step_length;
            gaitSelector->current_stand_height[1] = ideal_height;
            std::cout << "going down" << std::endl;
            while(gaitSelector->hip[1][0]<move_dist){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i],0, gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if(i==2 || i==3){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    
                    }
                    else{
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, -slope);
                    }
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(hybrid.swing_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }
            std::cout << "front leg swing to the floor done" << std::endl;

            std::cout << "4 leg move and swing the back (LH)" << std::endl;
            std::cout << "the last front leg going down" << std::endl;
            gaitSelector->current_stand_height[3] = gaitSelector->current_stand_height[2];
            gaitSelector->current_stand_height[0] = ideal_height;
            move_dist = gaitSelector->hip[3][0] + (1-gaitSelector->swing_time-gaitSelector->duty[3])*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length+gaitSelector->swing_time*gaitSelector->step_length;
            double inter =  gaitSelector->hip[3][0] + (1-gaitSelector->swing_time-gaitSelector->duty[3])*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length;
            while(gaitSelector->hip[3][0]<=move_dist){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], 0, gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if(i==0 && gaitSelector->hip[3][0]<=inter){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, -slope);
                    
                    }
                    else{
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    }
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(hybrid.swing_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }

            std::cout << "4 leg move and swing the back (RH) and swing the RF" << std::endl;
            move_dist = gaitSelector->hip[2][0] + (1-gaitSelector->swing_time-gaitSelector->duty[2])*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length;
            while(gaitSelector->hip[2][0]<=move_dist){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], 0, gaitSelector->step_length, 0.0,0);  
                    hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                            
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(hybrid.swing_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }


            std::cout << "4 leg move and swing the back (LH) and swing the LF" << std::endl;
            inter = gaitSelector->hip[3][0] + (1-gaitSelector->swing_time-gaitSelector->duty[3])*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length;
            h_op = abs(-slope*inter-ideal_height+stair_height+slope*terrain_second)/sqrt(1+pow(slope,2));
            std::cout << "h_op = " << h_op << std::endl;
            gaitSelector->current_stand_height[3] = h_op;
            new_land_pose = hybrid.find_pose(gaitSelector->current_stand_height[3], 0, gaitSelector->step_length,0.0, 0);  
            new_land_pose[1] -= slope; // add slope to the landing pose 
            move_dist = gaitSelector->hip[3][0] + (1-gaitSelector->swing_time-gaitSelector->duty[3])*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length;
            while(gaitSelector->hip[3][0]<=move_dist){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], 0, gaitSelector->step_length, 0.0,0);  
                    if(i==3){
                        hybrid.Swing(gaitSelector->eta, new_land_pose, hybrid.swing_variation, i);
                    }
                    else{   
                        hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                    }
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    if(i==2){
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, -slope);
                    }
                    else{
                        result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);  
                    }
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(hybrid.swing_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }

            std::cout << "4 leg move and swing the back (RH) and swing the RF" << std::endl;
            gaitSelector->current_stand_height[2] = ideal_height;
            gaitSelector->current_stand_height[3] = ideal_height;
            move_dist = gaitSelector->hip[2][0] + (1-gaitSelector->swing_time-gaitSelector->duty[2])*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length+ gaitSelector->swing_time*gaitSelector->step_length;
            while(gaitSelector->hip[2][0]<=move_dist){
                for (int i=0; i<4; i++) {
                    gaitSelector->next_hip[i][0] += gaitSelector->dS;
                    gaitSelector->duty[i] += gaitSelector->incre_duty;    
                }

                for (int i=0; i<4; i++) {
                    /* Keep duty in the range [0, 1] */
                    if (gaitSelector->duty[i] < 0){ 
                        gaitSelector->duty[i] += 1.0; 
                    }

                /* Calculate next foothold if entering swing phase*/
                // Enter SW (calculate swing phase traj)
                if ((gaitSelector->duty[i] > (1 - gaitSelector->swing_time)) && gaitSelector->swing_phase[i] == 0) {
                    gaitSelector->swing_phase[i] = 1;
                    // change to new step length when front leg start to swing
                    // front leg swing
                    gaitSelector->next_step_length[i] = gaitSelector->new_step_length;
                    auto swing_pose = hybrid.find_pose(gaitSelector->current_stand_height[i], 0, gaitSelector->step_length, 0.0,0);
                    hybrid.Swing(gaitSelector->eta, swing_pose, hybrid.swing_variation, i);
                } 
                // Enter TD
                else if ((gaitSelector->duty[i] > 1.0)) {                  
                    gaitSelector->swing_phase[i] = 0;
                    gaitSelector->duty[i] -= 1.0; 
                    gaitSelector->current_step_length[i] = gaitSelector->next_step_length[i];  
                }
                /* Calculate next gaitSelector->eta */
                // calculate the nest Stance phase traj
                if (gaitSelector->swing_phase[i] == 0) { 
                    gaitSelector->leg_model.forward(gaitSelector->eta[i][0], gaitSelector->eta[i][1],true);
                    std::array<double, 2> result_eta;
                    result_eta = gaitSelector->leg_model.move(gaitSelector->eta[i][0], gaitSelector->eta[i][1], {(gaitSelector->next_hip[i][0]-gaitSelector->hip[i][0]), gaitSelector->next_hip[i][2]-gaitSelector->hip[i][2]}, 0);  
                    gaitSelector->next_eta[i][0] = result_eta[0];
                    gaitSelector->next_eta[i][1] = result_eta[1];
                } 
                // read the next Swing phase traj
                else { 
                    hybrid.Swing_step(hybrid.swing_pose, hybrid.swing_variation, i, gaitSelector->duty[i]);
                }
                // update the gaitSelector->hip position
                gaitSelector->hip[i] = gaitSelector->next_hip[i];
                }
                gaitSelector->Send();
            }
            // optional to wheel
            std::cout << "lower to wheel mode" << std::endl;
            int lower_count=0;
            while(gaitSelector->stand_height > gaitSelector->leg_model.radius+0.02){
                lower_count = 0;
                for (int i=0; i<4; i++) {
                    if(gaitSelector->swing_phase[i]==0){
                        lower_count++;
                    }
                }
                if(lower_count==4){
                    hybrid.change_Height_all(gaitSelector->stand_height-0.0001);
                }
                hybrid.Step();
                gaitSelector->Send();
                if(gaitSelector->next_eta[0][0] <= 18 * M_PI/180 &&
                gaitSelector->next_eta[1][0] <= 18 * M_PI/180 &&
                gaitSelector->next_eta[2][0] <= 18 * M_PI/180 &&
                gaitSelector->next_eta[3][0] <= 18 * M_PI/180 ){
                    gaitSelector->stand_height = gaitSelector->leg_model.radius;
                }
            }
            std::cout << "wheel mode" << std::endl;
            double wheel_delta_beta = gaitSelector->velocity/(gaitSelector->leg_model.radius*gaitSelector->pub_rate);
            for (int i =0;i<5000;i++){
                for (int j=0;j<4;j++){
                    gaitSelector->next_eta[j][0] = 17 * M_PI/180;
                    gaitSelector->next_eta[j][1] -= wheel_delta_beta;
                }
                gaitSelector->Send();
            }
            break;
    }
        else{
            std::cout << "Waiting for trigger..." << std::endl;
        }
    }


    return 0;
}


