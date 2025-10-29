#include "transform_gen.hpp"
#include "hybrid_gen.hpp"
#include "wheeled_gen.hpp"
#include "legged_gen.hpp"

int main(int argc, char** argv) {
    RCLCPP_INFO(rclcpp::get_logger("CorgiTransformation"), "Transform mode test\n");
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("corgi_transform_test");
    
    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    /*     Gait Selector Setting    */ 
    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);

    // GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    auto gaitSelector = std::make_shared<GaitSelector>(nh, sim, CoM_bias, pub_rate);
    std::cout << "gaitSelector initialized" << std::endl;

    /*    Initialize of each mode   */ 
    auto hybrid = std::make_shared<Hybrid>(gaitSelector); 
    auto legged = std::make_shared<Legged>(gaitSelector);
    Transform transformer(hybrid,legged);
    for (int i =0;i<100;i++){
        for (int j=0;j<4;j++){
            gaitSelector->next_eta[j][0] = 17 * PI/180;
            if (j==0){
                gaitSelector->next_eta[j][1] = 100 * PI/180;
            }
            else if (j==1 ){
                gaitSelector->next_eta[j][1] = 100 * PI/180;
            }
            else{
                gaitSelector->next_eta[j][1] = 50 * PI/180;
            }
            
        }
        gaitSelector->Transfer(gaitSelector->transfer_sec, gaitSelector->wait_sec);
    }
    gaitSelector->Send();
    while(1){
        if((int)gaitSelector->trigger_msg.enable){  
        transformer.GaitTransform(Gait::WHEELED, Gait::HYBRID);
        transformer.GaitTransform(Gait::HYBRID, Gait::LEGGED);
        // transformer.GaitTransform(Gait::WHEELED, Gait::LEGGED);
        break;
    }
    else{
            std::cout << "Waiting for trigger..." << std::endl;
        }
    }

    
    std::cout << "End of testing!" << std::endl;
    return 0;
}
