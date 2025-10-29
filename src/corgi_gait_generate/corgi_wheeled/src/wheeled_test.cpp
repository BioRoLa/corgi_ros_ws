#include "wheeled_gen.hpp"
// single execute wheeled mode

int main(int argc, char **argv){
    RCLCPP_INFO(rclcpp::get_logger("CorgiWheeled"), "Wheeled mode test\n");
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("corgi_wheeled_test");

    //  Start an async spinner to run in parallel.
    ros::AsyncSpinner spinner(1);
    spinner.start();    

    /*     Gait Selector Setting    */ 
    bool sim = true;
    double CoM_bias = 0.0;
    int pub_rate = 1000;
    LegModel leg_model(sim);

    GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
   
    /*    Initialize of each mode   */ 
    // Wheeled mode
    WheeledCmd WheeledCmd("joystick");
    Wheeled wheeled(nh);

    while (rclcpp::ok()) {
        // rclcpp::spin_some(node);
    }
    return 0;
}

