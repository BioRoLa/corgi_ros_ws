#include "Simple_fsm.hpp"
#include "hybrid_gen.hpp"
#include "wheeled_gen.hpp"
#include "legged_gen.hpp"
// #include "transform_gen.hpp"

// class KeybordControl : public Wheeled, public Hybrid, public Legged, public Transform
// {
//     public: 
//         KeybordControl(rclcpp::Node& nh, GaitSelector& gs, Gait defaultGait): Wheeled(nh), Hybrid(nh), Legged(nh), Transform(), gaitSelector(gs)
//         {
//             gaitSelector.currentGait = defaultGait;
//             gaitSelector.newGait     = defaultGait;
//             if (defaultGait == Gait::WHEELED) {
//                 Roll(1,1,1,0,4095,0);
//             } else if (defaultGait == Gait::HYBRID) {
//                 // Initialize(2,1,1,1,5,2,-0.03);
//             }

//             input_thread = std::thread(&KeybordControl::keyboardInputThread, this);
//             exec_thread  = std::thread(&KeybordControl::Keep, this);
//         }
//         ~KeybordControl() {
//             if (input_thread.joinable()) input_thread.join();
//             if (exec_thread .joinable()) exec_thread .join();
//         }

//         void keyboardInputThread() {
//             while (rclcpp::ok()) {
//                 std::string input_line;
//                 std::cout << "Enter command: ";
//                 std::getline(std::cin, input_line);

//                 std::lock_guard<std::mutex> lock(input_mutex);
//                 if (input_line.empty()) {                    
//                     continue;
//                 }

//                 char identifier = input_line[0];
//                 if (identifier == 'p') {
//                     if (!paused) {
//                         paused = true;
//                         prevGait = gaitSelector.currentGait;
//                         std::cout << "Paused" << std::endl;
//                     }
//                 } else if (identifier == 'r') {
//                     if (paused) {
//                         paused = false;
//                         gaitSelector.currentGait = prevGait;
//                         std::cout << "Resume to ";
//                         printCurrentGait();
//                     }
//                 } else if (identifier == 'l') {
//                     changeGait("2");
//                 } else if (identifier == 'w') {
//                     changeGait("1");
//                 } else if (identifier == 'h') {
//                     changeGait("3");
//                 } else if (identifier == 'q') {
//                     std::cout << "Quitting..." << std::endl;
//                     ros::shutdown();
//                 } else {
//                     changeGait(input_line);
//                 }
//             }
//         }
        
//     private:
//         GaitSelector& gaitSelector; 
//         std::mutex input_mutex;
//         std::thread input_thread;
//         std::thread exec_thread;
//         std::atomic<bool> paused{false};
//         Gait prevGait = Gait::WHEELED;

//         void Keep() {
//             rclcpp::Rate rate(gaitSelector.pub_rate);
//             while (rclcpp::ok()) {
//                 bool is_paused = paused.load();
//                 Gait current = gaitSelector.currentGait;
    
//                 if (!is_paused) {
//                     switch (current) {
//                         case Gait::WHEELED:   Roll(1,1,1,0,4095,0);   break;
//                         case Gait::LEGGED:    next_Step();             break;
//                         case Gait::HYBRID:    Step(1,1,-0.03);         break;
//                         case Gait::TRANSFORM:  break;
//                         default: break;
//                     }
//                 } else {
//                     std::cout<<"Paused\n";
//                 }
//                 rate.sleep();
//             }
//         }

//         void changeGait(const std::string& command) {
//             // Set the newGait to the current value by default.
//             gaitSelector.newGait = gaitSelector.currentGait;
        
//             if (command == "1") {
//                 gaitSelector.newGait = Gait::WHEELED;
//             } else if (command == "2") {
//                 gaitSelector.newGait = Gait::LEGGED;
//             } else if (command == "3") {
//                 gaitSelector.newGait = Gait::HYBRID;
//             }

//             // Parameter change commands:
//             else if (command[0] == 'v') {
//                 std::string velocity_str = command.substr(1);
//                 try {
//                     // see the gait and use the function od the wheel mode just set
//                     double newVelocity = std::stod(velocity_str);
//                     std::cout << "Origin velocity : " << gaitSelector.velocity << std::endl;
//                     gaitSelector.velocity = newVelocity;
//                     std::cout << "Setting velocity : " << gaitSelector.velocity << std::endl;
//                 } catch (const std::exception& e) {
//                     std::cerr << "Invalid velocity parameter: " << velocity_str << std::endl;
//                 }
//                 return;
//             } else if (command[0] == 's') {
//                 std::string stepLength_str = command.substr(1);
//                 try {
//                     // see the gait and use the function od the wheel mode just set
//                     std::cout << "Origin step length : " << gaitSelector.step_length << std::endl;
//                     double newStepLength = std::stod(stepLength_str);
//                     gaitSelector.step_length = newStepLength;   
//                     std::cout << "Setting step length : " << gaitSelector.step_length << std::endl;
//                     // Update your system’s step length accordingly.
//                 } catch (const std::exception& e) {
//                     std::cerr << "Invalid step length parameter: " << stepLength_str << std::endl;
//                 }
//                 return;
//             } else if (command[0] == 'h' && command.size() > 1) {
//                 std::string standHeight_str = command.substr(1);
//                 try {
//                     // see the gait and use the function od the wheel mode just set
//                     std::cout << "Origin stand height : " << gaitSelector.stand_height << std::endl;
//                     double newStandHeight = std::stod(standHeight_str);
//                     gaitSelector.stand_height = newStandHeight;
//                     std::cout << "Setting stand height : " << gaitSelector.stand_height << std::endl;
//                     // Update system standing height here.
//                 } catch (const std::exception& e) {
//                     std::cerr << "Invalid stand height parameter: " << standHeight_str << std::endl;
//                 }
//                 return;
//             } else if (command[0] == 'j') {
                
//                 // Change lift height command:
//                 std::string stepHeight_str = command.substr(1);
//                 try {
//                     // see the gait and use the function od the wheel mode just set
//                     std::cout << "Origin lift height : " << gaitSelector.step_height << std::endl;
//                     double newStepHeight = std::stod(stepHeight_str);
//                     gaitSelector.step_height = newStepHeight;
//                     std::cout << "Setting lift height : " << gaitSelector.step_height << std::endl;
//                     // Update system lift height here.
//                 } catch (const std::exception& e) {
//                     std::cerr << "Invalid lift height parameter: " << stepHeight_str << std::endl;
//                 }
//                 return;
//             } else {
//                 std::cerr << "Unknown command: " << command << std::endl;
//                 return;
//             }
            
//             // If the command resulted in a gait change, do the gait transformation.
//             if (gaitSelector.newGait != gaitSelector.currentGait) {
//                 printCurrentGait();
//                 SwitchGait();
//                 gaitSelector.currentGait = gaitSelector.newGait;
//                 printCurrentGait();
//             }
//         }
        
        
//         void printCurrentGait() const {
//             switch (gaitSelector.currentGait) {
//             case Gait::WHEELED:
//                 std::cout << "Current gait: WHEELED" << std::endl;
//                 break;
//             case Gait::LEGGED:
//                 std::cout << "Current gait: LEGGED" << std::endl;
//                 break;
//             case Gait::HYBRID:
//                 std::cout << "Current gait: HYBRID" << std::endl;
//                 break;
//             case Gait::TRANSFORM:
//                 std::cout << "Current gait: TRANSFORM" << std::endl;
//                 break;
//             default:
//                 std::cerr << "Unknown gait" << std::endl;
//                 break;
//             }
//         }

//         void SwitchGait() {
//             switch (gaitSelector.currentGait)
//             {
//             case Gait::WHEELED:
//                 gaitSelector.currentGait = Gait::TRANSFORM;
//                 printCurrentGait();
//                 if(gaitSelector.newGait == Gait::LEGGED) {
//                     std::cout << "Transforming from WHEELED to LEGGED" << std::endl;
//                     GaitTransform(Gait::WHEELED, Gait::LEGGED);
//                 } 
//                 else if(gaitSelector.newGait == Gait::HYBRID) {
//                     std::cout << "Transforming from WHEELED to HYBRID" << std::endl;
//                     GaitTransform(Gait::WHEELED, Gait::HYBRID);
//                 }
//                 else{
//                     std::cerr << "Unknown transform gait" << std::endl;
//                 }
//             break;
//             case Gait::LEGGED:
//                 gaitSelector.currentGait = Gait::TRANSFORM;
//                 printCurrentGait();
//                 if(gaitSelector.newGait == Gait::WHEELED) {
//                     std::cout << "Transforming from LEGGED to WHEELED" << std::endl;
//                     GaitTransform(Gait::LEGGED, Gait::WHEELED);
//                 } 
//                 else if(gaitSelector.newGait == Gait::HYBRID) {
//                     std::cout << "Transforming from LEGGED to HYBRID" << std::endl;
//                     GaitTransform(Gait::LEGGED, Gait::HYBRID);
//                 } 
//                 else{
//                     std::cerr << "Unknown transform gait" << std::endl;
//                 }
//                 break;
//             case Gait::HYBRID:
//                 gaitSelector.currentGait = Gait::TRANSFORM;
//                 printCurrentGait();
//                 if(gaitSelector.newGait == Gait::WHEELED) {
//                     std::cout << "Transforming from HYBRID to WHEELED" << std::endl;
//                     GaitTransform(Gait::HYBRID, Gait::WHEELED);
//                 } 
//                 else if(gaitSelector.newGait == Gait::LEGGED) {
//                     std::cout << "Transforming from HYBRID to LEGGED" << std::endl;
//                     GaitTransform(Gait::HYBRID, Gait::LEGGED);
//                 } 
//                 else{
//                     std::cerr << "Unknown transform gait" << std::endl;
//                 }
//                 break;
            
//             default:
//                 break;
//             }
            
//             std::cout << "Transforming..." << std::endl;
//         }
   
//         bool joyShutdown = false; 
//         std::thread joyThread;
// };

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("corgi_gait_selector");
    // rclcpp::Node nh;

    // //  Start an async spinner to run in parallel.
    // ros::AsyncSpinner spinner(1);
    // spinner.start();    

    // std::string gait_str;
    // nh.param<std::string>("default_gait", gait_str, "wheeled");
    // Gait defaultGait = (gait_str=="legged" ? Gait::LEGGED :
    //     (gait_str=="hybrid"? Gait::HYBRID: Gait::WHEELED));

    // bool sim = true;
    // double CoM_bias = 0.0;
    // int pub_rate = 1000;
    // LegModel leg_model(sim);
    // GaitSelector gaitSelector(nh, sim, CoM_bias, pub_rate);
    
    // // Start the keyboard input thread
    // KeybordControl keybordControl(nh, gaitSelector, defaultGait);   

    // ros::waitForShutdown();
    return 0;
}



