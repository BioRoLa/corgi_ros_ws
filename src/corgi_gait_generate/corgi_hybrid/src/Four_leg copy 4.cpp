#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <vector>
#include <array>
#include <cmath>
#include <nlopt.hpp>
#include "rclcpp/rclcpp.hpp"
#include "leg_model.hpp"
#include <corgi_msgs/msg/motor_state.hpp>
#include <corgi_msgs/msg/motor_state_stamped.hpp>
#include <corgi_msgs/msg/motor_cmd.hpp>
#include <corgi_msgs/msg/motor_cmd_stamped.hpp>


// Structures
struct LegType{
    enum Type {
        WHEELED = 0, 
        HYBRID = 1, 
        LEGGED = 2,
        TRANSFORM = 3
    };
};
struct SwingType{
    enum Type {
        Reverse = 0, 
        Rotate = 1
    };
};
enum TouchdownStatus {
    LEGAL = 0,
    THETA_VIOLATION,
    COLLISION,
    PENETRATION,
    FURTHER,
    UN_CONVERGED
};
struct SwingPoint {
    double time;
    double theta;
    double beta;
};
// Terrain (length 為水平長度)
class Terrain {
    public:
        double terrain_start_height;
        Terrain(double init_height) {
            terrain_start_height = init_height;
        }

        void addPlain(double length) {
            if (segments.empty()) {
                segments.push_back({ 0.0, length, 0.0, terrain_start_height });
            }
            else{
                Segment &prev = segments.back();
                double new_x0 = prev.x0 + prev.length;
                double new_h0 = prev.h0 + prev.slope * prev.length; 
                segments.push_back({ new_x0, length, 0.0, new_h0 });
            }
        }

        void addSlope(double slope, double length) {
            if (segments.empty()) {
                segments.push_back({ 0.0, length, slope, terrain_start_height });
            }
            else{
                Segment &prev = segments.back();
                double new_x0 = prev.x0 + prev.length;
                double new_h0 = prev.h0 + prev.slope * prev.length; 
                segments.push_back({ new_x0, length, slope, new_h0 });
            }
        }

        double height(double x) const {
            if (segments.empty()) {
                return terrain_start_height;
            }
            if (x < segments.front().x0) {
                return terrain_start_height;
            }
            for (const auto &seg : segments) {
                if (x >= seg.x0 && x <= (seg.x0 + seg.length)) {
                    return seg.h0 + seg.slope * (x - seg.x0);
                }
            }
            const Segment &last = segments.back();
            return last.h0 + last.slope * last.length;
        }

        bool check_position(double x) const{
            if (segments.empty()) {
                return false;;
            }
            for (const auto &seg : segments) {
                if (x >= seg.x0 && x <= (seg.x0 + seg.length)) {
                    if(seg.slope == 0.0){
                        return true;
                    }
                    return false;
                }
                return false;
            }
        }

        bool compare_slope(double x,double y) const{
            if (segments.empty()) return true;
            bool found_x = false, found_y = false;
            double slope_x = 0.0, slope_y = 0.0;
            for (const auto &seg : segments) {
                if (!found_x && x >= seg.x0 && x <= seg.x0 + seg.length) {
                    slope_x = seg.slope;
                    found_x = true;
                }
                if (!found_y && y >= seg.x0 && y <= seg.x0 + seg.length) {
                    slope_y = seg.slope;
                    found_y = true;
                }
            }
            if (!found_x || !found_y) {
                return true; 
            }
            return (slope_x == slope_y);
        }

        double slopeAt(double hip_x, bool next) const {
                int idx = -1;
                int N   = segments.size();
                if (segments.empty()) return 0.0;
                if (hip_x < segments.front().x0) return 0.0;
                for (int i = 0; i < N; ++i) {
                    const Segment &seg = segments[i];
                    if (hip_x >= seg.x0 && hip_x <= seg.x0 + seg.length) {
                        idx = i;
                        break;
                    }
                }
                if (idx == -1) {
                    idx = N - 1;
                }

                if (!next) {
                    return segments[idx].slope;
                } else {
                    int next_idx = idx + 1;
                    if (next_idx >= N) {
                        return segments[idx].slope;
                    } else {
                        return segments[next_idx].slope;
                    }
                }
            }

        void printAll() const {
            std::cout << "Terrain segments (x0, length, slope, h0):\n";
            for (const auto &seg : segments) {
                std::cout << "  x0=" << seg.x0
                        << ", length=" << seg.length
                        << ", slope=" << seg.slope 
                        << ", h0=" << seg.h0 << "\n";
            }
        }
        

    private:
        struct Segment {
            double x0;      
            double length;  
            double slope;   // tan(θ)
            double h0;     
        };
        std::vector<Segment> segments;
};
// SingleLeg
struct SingleLeg {
    Eigen::Vector3d hip_position_current; //（世界座標）
    Eigen::Vector3d hip_position_next; //（世界座標）

    Eigen::Vector3d foothold_current;  //（世界座標
    Eigen::Vector3d foothold_next; //（世界座標

    Eigen::Vector2d relative_foothold_current; //（機器人座標）
    Eigen::Vector2d relative_foothold_next; //（機器人座標）

    std::array<double, 2> eta_current;
    std::array<double, 2> eta_next;

    std::vector<SwingPoint> swing_traj;

    double height_current;
    double height_next;

    double shift_current;
    double shift_next;

    double step_length_current;
    double step_length_next;

    double duty;
    int swing_phase;

    LegType::Type leg_type_current; // current leg mode
    LegType::Type leg_type_next; // nex leg mode
    SwingType::Type swing_type; // current swing mode

    SingleLeg()
        : hip_position_current(Eigen::Vector3d::Zero()),
          hip_position_next   (Eigen::Vector3d::Zero()),
          foothold_current    (Eigen::Vector3d::Zero()),
          foothold_next       (Eigen::Vector3d::Zero()),
          relative_foothold_current(Eigen::Vector2d::Zero()),
          relative_foothold_next   (Eigen::Vector2d::Zero()),
          eta_current{0.0, 0.0},
          eta_next   {0.0, 0.0},
          height_current(0.0),
          height_next   (0.0),
          shift_current (0.0),
          shift_next    (0.0),
          step_length_current(0.0),
          step_length_next(0.0),
          duty(0.0),
          swing_phase(0),
          leg_type_current(LegType::WHEELED), 
          leg_type_next(LegType::WHEELED),   
          swing_type(SwingType::Reverse)
        {

        }
};
// Robot
struct Robot {
    Eigen::Vector3d body_position; // 身體位置 （世界座標）
    Eigen::Vector2d CoM_pos; // CoM偏移位置 （世界座標）
    double dS; // 每一步前進距離
    double incre_duty; // duty cycle 增量
    double velocity; // 速度
    double swing_time; // 擺動時間
    int pub_rate;
    
    double new_step_length;

    double BL;
    double BW;
    double BH;

    const double g = 9.81;
    const double m_body = 19.5;
    const double m_leg  = 0.681;
    const double mass = 4 * m_leg + m_body; // 總質量 (kg)
    std::vector<int> S;

    double clearence      = 0.001;    // min安全距離 (m)
    double clearence_M    = 0.01;    // MAX安全距離 (m)

    Terrain* terrain;  // 地形
    std::array<SingleLeg, 4> legs; //  (index 0~3)= (LF, RF, LH, RH)
    Robot()
        : body_position(Eigen::Vector3d::Zero()),
          CoM_pos(Eigen::Vector2d::Zero()),
          dS(0.0),
          incre_duty(0.0),
          velocity(0.0),
          swing_time(0.0),
          pub_rate(1000),
          BL(0.444),
          BW(0.4),
          BH(0.2),
          terrain(nullptr)          
    {
        for (auto &leg : legs) {
            leg.leg_type_current   = LegType::WHEELED;  
            leg.leg_type_next   = LegType::WHEELED;  
            leg.swing_type = SwingType::Reverse;
        }
    }
};
// communitation
corgi_msgs::msg::MotorCmdStamped motor_cmd;
std::vector<corgi_msgs::msg::MotorCmd*> motor_cmd_modules = {
    &motor_cmd.module_a,
    &motor_cmd.module_b,
    &motor_cmd.module_c,
    &motor_cmd.module_d
}; 
corgi_msgs::msg::MotorStateStamped motor_state;
std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
    &motor_state.module_a,
    &motor_state.module_b,
    &motor_state.module_c,
    &motor_state.module_d
};
void motor_state_cb(const corgi_msgs::msg::MotorStateStamped::ConstSharedPtr& state) {
    motor_state = *state;
}
ros::Subscriber motor_state_sub_;
ros::Publisher motor_cmd_pub_; 
rclcpp::Rate* rate_ptr;
// tools
void Print_current_Info(Robot &robot){
    std::cout << "------------- Body Info -------------" << std::endl;
    std::cout << "Body Position: " << robot.body_position.transpose() << std::endl;
    std::cout << "CoM Offset: " << robot.CoM_pos.transpose() << std::endl;
    std::cout << "CoM Position: " << robot.body_position[0]+robot.CoM_pos[0] << "," <<  robot.body_position[1]+robot.CoM_pos[1] << "," <<  robot.body_position[2] << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "-------------  Leg Info -------------" << std::endl;
    for (int i = 0; i < 4; ++i) {
        std::cout << "Leg " << i << " Info: " << std::endl;
        std::cout << "  Hip Position: " << robot.legs[i].hip_position_current.transpose() << std::endl;
        std::cout << "  Foothold Position: " << robot.legs[i].foothold_current.transpose() << std::endl;
        std::cout << "  Relative Foothold Position: " << robot.legs[i].relative_foothold_current.transpose() << std::endl;
        std::cout << "  Height: " << robot.legs[i].height_current << std::endl;
        std::cout << "  Shift: " << robot.legs[i].shift_current << std::endl;
        std::cout << "  Step Length: " << robot.legs[i].step_length_current << std::endl;
        std::cout << "  Leg Type: " << robot.legs[i].leg_type_current << std::endl;
        std::cout << "  Swing Type: " << robot.legs[i].swing_type << std::endl;
        std::cout << "  Duty: " << robot.legs[i].duty << std::endl;
        std::cout << "  Swing Phase: " << robot.legs[i].swing_phase << std::endl;
        if(i == 0 || i == 3){
            std::cout << "  Eta Current: " << rad2deg(robot.legs[i].eta_current[0]) << " , " <<  rad2deg(-robot.legs[i].eta_current[1]) << std::endl;
        }
        else{
            std::cout << "  Eta Current: " << rad2deg(robot.legs[i].eta_current[0]) << " , " <<  rad2deg(robot.legs[i].eta_current[1]) << std::endl;
        }
        
        std::cout << "-------------------------------------" << std::endl;
    }
}
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}
double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}
std::array<double, 2> find_pose(Robot &robot, LegModel &leg, double height, float shift, float steplength, float duty_current, double terrain_slope)
{
    auto eta_ini = leg.inverse({0.0, -height + leg.r}, "G");
    for (double s1 = 0; s1 < shift; s1 += 0.001) {
        eta_ini = leg.move(eta_ini[0], eta_ini[1], { -0.001, 0.0 }, 0.0);
    }
    for (double s = 0; s < 0.5 * steplength; s += 0.001) {
        eta_ini = leg.move(eta_ini[0], eta_ini[1], {-0.001, 0.0}, 0.0);
    }
    for (double t = 0.0; t <= duty_current; t += robot.incre_duty) {
        eta_ini = leg.move(eta_ini[0], eta_ini[1], {robot.dS, 0.0}, atan(terrain_slope));
    }
    return eta_ini;
}
void Send(Robot &robot){
    // next to current and send
    for(int i =0; i<4; i++){
        if (i==0 || i==3) {
            motor_cmd_modules[i]->beta  = -robot.legs[i].eta_next[1];            
        } else {
            motor_cmd_modules[i]->beta  =  robot.legs[i].eta_next[1];
        }     
        motor_cmd_modules[i]->theta = robot.legs[i].eta_next[0];
        motor_cmd_modules[i]->kp_r = 150;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.75;
        motor_cmd_modules[i]->kp_l = 150;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.75;
        robot.legs[i].eta_current=robot.legs[i].eta_next;
    }
    motor_cmd_pub_.publish(motor_cmd);
    rate_ptr->sleep();
}

void optimizePose(Robot &robot, int swing_index) {
    // 1. 先optimize 出一步 sw+td ()
    simulate_status()
}

// Hybrid
void Initialize(Robot &robot, LegModel &leg_model, bool swing_index, int set_type) {
    // set_type = set original pose or not
    // 0 = 設置哪隻前腳先; 1 = 讀取當前姿態並設定duty ; 1 = 讀取當前姿態並繼承參數 
    // (要確定穩定-至少到下一隻腳抬腳前)
    switch(set_type){
        case 0:
            std::array<double, 4>  temp_duty;
            if (!swing_index){
                std::cout << "LF" << std::endl;
                // LF 
                temp_duty ={(1-robot.swing_time), (0.5-robot.swing_time), 0.5, 0.0};
            } 
            else{
                std::cout << "RF" << std::endl;
                // RF
                temp_duty ={(0.5-robot.swing_time), (1-robot.swing_time), 0.0, 0.5};
            }
            for (int i = 0; i < 4; ++i) {
                robot.legs[i].duty = temp_duty[i];
                robot.legs[i].swing_phase = 0;
            }
            optimizePose(robot, swing_index);
             robot.legs[i].eta_next = find_pose(robot, leg_model, robot.legs[i].height_current, robot.legs[i].shift_current, robot.legs[i].step_length_current, robot.legs[i].duty, 0.0);
                // assume initial on the plain
                leg_model.contact_map(robot.legs[i].eta_next[0], robot.legs[i].eta_next[1], 0.0);
                
                if(i==0 || i==3){
                    robot.legs[i].relative_foothold_current = Eigen::Vector2d(-leg_model.contact_p[0], leg_model.contact_p[1]);
                } else {
                    robot.legs[i].relative_foothold_current = Eigen::Vector2d( leg_model.contact_p[0], leg_model.contact_p[1]);
                }
                std::cout << "relative foothold: " << robot.legs[i].relative_foothold_current[0] << "," << robot.legs[i].relative_foothold_current[1] << std::endl;
            
            // set hip position
            robot.legs[0].hip_position_current = Eigen::Vector3d(robot.body_position.x() + robot.BL / 2, robot.body_position.y() + robot.BW / 2, robot.body_position.z() );
            robot.legs[1].hip_position_current = Eigen::Vector3d(robot.body_position.x() + robot.BL / 2, robot.body_position.y() - robot.BW / 2, robot.body_position.z() );
            robot.legs[2].hip_position_current = Eigen::Vector3d(robot.body_position.x() - robot.BL / 2, robot.body_position.y() - robot.BW / 2, robot.body_position.z() );
            robot.legs[3].hip_position_current = Eigen::Vector3d(robot.body_position.x() - robot.BL / 2, robot.body_position.y() + robot.BW / 2, robot.body_position.z() );
            // base on the body position
            robot.legs[0].foothold_current = Eigen::Vector3d(robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                             robot.BW / 2,
                                                             robot.legs[0].relative_foothold_current.y());
            robot.legs[1].foothold_current = Eigen::Vector3d(robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                             -robot.BW / 2,
                                                             robot.legs[0].relative_foothold_current.y());
            robot.legs[2].foothold_current = Eigen::Vector3d(-robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                             -robot.BW / 2,
                                                             robot.legs[0].relative_foothold_current.y());
            robot.legs[3].foothold_current = Eigen::Vector3d(-robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                             robot.BW / 2,
                                                             robot.legs[0].relative_foothold_current.y());
            Send(robot);
            break;
    }
}



int main(int argc, char** argv) {
    // four leg
    // 0. setup
    bool sim = true;
    LegModel leg_model(sim);
    double initial_SL = 0.25; // 初始步長
    // A. Create terrain
    double terrain_start_height = 0;
    Terrain terrain_zigzag(terrain_start_height);
    terrain_zigzag.addPlain(5);     
    // in tan() ___/\/\/\/\_________ 
    terrain_zigzag.addSlope((2/8), 0.08); 
    terrain_zigzag.addSlope(-(2/8), 0.08);  
    terrain_zigzag.addSlope((2/8), 0.08); 
    terrain_zigzag.addSlope(-(2/8), 0.08);  
    terrain_zigzag.addSlope((2/8), 0.08); 
    terrain_zigzag.addSlope(-(2/8), 0.08);  
    terrain_zigzag.addSlope((2/8), 0.08); 
    terrain_zigzag.addSlope(-(2/8), 0.08);  
    terrain_zigzag.addPlain(5);  
    // B. Create robot and legs
    double ideal_height = 0.149;
    Robot robot;
    robot.terrain = &terrain_zigzag;
    robot.body_position = Eigen::Vector3d(0.0, 0.0, ideal_height); 
    robot.CoM_pos       = Eigen::Vector2d(0.0, 0.0); // CoM 偏移位置
    robot.velocity      = 0.05;   // m/s
    robot.swing_time    = 0.2;   // 占空比
    for (int i = 0; i < 4; ++i){
        robot.legs[i].height_current = ideal_height; 
        robot.legs[i].height_next = ideal_height; 
        robot.legs[i].shift_current = 0.0; // 初始偏移
        robot.legs[i].shift_next = 0.0; // 初始偏移        
        robot.legs[i].step_length_current = initial_SL; // 初始步長
        robot.legs[i].step_length_next = initial_SL; // 初始步長
        robot.legs[i].leg_type_current = LegType::HYBRID;
        robot.legs[i].leg_type_next = LegType::HYBRID;
        // in plain -> Reserse?
        robot.legs[i].swing_type = SwingType::Rotate; 
    }
    robot.new_step_length = initial_SL;
    robot.dS            = robot.velocity / robot.pub_rate;
    robot.incre_duty    = robot.dS / initial_SL; // duty cycle 增量
    // initial_SL = swing+td
    
    // C. check setup
    // robot.terrain->printAll();

    // D. Ros
    RCLCPP_INFO(rclcpp::get_logger("CorgiHybrid"), "Test\n");
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("corgi_zigzag_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // motor state sub
    motor_state_sub_ = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("/motor/state", robot.pub_rate, motor_state_cb);
    // motor cmd pub
    motor_cmd_pub_ = nh.advertise<corgi_msgs::msg::MotorCmdStamped>("/motor/command", robot.pub_rate);
    
    rate_ptr = new rclcpp::Rate(robot.pub_rate);

    // 1. set the initial pose
    // A. Set with exiciting pose or specified pose(先讓前腳抬 -> 有機會可以調整到後腳不倒)
    // B. 足尖觸地?
    // initial 應該頁要根據type 分類
    Initialize(robot, leg_model, false, 0); 
    for(int i = 0; i < 20; ++i) {
        Send(robot);
    }
    // cout robot info
    Print_current_Info(robot);

    // // // 2. separate to fourleg
    // while(1){
    // //     // std::cout << "robot position: " << robot.body_position[0] << " , " << robot.body_position[1] << " , " << robot.body_position[2]  << std::endl;
    //     Step(robot, leg_model);
    //     Send(robot);
    // }
    
    // 3. genarate four leg traj
    // 4. sent to sim to check
    // 5. add stability 
    // 6. turn to real-time 
    // 7. finish!

    return 0;

    // find_pose() -> height+shift
    // turn all the function to execute along duty! and store the important parameter for each leg
    // 用type來決定呼叫哪一個腳型態的程式   
}



void generate_and_save_swing_trajectory(
    LegModel& leg,
    double theta_start, double beta_start,
    double theta_end,   double beta_end,
    int N,
    int rim, double alpha,
    const Eigen::Vector2d &body_vel,
    const Eigen::Vector2d &ground_tangent,
    std::ofstream& out)
{
    // 落地時的觸地點
    Eigen::Vector2d P_end = pointOnRimByAlpha(leg, theta_end, beta_end, rim, alpha);
    double raw_prev = double(N-1)/N;
    double mid_theta = 17.0 * M_PI / 180.0;
    auto compute_theta_raw = [&](double t){
        if (t < 0.5)
            return theta_start + (mid_theta - theta_start) * (2*t);
        else
            return mid_theta + (theta_end - mid_theta) * (2*(t - 0.5));
    };
    auto compute_beta_raw = [&](double t){ return beta_start + (beta_end - beta_start) * quintic(t); };
    Eigen::Vector2d P_prev = pointOnRimByAlpha(
        leg,
        compute_theta_raw(raw_prev),
        compute_beta_raw(raw_prev),
        rim, alpha
    );
     Eigen::Vector2d dPdr = (P_end - P_prev) * N;
    double denom = dPdr.dot(ground_tangent);
    double vb_t  = body_vel.dot(ground_tangent);
    double eps   = (std::abs(denom) > 1e-6 ? 1.0 - vb_t/denom : 0.0);
    eps = std::clamp(eps, -0.5, 0.5);

    // 2) warp for theta only
    auto warp_theta = [&](double r){ return std::clamp(r + eps*r*(1-r), 0.0, 1.0); };

    // constraints
    const double max_dtheta = 1.0 * M_PI/180.0; // max 1° per step
    const double lambda     = 0.3;              // beta mix weight

    double prev_theta = theta_start;

    // 3) generate trajectory
    for (int i = 0; i <= N; ++i) {
        double raw = double(i) / N;
        // theta: warped two-segment + clamp delta
        double t_th    = warp_theta(raw);
        double theta_uns = compute_theta_raw(t_th);
        double dtheta_val = theta_uns - prev_theta;
        double theta_cur  = prev_theta + std::copysign(std::min(std::abs(dtheta_val), max_dtheta), dtheta_val);

        // beta: linear+quintic on raw (no warp)
        double s_lin   = raw;
        double s_quint = quintic(raw);
        double s_beta  = lambda*s_lin + (1.0 - lambda)*s_quint;
        double beta_cur = beta_start + (beta_end - beta_start) * s_beta;

        Eigen::Vector2d p = pointOnRimByAlpha(leg, theta_cur, beta_cur, rim, alpha);
        out << std::fixed << std::setprecision(6) 
            << theta_cur << "," 
            << beta_cur << "," 
            << p.x()<<"," 
            << p.y()<<","
            << "swing"<<"\n";
        
        prev_theta = theta_cur;
    }
}


