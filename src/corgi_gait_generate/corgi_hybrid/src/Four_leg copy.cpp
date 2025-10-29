#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <vector>
#include <array>
#include <cmath>
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

    Eigen::Vector2d O_position_current; //（機器人座標）
    Eigen::Vector2d O_position_next; //（機器人座標）

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
          O_position_current (Eigen::Vector2d::Zero()),
          O_position_next    (Eigen::Vector2d::Zero()),
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
                robot.legs[i].eta_next = find_pose(robot, leg_model, robot.legs[i].height_current, robot.legs[i].shift_current, robot.legs[i].step_length_current, robot.legs[i].duty, 0.0);
                // std::cout << i << ": " << robot.legs[i].eta_next[0]<< ", " << robot.legs[i].eta_next[1] << std::endl;
            }
            Send(robot);
            break;
    }
}
Eigen::Vector2d pointOnRimByAlpha(
    LegModel &legmodel,
    double theta, double beta,
    int rim, double alpha)
{
    legmodel.forward(theta, beta, false);
    std::complex<double> O, V0;
    switch (rim) {
        case 1: O = legmodel.U_l_c; V0 = legmodel.H_l_c - O; break;
        case 2: O = legmodel.L_l_c; V0 = legmodel.F_l_c - O; break;
        case 3: O = legmodel.L_l_c; V0 = legmodel.G_c   - O; break;
        case 4: O = legmodel.L_r_c; V0 = legmodel.F_r_c - O; break;
        case 5: O = legmodel.U_r_c; V0 = legmodel.H_r_c - O; break;
        default: throw std::runtime_error("invalid rim id");
    }
    const double deg50 = 50.0 * M_PI / 180.0;
    std::complex<double> rotated;
    if      (alpha >= -M_PI   && alpha < -deg50) rotated = std::polar(1.0, alpha + M_PI)  * V0;
    else if (alpha >= -deg50  && alpha <  0.0)   rotated = std::polar(1.0, alpha + deg50) * V0;
    else if (alpha >= 0.0     && alpha <  deg50) rotated = std::polar(1.0, alpha)        * V0;
    else if (alpha >= deg50   && alpha <  M_PI)  rotated = std::polar(1.0, alpha - deg50) * V0;
    else {
        auto VG = legmodel.G_c - legmodel.L_l_c;
        auto tip = legmodel.L_l_c + (legmodel.r / legmodel.R) * std::polar(1.0, alpha) * VG;
        return { tip.real(), tip.imag() };
    }
    auto P = O + (legmodel.radius / legmodel.R) * rotated;
    return { P.real(), P.imag() };
}
double quintic(double t) { 
    return 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t; 
}
double clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
}
void generate_swing_trajectory(
    Robot &robot, LegModel& leg, int leg_index,
    double stand_h,
    double theta_start, double beta_start,
    double theta_end,   double beta_end,
    int N,
    int rim, double alpha,
    const Eigen::Vector2d &body_vel,
    const Eigen::Vector2d &ground_tangent)
{   
    robot.legs[leg_index].swing_traj.clear();
    robot.legs[leg_index].swing_traj.reserve(N + 1);
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
    eps = clamp(eps, -0.5, 0.5);

    // 2) warp for theta only
    auto warp_theta = [&](double r){ return clamp(r + eps*r*(1-r), 0.0, 1.0); };

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

        robot.legs[leg_index].swing_traj.push_back({ raw, theta_cur, beta_cur});
        // std::cout << "swing_traj[" << i << "]: " << raw << ", " << theta_cur << ", " << beta_cur << std::endl;
        prev_theta = theta_cur;

    }
}
void Hybrid_Swing_Step(Robot &robot, LegModel &leg_model, int leg_index){
    double ratio = (robot.legs[leg_index].duty - (1-robot.swing_time)) / robot.swing_time;
    ratio = clamp(ratio, 0.0, 1.0); 
    // std::cout << "ratio: " << ratio << std::endl;
    int num = robot.swing_time* robot.legs[leg_index].step_length_current / robot.dS;
    int idx = static_cast<int>(ratio * num);

    robot.legs[leg_index].eta_next[0] = robot.legs[leg_index].swing_traj[idx].theta;
    robot.legs[leg_index].eta_next[1] = robot.legs[leg_index].swing_traj[idx].beta;
};
void Hybrid_Step(Robot &robot, LegModel &leg_model, int leg_index){
    robot.legs[leg_index].hip_position_next[0] += robot.dS;
    robot.legs[leg_index].duty += robot.incre_duty;   

    /* Keep duty in the range [0, 1] */
    if (robot.legs[leg_index].duty < 0) robot.legs[leg_index].duty += 1.0; 

    /* Calculate next foothold if entering swing phase*/
    // Enter SW (calculate swing phase traj)
    if ((robot.legs[leg_index].duty > (1 - robot.swing_time)) && robot.legs[leg_index].swing_phase == 0) {
        robot.legs[leg_index].swing_phase = 1;
        auto swing_end = find_pose(robot, leg_model, robot.legs[leg_index].height_current, robot.legs[leg_index].shift_current, robot.legs[leg_index].step_length_current, 0, 0.0); 
        int N = robot.swing_time* robot.legs[leg_index].step_length_current / robot.dS;
        leg_model.forward(swing_end[0], swing_end[1], false);
        leg_model.contact_map(swing_end[0], swing_end[1], 0.0);
        int rim_id = leg_model.rim;
        double alpha0 = leg_model.alpha;
        // std::cout << "swing_end: " << swing_end[0] << ", " << swing_end[1] << std::endl;
        while (robot.legs[leg_index].eta_current[1]<swing_end[1]){
            swing_end[1]-=2*M_PI;
        }
        generate_swing_trajectory(robot, leg_model, leg_index, 
                                  robot.legs[leg_index].height_current, 
                                  robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], 
                                  swing_end[0], swing_end[1], 
                                  N, rim_id, alpha0, 
                                  Eigen::Vector2d(robot.velocity,0), Eigen::Vector2d(1,0));      
        
    } 
    // Enter TD
    else if (robot.legs[leg_index].duty > 1.0){     
        robot.legs[leg_index].swing_phase = 0;
         // Keep gaitSelector->duty in the range [0, 1]
        robot.legs[leg_index].duty -= 1.0;
    }

    /* Calculate next gaitSelector->eta */
    // calculate the nest Stance phase traj
    if (robot.legs[leg_index].swing_phase == 0) { 
        leg_model.forward(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1],true);
        robot.legs[leg_index].eta_next = leg_model.move(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], {robot.dS, 0.0}, 0.0);
    } 
    // read the next Swing phase traj
    else { 
        Hybrid_Swing_Step(robot, leg_model, leg_index);
    }

    // update 
    robot.legs[i].hip_position_current = robot.legs[i].hip_position_next;
    robot.legs[i].O_position_current = robot.legs[i].O_position_next;
    
}

// Whole process
void Step(Robot &robot, LegModel &leg_model){
    // update the body position
    robot.body_position[0] += robot.dS;
    for (int i=0 ; i<4 ; ++i){
        switch(robot.legs[i].leg_type_current){
            // update
            robot.legs[i].hip_position_next = robot.body_position + Eigen::Vector3d(robot.BL/2, robot.BW/2, 0);
            robot.legs[i].O_position_next = robot.legs[i].O_position_next + Eigen::Vector2d(robot.dS, 0.0);
            case 0:
                // std::cout << "Leg: "<< i << " ,WHEELED mode continue!"<<std::endl;
                break;
            case 1:
                // std::cout << "Leg: "<< i << " ,HYBRID mode continue!"<<std::endl;
                Hybrid_Step(robot, leg_model, i);
                break;
            case 2:
                // std::cout << "Leg: "<< i << " ,LEGGED mode continue!"<<std::endl;
                break;
            default:
                // std::cout << "Leg: "<< i << " ,TRANSFORM!"<<std::endl;
                break;
                
        }
    }
}

int main(int argc, char** argv) {
    // four leg
    // 0. setup
    bool sim = true;
    LegModel leg_model(sim);
    double initial_SL = 0.2; // 初始步長
    // A. Create terrain
    double terrain_start_height = 0;
    Terrain terrain_zigzag(terrain_start_height);
    terrain_zigzag.addPlain(0.5);     
    terrain_zigzag.addSlope(tan(deg2rad( 10)), 0.12); 
    terrain_zigzag.addSlope(tan(deg2rad(-10)), 0.12);  
    terrain_zigzag.addPlain(5);  
    // B. Create robot and legs
    Robot robot;
    robot.terrain = &terrain_zigzag;
    robot.body_position = Eigen::Vector3d(0.0, 0.0, 0.0); 
    robot.CoM_pos       = Eigen::Vector2d(0.005, 0.005); // CoM 偏移位置
    robot.velocity      = 0.1;   // m/s
    robot.swing_time    = 0.2;   // 占空比
    for (int i = 0; i < 4; ++i){
        robot.legs[i].height_current = 0.149; 
        robot.legs[i].height_next = 0.149; 
        robot.legs[i].shift_current = 0.0; // 初始偏移
        robot.legs[i].shift_next = 0.0; // 初始偏移
        robot.legs[i].step_length_current = initial_SL; // 初始步長
        robot.legs[i].step_length_next = initial_SL; // 初始步長
        robot.legs[i].leg_type_current = LegType::HYBRID;
        robot.legs[i].leg_type_next = LegType::HYBRID;
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

    // 2. separate to fourleg
    while(1){
        Step(robot, leg_model);
        Send(robot);
    }
    
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



// void Step(Robot &robot, LegModel &leg_model, double terrain_slope) {
//     for (int i=0; i<4; i++) {
//         robot.legs[i].hip_position_next[0] += robot.dS;
//         robot.legs[i].duty += robot.incre_duty;    
//     }

//     for (int i=0; i<4; i++) {
//         /* Keep duty in the range [0, 1] */
//         if (robot.legs[i].duty < 0) robot.legs[i].duty += 1.0; 

//         /* Calculate next foothold if entering swing phase*/
//         // Enter SW (calculate swing phase traj)
//         if ((robot.legs[i].duty > (1 - robot.swing_time)) && robot.legs[i].swing_phase == 0) {
//             robot.legs[i].swing_phase = 1;
            

//             swing_pose = find_pose(swing_desired_height, gaitSelector->current_shift[i], (gaitSelector->step_length*3/6), terrain_slope);  
//             Swing(gaitSelector->eta, swing_pose, swing_variation, i);      
            
//         } 
//         // Enter TD
//         else if (robot.legs[i].duty > 1.0){                  
//             robot.legs[i].swing_phase = 0;
//             robot.legs[i].duty -= 1.0; // Keep gaitSelector->duty in the range [0, 1]
//         }

//         /* Calculate next gaitSelector->eta */
//         // calculate the nest Stance phase traj
//         if (robot.legs[i].swing_phase == 0) { 
//             leg_model.forward(robot.legs[i].eta_current[0], robot.legs[i].eta_current[1],true);
//             robot.legs[i].eta_next = leg_model.move(robot.legs[i].eta_current[0], robot.legs[i].eta_current[1], {robot.dS, 0.0}, 0.0);
//         } 
//         // read the next Swing phase traj
//         else { 
//             Swing_step(swing_pose, swing_variation, i, gaitSelector->duty[i]);
//         }
//     }
// }





