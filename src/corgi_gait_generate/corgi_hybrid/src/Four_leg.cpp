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
struct SwingPoint {
    double time;
    double theta;
    double beta;
};
// === 權重設定 ===
struct Weights{
    double w_stab  = 5.0;
    double w_clr   = 2.0;
    double w_pitch = 1.0;
};
// === 得分結果結構 ===
struct PoseBest{
    double height  = 0.0;
    double shift   = 0.0;
    double score  = 1e9;
    bool is_current = false;
};
struct InertialWrench{
    Eigen::Vector3d f;   // 力  (N)
    Eigen::Vector3d n;   // 力偶(N·m)
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

    std::array<double, 2> eta_start_temp;
    std::array<double, 2> eta_end_temp;

    std::vector<SwingPoint> swing_traj;
    int rim_id;
    double alpha0;

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

    double kThetaMin = 17.0 * M_PI / 180.0;
    double kThetaMax = 160.0 * M_PI / 180.0;
    double kClrRef   = 0.05;    // 5 cm → clearance = 1
    double initial_SL = 0.2; // 初始步長
    double ideal_height = 0.149; // 初始高度
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
double clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
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
void csv(Robot &robot,LegModel &leg,std::ofstream &file2){
    // open the csv and store the data
    file2 << std::fixed << std::setprecision(6)
    << robot.legs[0].eta_current[0] << "," << robot.legs[0].eta_current[1] << ","
    << robot.legs[1].eta_current[0] << "," << robot.legs[1].eta_current[1] << ","
    << robot.legs[2].eta_current[0] << "," << robot.legs[2].eta_current[1] << ","
    << robot.legs[3].eta_current[0] << "," << robot.legs[3].eta_current[1] << ","
    << robot.legs[0].swing_phase << "," << robot.legs[1].swing_phase << ","
    << robot.legs[2].swing_phase << "," << robot.legs[3].swing_phase << ","
    << robot.legs[0].foothold_current[0] << "," << robot.legs[0].foothold_current[1] << "," << robot.legs[0].foothold_current[2] << ","
    << robot.legs[1].foothold_current[0] << "," << robot.legs[1].foothold_current[1] << "," << robot.legs[1].foothold_current[2] << ","
    << robot.legs[2].foothold_current[0] << "," << robot.legs[2].foothold_current[1] << "," << robot.legs[2].foothold_current[2] << ","
    << robot.legs[3].foothold_current[0] << "," << robot.legs[3].foothold_current[1] << "," << robot.legs[3].foothold_current[2]
    << "\n";

}

// ———— 碰撞檢查 —— 取樣扇形 A–D、G-sector 與 E 線段 ————
bool checkCollision(Robot &robot, LegModel &leg, 
                    double theta, double beta,
                    double hip_x, double stand_h, double hip_z) 
{
    // 現在地板才是 0 so all points add height of hip  
    std::complex<double> hipOff = {0.0, hip_z};
    // 1) 前向運算更新所有關節點
    leg.forward(theta, beta, true);

    const int N = 50;   // 每條弧線/線段取 N+1 個點檢查
    auto below = [&](double wx, double wy){
        // 僅檢查 hip_x ± stand_h 範圍內的地形
        if (wx < hip_x - stand_h || wx > hip_x + stand_h)
            return false;
        return wy < robot.terrain->height(wx);
    };

    // 2) 扇形取樣檢查函式
    auto sampleSector = [&](std::complex<double> C,
                            std::complex<double> P0,
                            std::complex<double> P1,
                            double R) {
        double a0 = std::atan2((P0 - C).imag(), (P0 - C).real());
        double a1 = std::atan2((P1 - C).imag(), (P1 - C).real());
        if (a1 < a0) a1 += 2*M_PI;
        for (int i = 0; i <= N; ++i) {
            double a = a0 + (a1-a0)*i/N;
            double xr = C.real() + R * std::cos(a);
            double yr = C.imag() + R * std::sin(a);
            double wx = hip_x + xr;
            double H = robot.terrain->height(wx);
            if (below(wx, yr)) return true;
        }
        return false;
    };

    // A: L_l_c → [F_l_c, G_c]
    {
        auto C = leg.L_l_c;
        double R = std::abs(leg.F_l_c - C) + leg.r ;
        if (sampleSector(C + hipOff, leg.F_l_c + hipOff, leg.G_c + hipOff, R)) return true;
    }
    // B: L_r_c → [G_c, F_r_c]
    {
        auto C = leg.L_r_c;
        double R = std::abs(leg.F_r_c - C) + leg.r ;
        if (sampleSector(C + hipOff, leg.G_c + hipOff, leg.F_r_c + hipOff, R)) return true;
    }
    // C: U_r_c → [H_r_c, F_r_c]
    {
        auto C = leg.U_r_c;
        double R = std::abs(leg.H_r_c - C) + leg.r ;
        if (sampleSector(C + hipOff, leg.H_r_c + hipOff, leg.F_r_c + hipOff, R)) return true;
    }
    // D: U_l_c → [H_l_c, F_l_c]
    {
        auto C = leg.U_l_c;
        double R = std::abs(leg.H_l_c - C) + leg.r ;
        if (sampleSector(C + hipOff, leg.H_l_c + hipOff, leg.F_l_c + hipOff, R)) return true;
    }

    // G-sector: G_c → [extend toward L_l_c, extend toward L_r_c]
    {
        auto C = leg.G_c;
        // 往 L_*_c 方向的反向延伸 leg.r
        auto dirL = (leg.L_l_c - C) / std::abs(leg.L_l_c - C);
        auto dirR = (leg.L_r_c - C) / std::abs(leg.L_r_c - C);
        std::complex<double> P0 = C - dirL * leg.r;
        std::complex<double> P1 = C - dirR * leg.r;
        if (sampleSector(C + hipOff, P0 + hipOff, P1 + hipOff, leg.r)) return true;
    }

    // E 段：U_l_c↔H_l_c、U_r_c↔H_r_c 延伸 leg.r 後取樣直線
    {
        // 延伸端點
        auto dirL = (leg.H_l_c - leg.U_l_c) / std::abs(leg.H_l_c - leg.U_l_c);
        auto dirR = (leg.H_r_c - leg.U_r_c) / std::abs(leg.H_r_c - leg.U_r_c);
        auto extL = leg.U_l_c + dirL * leg.r + hipOff;
        auto extR = leg.U_r_c + dirR * leg.r + hipOff;
        // 直線取樣
        for (int i = 0; i <= N; ++i) {
            double t = double(i)/N;
            double xr = (1-t)*extL.real() + t*extR.real();
            double yr = (1-t)*extL.imag() + t*extR.imag();
            double wx = hip_x + xr;
            if (below(wx, yr)) return true;
        }
    }
    
    return false;
}
std::pair<double,bool> cal_clearence(Robot &robot, LegModel &leg, 
                    double theta, double beta, double hip_x, double hip_z){
    leg.contact_map(theta, beta, atan(robot.terrain->slopeAt(hip_x, false)));
    double orgin_clearence = (hip_z + leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, false)));
    leg.contact_map(theta, beta, atan(robot.terrain->slopeAt(hip_x, true)));
    double next_clearence = (hip_z + leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, true)));
    if (orgin_clearence <0 || next_clearence < 0) {
        return {1e9,false}; // 不合法
    }
    return {std::min(orgin_clearence, next_clearence), orgin_clearence<next_clearence? false : true}; // 返回最小 clearance 和是否是原地
}
// ─────────────────────────────────────────────────────────────────────────────
// scoreCandidate() – 單姿態評分
// ─────────────────────────────────────────────────────────────────────────────
inline std::pair<double,bool> scoreCandidate(Robot& R, LegModel& leg,
                             double h_op, double sh_op,
                             const Weights& W,
                             int leg_id)
{
    auto& L = R.legs[leg_id];
    double x_temp = L.hip_position_current.x() + L.step_length_current*R.swing_time;
    LegModel leg_opt(true);
    auto eta_temp = leg_opt.inverse({0.0, -h_op + leg_opt.r}, "G");
    // === 硬性限制 1: θ 合法 ===
    // 後退 shift
    for (double s1 = 0; s1 < sh_op; s1 += 0.001) {
        eta_temp = leg_opt.move(eta_temp[0], eta_temp[1], { -0.001, 0.0 }, 0.0);
        if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
            return {1e9,false};
        }
    }
    // 半步長
    for (double s = 0; s < 0.5 * L .step_length_current; s += 0.001) {
        try {
            eta_temp = leg_opt.move(eta_temp[0], eta_temp[1], {-0.001, 0.0}, 0.0);
        } catch (std::runtime_error &e) {
            return {1e9,false};
        }
    
        if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
            return {1e9,false};
        }
    }
    // === 硬性限制 4: 碰撞 ===
    if (checkCollision(R, leg, eta_temp[0], eta_temp[1], x_temp, h_op, L.hip_position_current.z())){
        // std::cout << "Collision " << std::endl;
        return {1e9,false};
    }
    // === 可加權指標 1: 接觸 Clearance ===
    auto [clr, which_terrain] = cal_clearence(R, leg, eta_temp[0], eta_temp[1], x_temp, L.hip_position_current.z());
    double clr_n  = clamp((clr - R.clearence) / R.kClrRef, 0.0, 1.0);
    // TD which_terrain
    // touchdown phase
    bool which_terrain_step = which_terrain;
    // std::cout << "end td: " << x_temp +  R.swing_time* L.step_length_current<< std::endl;
    for (double t = 0.0; t <= (1.0 - R.swing_time); t += R.incre_duty) {
        x_temp += R.dS;
        eta_temp = leg.move(eta_temp[0], eta_temp[1], {R.dS, 0.0}, atan(R.terrain->slopeAt(x_temp, which_terrain_step)));
        if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
            return {1e9,false};
        }
        // === 硬性限制 4: 碰撞 ===
        if (checkCollision(R, leg, eta_temp[0], eta_temp[1], x_temp, h_op, L.hip_position_current.z())){
            return {1e9,false};
        }
        auto [clr_step, which_terrain_step] = cal_clearence(R, leg, eta_temp[0], eta_temp[1], x_temp, L.hip_position_current.z());
        // if (clr_step<0 || clr_step > R.clearence_M) {
        //     std::cout << "At x: "<< x_temp <<"clearence is not valid: " << clr_step << std::endl;
        //     return {1e9,false}; // 不合法
        // }
        clr_step  = clamp((clr_step - R.clearence) / R.kClrRef, 0.0, 1.0);
        clr_n += clr_step;
    }
    // === 加權分數 ===
    return {clr_n, which_terrain}; 
}
// ─────────────────────────────────────────────────────────────────────────────
// applyLandingPose() – 把最佳姿態寫回 Robot，並處理 18° 模式切換
// ─────────────────────────────────────────────────────────────────────────────
inline std::array<double, 2> applyLandingPose(Robot& R, int leg_idx, const PoseBest& best)
{   
    LegModel Land(true);
    auto& leg = R.legs[leg_idx];
    leg.height_next = best.height;
    leg.shift_next = best.shift;
    // apply the rim and alpha
    auto eta_temp = Land.inverse({0.0, -leg.height_next + Land.r}, "G");
    // 後退 shift
    for (double s1 = 0; s1 < leg.shift_next; s1 += 0.001) {
        eta_temp = Land.move(eta_temp[0], eta_temp[1], { -0.001, 0.0 }, 0.0);
    }
    // 前進半步長
    for (double s = 0; s < 0.5 * leg.step_length_current; s += 0.001) {
        eta_temp = Land.move(eta_temp[0], eta_temp[1], {-0.001, 0.0}, 0.0);
    }
    Land.forward(eta_temp[0], eta_temp[1], false);
    Land.contact_map(eta_temp[0], eta_temp[1], atan(R.terrain->slopeAt(leg.hip_position_current[0] + R.swing_time * leg.step_length_current, best.is_current)));
    leg.rim_id = Land.rim;
    leg.alpha0 = Land.alpha;

    return eta_temp;
}
// ─────────────────────────────────────────────────────────────────────────────
// searchLandingPose() – 為指定 swing 腿找透過尋找最佳(height, shift)再轉換成落地姿態(θ,β)
// height 從 我指定得hmax-hmin之間尋找
// shift 範圍是 [-step_length ~ step_length]
// ─────────────────────────────────────────────────────────────────────────────
inline PoseBest searchLandingPose(Robot& R, LegModel& leg_model,
                                 int leg_idx,
                                 double h_min, double h_max,
                                 double h_step = 0.005,      
                                 double shift_step = 0.005,
                                 const Weights& W = Weights{})  
{
    PoseBest best; 
    auto& L = R.legs[leg_idx];
    double hip_x = L.hip_position_current.x() + L.step_length_current*R.swing_time;

    for(double h = h_max; h >= h_min; h -= h_step){
        for(double sh = -L.step_length_current; sh <= L.step_length_current; sh += shift_step){
            auto [score, which]  = scoreCandidate(R, leg_model, h, sh, W, leg_idx);
            // std::cout << "Height: " << h << ", Shift: " << sh << ", Score: " << score << std::endl;
            if (score <= best.score){
                // 更新最佳解
                best = {h, sh, score, which};
            } 
        }
    }
    // std::cout << "Height: " << best.height << ", Shift: " << best.shift  << ", Score: " <<  best.score << std::endl;
    return best;
}

static inline double cubicHermite(double t0, double q0, double v0,
                                  double t1, double q1, double v1,
                                  double t)
{
    const double h  = t1 - t0;
    const double s  = (t - t0) / h;
    const double h00 =  2*s*s*s - 3*s*s + 1;
    const double h10 =      s*s*s - 2*s*s + s;
    const double h01 = -2*s*s*s + 3*s*s;
    const double h11 =      s*s*s -   s*s;
    return h00*q0 + h10*h*v0 + h01*q1 + h11*h*v1;
}

static void generateSegmentedHermite(Robot &robot,
                                     double theta_start, double beta_start,
                                     double theta_end,   double beta_end,
                                     const Eigen::Vector2d &q_via,
                                     unsigned N, int leg_index)
{
    for (unsigned i = 0; i <= N; ++i) {
        double t_norm = static_cast<double>(i) / N; // 0 → 1
        double theta, beta;
        // if (t_norm <= 0.5) {
        //     theta = cubicHermite(0.0, theta_start, 0.0,
        //                           0.5, q_via(0),   0.0, t_norm);
        //     beta  = cubicHermite(0.0, beta_start,  0.0,
        //                           0.5, q_via(1),   0.0, t_norm);
        // } else {
        //     theta = cubicHermite(0.5, q_via(0), 0.0,
        //                           1.0, theta_end, 0.0, t_norm);
        //     beta  = cubicHermite(0.5, q_via(1), 0.0,
        //                           1.0, beta_end,  0.0, t_norm);
        // }

        // theta = cubicHermite(0.0, theta_start, 0.0,
        //                           1.0, theta_end, 0.0, t_norm);
        //     beta  = cubicHermite(0.0, beta_start, 0.0,
        //                           1.0, beta_end,  0.0, t_norm);
        double temp = beta_end-beta_start;
        
        double temp_2 = std::max(theta_start-deg2rad(N*0.5*0.5), deg2rad(17.00));
        if (t_norm <= 0.5) {
            theta = cubicHermite(0.0, theta_start, 0.0,
                                  0.5, temp_2,   0.0, t_norm);
            beta  = cubicHermite(0.0, beta_start,  0.0,
                                  0.5, beta_start+0.2*temp,   0.0, t_norm);
        } else {
            theta = cubicHermite(0.5, temp_2, 0.0,
                                  1.0, theta_end, 0.0, t_norm);
            beta  = cubicHermite(0.5, beta_start+0.2*temp, 0.0,
                                  1.0, beta_end,  0.0, t_norm);
        }
        robot.legs[leg_index].swing_traj.push_back({ t_norm, theta, beta });
    }
}

void planSwingTrajectory(Robot& robot, int leg_index, std::array<double, 2> eta_pose){
    int N = robot.swing_time/ robot.incre_duty;
    robot.legs[leg_index].swing_traj.clear();
    robot.legs[leg_index].swing_traj.reserve(N + 1);
    Eigen::Vector2d q_via;
    // std::cout << "eta_pose: " << rad2deg(eta_pose[0]) << ", " << rad2deg(eta_pose[1]) << std::endl;
    while (robot.legs[leg_index].eta_current[1]<eta_pose[1]){
                eta_pose[1] -= 2.0*M_PI;
    }
    q_via(1) = 0.5 * (robot.legs[leg_index].eta_current[1] + eta_pose[1]);   // β midpoint
    q_via(0) = 17.0 *M_PI / 180.0;
    generateSegmentedHermite(robot, robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1],
                             eta_pose[0], eta_pose[1], q_via, N, leg_index);

}

inline std::vector<int> makeSupportSet( Robot& robot)
{
    std::vector<int> S;
    for (int i = 0; i < 4; ++i)
        if (robot.legs[i].swing_phase == 0)      // 0 = stance
            S.push_back(i);                      // 3 或 4 腳
    return S;
}

inline Eigen::Vector3d getCOMProjected(Robot& R)
{
    Eigen::Vector3d p_com = {R.body_position.x() + R.CoM_pos.x(),
                             R.body_position.y() + R.CoM_pos.y(),
                             R.body_position.z() };    // 斜坡或不平整
    // p_com.z() = R.terrain->height(p_com[0]); // 垂直投影
    // std::cout << "COM projected: " << p_com.transpose() << std::endl;
    return p_com;
}

inline double poly3(double c3,double c2,double c1,double c0,double x){
    return ((c3*x + c2)*x + c1)*x + c0;
}

InertialWrench calcLegInertia(Robot& R,double duty, int swing_leg){
    // std::cout << "calcLegInertia " << std::endl;
    double dt = 1/R.pub_rate;
    int time_point = ((duty - (1-R.swing_time)) / R.swing_time) * R.legs[swing_leg].swing_traj.size();
    // std::cout << "time_point " << time_point << std::endl;
     /* === 1. 取 theta_Iner,  θdot, θddot (二階差分) === */
    //  std::cout << "theta " << std::endl;
    double theta_swing   = R.legs[swing_leg].swing_traj[time_point].theta;
    double beta_swing   = R.legs[swing_leg].swing_traj[time_point].beta;
    double R_m_2 = 6 * 4.320353e-08 *theta_swing; 
    double R_m_1 = 3 * 4.320353e-08 *theta_swing *theta_swing - 2 * 4.195110e-04 *theta_swing - 1.175735e-05;
    double inertia = poly3(4.930266e-03,5.096802e-03,5.014408e-03,5.177460e-03, theta_swing);
    double inertia_1 = 3 * 4.930266e-03 *theta_swing *theta_swing + 2 * 5.096802e-03 *theta_swing + 5.014408e-03;
    double theta_d;
    double theta_dd;
    double beta_d;
    double beta_dd;
    if (time_point ==0){
        theta_d  = (R.legs[swing_leg].swing_traj[1].theta -   R.legs[swing_leg].swing_traj[0].theta ) / (dt);
        theta_dd = (R.legs[swing_leg].swing_traj[0].theta - 2*R.legs[swing_leg].swing_traj[1].theta + R.legs[swing_leg].swing_traj[2].theta)/(dt*dt);
        beta_d  = (R.legs[swing_leg].swing_traj[1].beta -   R.legs[swing_leg].swing_traj[0].beta ) / (dt);
        beta_dd = (R.legs[swing_leg].swing_traj[0].beta - 2*R.legs[swing_leg].swing_traj[1].beta + R.legs[swing_leg].swing_traj[2].beta)/(dt*dt);
    }
    else if (time_point == R.legs[swing_leg].swing_traj.size()){
        int num = R.legs[swing_leg].swing_traj.size();
        theta_d  = (R.legs[swing_leg].swing_traj[num].theta - R.legs[swing_leg].swing_traj[num-1].theta ) / (dt);
        theta_dd = (R.legs[swing_leg].swing_traj[num-2].theta - 2*R.legs[swing_leg].swing_traj[num-1].theta + R.legs[swing_leg].swing_traj[num].theta)/(dt*dt);
        beta_d  = (R.legs[swing_leg].swing_traj[num].beta - R.legs[swing_leg].swing_traj[num-1].beta ) / (dt);
        beta_dd = (R.legs[swing_leg].swing_traj[num-2].beta - 2*R.legs[swing_leg].swing_traj[num-1].beta + R.legs[swing_leg].swing_traj[num].beta)/(dt*dt);
    }
    else{
        theta_d  = (R.legs[swing_leg].swing_traj[time_point+1].theta - R.legs[swing_leg].swing_traj[time_point-1].theta ) / (2*dt);
        theta_dd = (R.legs[swing_leg].swing_traj[time_point+1].theta - 2*theta_swing + R.legs[swing_leg].swing_traj[time_point-1].theta)/(dt*dt);
        beta_d  = (R.legs[swing_leg].swing_traj[time_point+1].beta - R.legs[swing_leg].swing_traj[time_point-1].beta ) / (2*dt);
        beta_dd = (R.legs[swing_leg].swing_traj[time_point+1].beta - 2*theta_swing + R.legs[swing_leg].swing_traj[time_point-1].beta)/(dt*dt);
    }

    double fee_or = R.m_leg * (R_m_2*theta_d*theta_d+R_m_1*theta_dd);
    double nee = inertia * beta_dd + inertia_1 * beta_d ;
    
    Eigen::Vector3d fee = { -fee_or * cos(beta_swing), 
                            -fee_or * sin(beta_swing), 
                            0.0 }; // 垂直於腿的方向
    Eigen::Vector3d nee_vec = { 0.0, nee,0.0 }; // y軸

    // a3 = 4.320353e-08;
    // a1 = -1.175735e-05;
    // a2 = -4.195110e-04;
    // a0 = 1.581175e-02;

    // b3 = 4.930266e-03;
    // b1 = 5.014408e-03;
    // b2 = 5.096802e-03;
    // b0 = 5.177460e-03;

    return { fee, nee_vec };
}

// ============================================================================
// assembleResultantWrench()
//   • 依 FASM (Reaction Wrench Theory) 組合整機的等效慣性力 (f) 與力偶 (n)
//   • 需先定義：Robot.legs[k].swing_phase==1 代表該腳處於 swing
//   • 依據 calcLegInertia() 取得每支 swing 腿的慣性項
// ============================================================================
inline InertialWrench assembleResultantWrench(Robot& R, double duty, int swing_leg)
{
    // std::cout << "Assembling resultant wrench for swing leg: " << swing_leg << std::endl;
    InertialWrench out; // 結果慣性力偶
    out.f = Eigen::Vector3d::Zero(); // 初始化慣性力
    out.n = Eigen::Vector3d::Zero(); // 初始化慣性力偶
    // --- 0. 重力 ------------------------------------------------------------
    out.f = { 0.0, 0.0, -R.mass * R.g };
    out.n = { 0.0, 0.0, 0.0 };

    // --- 1. COM 慣性 (-m a_COM) --------------------------------------------
    // out.f -= R.mass * Eigen::Vector3d::Zero();   // 機身等速

    // --- 2. swing 腿的慣性貢獻 ----------------------------------------
    const auto& L = R.legs[swing_leg];
    // // 取當前 swing 軌跡 & 時間點
    // const auto& traj   = L.swing_traj;
    // size_t idx         = std::min(L.swing_phase_idx,
    //                                 traj.size() - 2); // 保底

    // 每腿慣性力偶
    // std::cout << "Assembling inertial " << std::endl;
    InertialWrench w = calcLegInertia(R, duty, swing_leg);
    out.f += w.f;
    out.n += w.n;
    // std::cout << "end " << std::endl;
    return out;

}


double stabilityScore(Robot& robot, double duty, int swing_leg)
{
    /* ---------- 1. 支撐腳索引 ---------- */
    std::vector<int> support = makeSupportSet(robot);
    const int n = support.size();
    // std::cout << "support size: " << n << std::endl;
    // for (int i = 0; i < n; ++i) {
    //     std::cout << "support[" << i << "]: " << support[i] << std::endl;
    // }
    // creat an array with size n
    std::vector<double> psi_array(n);
    double psi_min;
    if (n < 3) return -1; // 無支撐三角形
    if (n==4){
        if (duty == 0.8){
            // std::cout << "here" << std::endl;
            // // 揮腳前一刻
            // // 1. 確認四角觸地支撐三角形
            Eigen::Vector3d pc  = getCOMProjected(robot);
            // // fr = gravity (only) in this case
            Eigen::Vector3d fr = { 0.0, 0.0, -robot.mass * robot.g };
            // // nr = no other force, so no torque
            Eigen::Vector3d nr = {0,0,0};
            for (int k = 0; k < n; ++k) {
                    Eigen::Vector3d pi = robot.legs[support[k] ].foothold_current;
                    Eigen::Vector3d pj = robot.legs[support[(k+1)%n]].foothold_current;

                    /* --- 翻覆軸 a_i 與單位向量 â_i --- */
                    Eigen::Vector3d ai = pj - pi;
                    Eigen::Vector3d a = ai.normalized();
                    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
                    /* ---- (4.2-5) ---- */
                    Eigen::Vector3d li = (I - a*a.transpose())*(pj-pc);
                    
                    /* ---- (4.2-10) ---- */
                    Eigen::Vector3d fi = (I - a*a.transpose())*fr;

                    /* ---- (4.2-11) ： n_i = (a aᵀ) n_r ---- */
                    Eigen::Vector3d ni = (a*a.transpose())*nr;

                    /* ---- (4.2-12) ： l  ---- */
                    Eigen::Vector3d l = li.normalized();

                    /* ---- (4.2-13) ： f_ni = i*n_i/ li.norm() ---- */
                    Eigen::Vector3d f_ni = (l.cross(ni))/li.norm();

                    /* ---- (4.2-14)  f*_i = f_i + (a × n_i)/|a| ---- */
                    Eigen::Vector3d fi_star = fi + f_ni;

                    /* ---- (4.2-15) fi_star_h =  fi_star.normalized() ---- */
                    Eigen::Vector3d fi_star_h = fi_star.normalized();

                    /* ---- (4.2-16) di =  li - (li_trans.dot(fi_star_h)*fi_star_h) ---- */
                    Eigen::Vector3d di = li - (li.dot(fi_star_h)*fi_star_h);

                    /* ---- (4.2-17)  get simbol  ---- */
                    double sigma = (fi_star.cross(li).dot(a)>=0)?1:-1;
                    /* ---- (4.2-18)  get theta  ---- */
                    double theta = sigma *std::acos( fi_star_h.dot(l));
                    /* ---- (4.2-19)  get psi  ---- */
                    double psi = theta*di.norm()*fi_star.norm();
                    // std::cout << "psi: " << psi << std::endl;
                    psi_array[k] = psi;
            }
            psi_min = *std::min_element(psi_array.begin(), psi_array.end());
            if (psi_min > 0.087)           // 5°(≈0.087 rad) 以上視為安全
                ROS_INFO_STREAM("Stable in 4 leg! score = " << psi_min << " rad");
            else
                ROS_WARN_STREAM("Unstable in 4 leg ! score = " << psi_min << " rad");
            // 2. 確認離地後勝三隻腳也是穩的
            support.erase(
                std::remove(support.begin(), support.end(), swing_leg),
                support.end()
            );
            const int m = support.size();
            // std::cout << "support size after remove swing leg: " << support.size() << std::endl;
            // std::cout << "support legs: ";
            // for (int i = 0; i < support.size(); ++i) {
            //     std::cout << support[i] << " ";
            // }
            std::vector<double> psi_array(m);
            for (int k = 0; k < m; ++k) {
                Eigen::Vector3d pi = robot.legs[support[k] ].foothold_current;
                Eigen::Vector3d pj = robot.legs[support[(k+1)%m]].foothold_current;
                // std::cout << "pi: " << pi.transpose() << ", pj: " << pj.transpose() << std::endl;


                /* --- 翻覆軸 a_i 與單位向量 â_i --- */
                Eigen::Vector3d ai = pj - pi;
                Eigen::Vector3d a = ai.normalized();
                Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
                /* ---- (4.2-5) ---- */
                Eigen::Vector3d li = (I - a*a.transpose())*(pj-pc);
                
                /* ---- (4.2-10) ---- */
                Eigen::Vector3d fi = (I - a*a.transpose())*fr;

                /* ---- (4.2-11) ： n_i = (a aᵀ) n_r ---- */
                Eigen::Vector3d ni = (a*a.transpose())*nr;

                /* ---- (4.2-12) ： l  ---- */
                Eigen::Vector3d l = li.normalized();

                /* ---- (4.2-13) ： f_ni = i*n_i/ li.norm() ---- */
                Eigen::Vector3d f_ni = (l.cross(ni))/li.norm();

                /* ---- (4.2-14)  f*_i = f_i + (a × n_i)/|a| ---- */
                Eigen::Vector3d fi_star = fi + f_ni;

                /* ---- (4.2-15) fi_star_h =  fi_star.normalized() ---- */
                Eigen::Vector3d fi_star_h = fi_star.normalized();

                /* ---- (4.2-16) di =  li - (li_trans.dot(fi_star_h)*fi_star_h) ---- */
                Eigen::Vector3d di = li - (li.dot(fi_star_h)*fi_star_h);

                /* ---- (4.2-17)  get simbol  ---- */
                double sigma = (fi_star.cross(li).dot(a)>=0)?1:-1;
                /* ---- (4.2-18)  get theta  ---- */
                double theta = sigma *std::acos( fi_star_h.dot(l));
                /* ---- (4.2-19)  get psi  ---- */
                double psi = theta*di.norm()*fi_star.norm();
                // std::cout <<"LEG" << k <<"psi: " << psi << std::endl;
                psi_array[k] = psi;
            }
            psi_min = *std::min_element(psi_array.begin(), psi_array.end());
        }
        else{
            // std::cout << " oh no here" << std::endl;
            // // 1. 確認四角觸地支撐三角形
            Eigen::Vector3d pc  = getCOMProjected(robot);
            // fr = gravity (only) in this case
            Eigen::Vector3d fr = { 0.0, 0.0, -robot.mass * robot.g };
            // nr = no other force, so no torque
            Eigen::Vector3d nr = {0,0,0};
            for (int k = 0; k < n; ++k) {
                    Eigen::Vector3d pi = robot.legs[support[k] ].foothold_current;
                    Eigen::Vector3d pj = robot.legs[support[(k+1)%n]].foothold_current;

                    /* --- 翻覆軸 a_i 與單位向量 â_i --- */
                    Eigen::Vector3d ai = pj - pi;
                    Eigen::Vector3d a = ai.normalized();
                    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
                    /* ---- (4.2-5) ---- */
                    Eigen::Vector3d li = (I - a*a.transpose())*(pj-pc);
                    
                    /* ---- (4.2-10) ---- */
                    Eigen::Vector3d fi = (I - a*a.transpose())*fr;

                    /* ---- (4.2-11) ： n_i = (a aᵀ) n_r ---- */
                    Eigen::Vector3d ni = (a*a.transpose())*nr;

                    /* ---- (4.2-12) ： l  ---- */
                    Eigen::Vector3d l = li.normalized();

                    /* ---- (4.2-13) ： f_ni = i*n_i/ li.norm() ---- */
                    Eigen::Vector3d f_ni = (l.cross(ni))/li.norm();

                    /* ---- (4.2-14)  f*_i = f_i + (a × n_i)/|a| ---- */
                    Eigen::Vector3d fi_star = fi + f_ni;

                    /* ---- (4.2-15) fi_star_h =  fi_star.normalized() ---- */
                    Eigen::Vector3d fi_star_h = fi_star.normalized();

                    /* ---- (4.2-16) di =  li - (li_trans.dot(fi_star_h)*fi_star_h) ---- */
                    Eigen::Vector3d di = li - (li.dot(fi_star_h)*fi_star_h);

                    /* ---- (4.2-17)  get simbol  ---- */
                    double sigma = (fi_star.cross(li).dot(a)>=0)?1:-1;
                    /* ---- (4.2-18)  get theta  ---- */
                    double theta = sigma *std::acos( fi_star_h.dot(l));
                    /* ---- (4.2-19)  get psi  ---- */
                    double psi = theta*di.norm()*fi_star.norm();
                    // std::cout << "psi: " << psi << std::endl;
                    if (psi< psi_min){
                        psi_min = psi;
                    }
            }
        }
    }
    else{
        // std::cout << "start COM" << std::endl;
        /* ---------- 2. 依 COM 投影距離排序 ---------- */
        Eigen::Vector3d pc  = getCOMProjected(robot);
        // std::cout << "start Wrench" << std::endl;
        // InertialWrench wrench = assembleResultantWrench(robot, duty, swing_leg);
        // Eigen::Vector3d fr = wrench.f;
        // // std::cout << "fr: " << fr.transpose() << std::endl;
        // Eigen::Vector3d nr = wrench.n;
        // // std::cout << "nr: " << nr.transpose() << std::endl;

        // // fr = gravity (only) in this case
        Eigen::Vector3d fr = { 0.0, 0.0, -robot.mass * robot.g };
        // // nr = no other force, so no torque
        Eigen::Vector3d nr = {0,0,0};


        for (int k = 0; k < n; ++k) {
            // std::cout << "start " << k << std::endl;
            Eigen::Vector3d pi = robot.legs[support[k] ].foothold_current;
            // std::cout << "pi: " << pi.transpose() << std::endl;
            Eigen::Vector3d pj = robot.legs[support[(k+1)%n]].foothold_current;
            // std::cout << "pj: " << pj.transpose() << std::endl;

            /* --- 翻覆軸 a_i 與單位向量 â_i --- */
            Eigen::Vector3d ai = pj - pi;
            // std::cout << "ai: " << ai.transpose() << std::endl;
            Eigen::Vector3d a = ai.normalized();
            // std::cout << "a: " << a.transpose() << std::endl;
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            // std::cout << "I: " << I << std::endl;
            /* ---- (4.2-5) ---- */
            Eigen::Vector3d li = (I - a*a.transpose())*(pj-pc);
            // std::cout << "li: " << li.transpose() << std::endl;
            /* ---- (4.2-10) ---- */
            Eigen::Vector3d fi = (I - a*a.transpose())*fr;
            // std::cout << "fi: " << fi.transpose() << std::endl;
            /* ---- (4.2-11) ： n_i = (a aᵀ) n_r ---- */
            Eigen::Vector3d ni = (a*a.transpose())*nr;
            // std::cout << "ni: " << ni.transpose() << std::endl;
            /* ---- (4.2-12) ： l  ---- */
            Eigen::Vector3d l = li.normalized();
            // std::cout << "l: " << l.transpose() << std::endl;
            /* ---- (4.2-13) ： f_ni = i*n_i/ li.norm() ---- */
            Eigen::Vector3d f_ni = (l.cross(ni))/li.norm();
            // std::cout << "f_ni: " << f_ni.transpose() << std::endl;
            /* ---- (4.2-14)  f*_i = f_i + (a × n_i)/|a| ---- */
            Eigen::Vector3d fi_star = fi + f_ni;
            // std::cout << "fi_star: " << fi_star.transpose() << std::endl;
            /* ---- (4.2-15) fi_star_h =  fi_star.normalized() ---- */
            Eigen::Vector3d fi_star_h = fi_star.normalized();
            // std::cout << "fi_star_h: " << fi_star_h.transpose() << std::endl;
            /* ---- (4.2-16) di =  li - (li_trans.dot(fi_star_h)*fi_star_h) ---- */
            Eigen::Vector3d di = li - (li.dot(fi_star_h)*fi_star_h);
            // std::cout << "di: " << di.transpose() << std::endl;
            /* ---- (4.2-17)  get simbol  ---- */
            double sigma = (fi_star.cross(li).dot(a)>=0)?1:-1;
            // std::cout << "sigma: " << sigma << std::endl;
            /* ---- (4.2-18)  get theta  ---- */
            double theta = sigma *std::acos( fi_star_h.dot(l));
            // std::cout << "theta: " << theta << std::endl;
            /* ---- (4.2-19)  get psi  ---- */
            double psi = theta*di.norm()*fi_star.norm();
            // std::cout << "psi: " << psi << std::endl;
            psi_array[k] = psi;
        }
        psi_min = *std::min_element(psi_array.begin(), psi_array.end());
    }
    // psi_min >0 穩定
    //         ＝0 臨界
    //         <0 不穩定
    // psi_min >=5 degree (≈0.087 rad) （安全）
    return psi_min;
}

// ─────────────────────────────────────────────────────────────────────────────
// adjustTripodPose() – 調整 3 支撐腿 (height, shift) 以提高穩定度 ψmin
//   1. 取得支撐腿索引（除了 swing_idx 那隻之外）
//   2. 依 COM 投影距離由大到小排序 → 先調整影響最大者
//   3. 對每腿在 (h, shift) 網格上做 greedy coordinate-descent：
//        - 若分數提高就立即採用新 (h,shift)
//   4. 揮腳階段每一步都確認完畢
//   5. 呼叫時須保證 scoreCandidate()、Weights 等皆已定義
// ─────────────────────────────────────────────────────────────────────────────
inline double adjustTripodPose(Robot& R, LegModel& leg_model,
                             int swing_idx, double duty)
{
    // 計算目前的穩定度
    bool stability = false;
    // std::cout << "start calculation " << std::endl;
    double psi_min = stabilityScore(R, duty, swing_idx);
    // psi = s（N·m）
    if (psi_min >= 0 ) {         // 5°(≈0.087 rad) 以上視為安全
        ROS_INFO_STREAM("Stable!  score = " << psi_min << "  Nm");
        stability = true;
    }
    else{
        ROS_WARN_STREAM("Unstable! score = " << psi_min << "  Nm");
    }
    return psi_min;

}

void optimizeCycle(Robot& R, LegModel& leg_model, int swing_id)
{   
    double hip_x_temp = R.legs[swing_id].hip_position_current[0] + R.swing_time * R.legs[swing_id].step_length_current;
    double terrain_height_temp = R.terrain->height(hip_x_temp);
    double h_max = R.ideal_height-terrain_height_temp + R.clearence_M;
    double h_min = R.ideal_height-terrain_height_temp -R.clearence_M;
    // === A. 落地點搜尋 ===
    PoseBest best = searchLandingPose(R, leg_model, swing_id,h_min, h_max, 0.005, 0.005); 
    std::cout << "Best Height: " << best.height << ", Best Shift: " << best.shift << ", Score: " << best.score << ", terrain contact: " << best.is_current << std::endl;
    // 寫回該腳 eta_next / foothold_next ...
    auto pose = applyLandingPose(R, swing_id, best);
    
    // === B. 產生揮腳軌跡 ===
    planSwingTrajectory(R, swing_id, pose);
    
    // === C. 調整三腳支撐姿態 ===
    double score = adjustTripodPose(R, leg_model, swing_id, R.legs[swing_id].duty);
    // std::cout << "Stability Score: " << score << std::endl;
}

void Initialize(Robot &robot, LegModel &leg_model, bool swing_index, int set_type,std::ofstream &file2) {
    // set_type = set original pose or not
    // 0 = 設置哪隻前腳先; 1 = 讀取當前姿態並設定duty ; 1 = 讀取當前姿態並繼承參數 
    // (最佳化要確定穩定-至少到下一隻腳抬腳前)
    switch(set_type){
        case 0:
            std::array<double, 4>  temp_duty;
            int swing=1;
            if (!swing_index){
                // LF 
                std::cout << "LF first" << std::endl;
                temp_duty ={(1-robot.swing_time), (0.5-robot.swing_time), 0.5, 0.0};
                swing = 0;
            } 
            else{
                // RF
                std::cout << " RF first" << std::endl;
                temp_duty ={(0.5-robot.swing_time), (1-robot.swing_time), 0.0, 0.5};
            }
            for (int i = 0; i < 4; ++i) {
                robot.legs[i].duty = temp_duty[i];
                robot.legs[i].swing_phase = 0;
            }
            for (int i = 0; i < 4; ++i) {
                robot.legs[i].eta_next = find_pose(robot, leg_model, robot.legs[i].height_current, robot.legs[i].shift_current, robot.legs[i].step_length_current, robot.legs[i].duty, 0.0);
                // assume initial on the plain
                auto [gap, which_] = cal_clearence(robot, leg_model, robot.legs[i].eta_next[0], robot.legs[i].eta_next[1], robot.legs[i].hip_position_current[0] , robot.legs[i].hip_position_current[2]);
                leg_model.contact_map(robot.legs[i].eta_next[0], robot.legs[i].eta_next[1], atan(robot.terrain->slopeAt(robot.legs[i].hip_position_current[0], which_)));
                // deal with the foot point
                if(i==0 || i==3){
                    robot.legs[i].relative_foothold_current = Eigen::Vector2d(-leg_model.contact_p[0], leg_model.contact_p[1]);
                    
                } else {
                    robot.legs[i].relative_foothold_current = Eigen::Vector2d( leg_model.contact_p[0], leg_model.contact_p[1]);
                } 
                robot.legs[i].hip_position_current[2] = -robot.legs[i].relative_foothold_current[1];
            }
            // base on the body position
            robot.legs[0].foothold_current = Eigen::Vector3d(robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                             robot.BW / 2,
                                                             robot.terrain->height(robot.BL / 2 - robot.legs[0].relative_foothold_current.x()) );
            robot.legs[1].foothold_current = Eigen::Vector3d(robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                             -robot.BW / 2,
                                                             robot.terrain->height(robot.BL / 2 - robot.legs[1].relative_foothold_current.x()) );
            robot.legs[2].foothold_current = Eigen::Vector3d(-robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                             -robot.BW / 2,
                                                             robot.terrain->height(-robot.BL / 2 - robot.legs[2].relative_foothold_current.x()) );
            robot.legs[3].foothold_current = Eigen::Vector3d(-robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                             robot.BW / 2,
                                                             robot.terrain->height(-robot.BL / 2 - robot.legs[3].relative_foothold_current.x()) );
            
            // 進行最佳化
            // （目標： 調整四腳的姿態）
            // 1. for 下一刻要揮腳的腳 要確定離地後剩下三隻腳是穩定的（可以變換 該腳的height, shift來調整姿態)
            // 2. for 下一刻要揮腳的腳 揮腳期間也需要是穩定的
            // 3. for 下一刻要揮腳的腳 揮腳後落地時有找到姿態確定至少到下次自己揮腳前都可以持續觸地（姿態都找的到）
            // 4. 持續觸地腳則需要確定至少到下次自己揮腳前都可以持續觸地（姿態都找的到）    
            // optimizeCycle(robot, leg_model, swing);
            // current to next
            // hip , foot, relative foothold
            for (int i = 0; i < 4; ++i) {
                robot.legs[i].hip_position_next = robot.legs[i].hip_position_current;
                robot.legs[i].foothold_next = robot.legs[i].foothold_current;
                robot.legs[i].relative_foothold_next = robot.legs[i].relative_foothold_current;
            }
            break;
        }
           
}

void Hybrid_Swing_Step(Robot &robot, int leg_index){
    double ratio = (robot.legs[leg_index].duty - (1-robot.swing_time)) / robot.swing_time;
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
        robot.legs[leg_index].eta_start_temp = robot.legs[leg_index].eta_current;
        robot.legs[leg_index].swing_phase = 1;
        // TDL swing with opt result
        robot.legs[leg_index].height_current = robot.legs[leg_index].height_next;
        robot.legs[leg_index].shift_current = robot.legs[leg_index].shift_next;
        
        auto swing_end = find_pose(robot, leg_model, robot.legs[leg_index].height_current, robot.legs[leg_index].shift_current, robot.legs[leg_index].step_length_current, 0, 0.0); 
        
        leg_model.forward(swing_end[0], swing_end[1], false);
        leg_model.contact_map(swing_end[0], swing_end[1], 0.0);
        robot.legs[leg_index].rim_id = leg_model.rim;
        robot.legs[leg_index].alpha0 = leg_model.alpha;
        // std::cout << "swing_end: " << swing_end[0] << ", " << swing_end[1] << std::endl;
        while (robot.legs[leg_index].eta_current[1]<swing_end[1]){
            swing_end[1]-=2*M_PI;
        }
        planSwingTrajectory(robot, leg_index, swing_end);
        robot.legs[leg_index].eta_end_temp = swing_end;
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
        bool check_csv = false;
        double contactA;
        double contactB;
        // if touching the same slope
        leg_model.contact_map(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_next[0], false)));
        contactA = -leg_model.contact_p[1]+ robot.terrain->height(leg_model.contact_p[0]+ robot.legs[leg_index].hip_position_next[0])-robot.terrain->terrain_start_height;
        leg_model.contact_map(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_next[0], true)));
        contactB = -leg_model.contact_p[1]+ robot.terrain->height(leg_model.contact_p[0]+ robot.legs[leg_index].hip_position_next[0])-robot.terrain->terrain_start_height;
        // check_csv = robot.terrain->compare_slope(contactA, contactB);
        if (contactB>contactA){check_csv = true;} 
        else {check_csv = false;}
    
        if (check_csv){
            robot.legs[leg_index].eta_next = leg_model.move(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], {robot.dS, 0.0}, atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_next[0], false)));
            leg_model.contact_map(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_next[0], false)));

        } else {
            robot.legs[leg_index].eta_next  = leg_model.move(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], {robot.dS, 0.0},  atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_next[0], true)));
            leg_model.contact_map(robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1],  atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_next[0], true)));
        }
        if(leg_index==0 || leg_index==3){
            robot.legs[leg_index].relative_foothold_current = Eigen::Vector2d(-leg_model.contact_p[0], leg_model.contact_p[1]);
        } else {
            robot.legs[leg_index].relative_foothold_current = Eigen::Vector2d( leg_model.contact_p[0], leg_model.contact_p[1]);
        }
        robot.legs[leg_index].hip_position_current[2] = -robot.legs[leg_index].relative_foothold_current[1];

        switch(leg_index){
            case 0:
                 robot.legs[0].foothold_current = Eigen::Vector3d(robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                            robot.BW / 2,
                                                            robot.terrain->height(robot.BL / 2 - robot.legs[0].relative_foothold_current.x()) );
                break;
            case 1:
                robot.legs[1].foothold_current = Eigen::Vector3d(robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                            -robot.BW / 2,
                                                            robot.terrain->height(robot.BL / 2 - robot.legs[1].relative_foothold_current.x()) );
                break;
            case 2:
                robot.legs[2].foothold_current = Eigen::Vector3d(-robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                            -robot.BW / 2,
                                                            robot.terrain->height(-robot.BL / 2 - robot.legs[2].relative_foothold_current.x()) );
                break;
            case 3:
                robot.legs[3].foothold_current = Eigen::Vector3d(-robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                            robot.BW / 2,
                                                            robot.terrain->height(-robot.BL / 2 - robot.legs[3].relative_foothold_current.x()) );
                break;
        }
    } 
    // read the next Swing phase traj
    else { 
        Hybrid_Swing_Step(robot, leg_index);
    }    
}

void Step(Robot &robot, LegModel &leg_model, std::ofstream &file2){
    
    // update the body position
    robot.body_position[0] += robot.dS;
    for (int i=0 ; i<4 ; ++i){
        switch(robot.legs[i].leg_type_current){
            // update
            robot.legs[i].hip_position_next = robot.body_position + Eigen::Vector3d(robot.BL/2, robot.BW/2, 0);
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

    // update (next -> stablilti adjust(move) -> current)
    // hip , foot, relative foothold, eta
    // swing: alpha, 
    
    // calculaye stability
    int index = 0;
    for (int k =0;k<4;k++){
        if(robot.legs[k].swing_phase == 1){
            index = k;
            break;
        } 
    }
    // std::cout << "Swing leg index: " << index << std::endl;
    // adjustTripodPose(robot, leg_model, index, robot.legs[index].duty);
    // std::cout << "adjustTripodPose"<<std::endl;
    // to current (eta in send() )
    for(int p =0; p<4; p++){
        robot.legs[p].hip_position_current = robot.legs[p].hip_position_next;
        robot.legs[p].foothold_current = robot.legs[p].foothold_next;
        robot.legs[p].relative_foothold_current = robot.legs[p].relative_foothold_next;
    }
    // adjustTripodPose(robot, leg_model, index, robot.legs[index].duty);
    // robot.legs[leg_index].hip_position_current = robot.legs[leg_index].hip_position_next;

    csv(robot, leg_model, file2);
}

int main(int argc, char** argv) {
    std::ofstream file2("opt_0612_2.csv");
    file2 << "theta0,beta0,theta1,beta1,theta2,beta2,theta3,beta3,swing_phase0,swing_phase1,swing_phase2,swing_phase3,foothold0_x,foothold0_y,foothold0_z,foothold1_x,foothold1_y,foothold1_z,foothold2_x,foothold2_y,foothold2_z,foothold3_x,foothold3_y,foothold3_z\n";
    // four leg
    // 0. setup
    bool sim = true;
    LegModel leg_model(sim);
    
    // A. Create terrain
    double terrain_start_height = 0;
    Terrain terrain_zigzag(terrain_start_height);
    terrain_zigzag.addPlain(10);  
    terrain_zigzag.addSlope(0.2593, 0.27); 
    terrain_zigzag.addPlain(0.4); 
    terrain_zigzag.addSlope(-0.2593, 0.27);  
    terrain_zigzag.addPlain(0.4); 
    terrain_zigzag.addSlope(0.2593, 0.27); 
    terrain_zigzag.addPlain(0.4); 
    terrain_zigzag.addSlope(-0.2593, 0.27); 
    terrain_zigzag.addPlain(5);  
    // B. Create robot and legs
    Robot robot;
    robot.terrain = &terrain_zigzag;
    robot.body_position = Eigen::Vector3d(0.0, 0.0, robot.ideal_height); 
    robot.CoM_pos       = Eigen::Vector2d(0, 0); // CoM 偏移位置
    robot.velocity      = 0.05;   // m/s
    robot.swing_time    = 0.2;   // 占空比
    // set hip position
    robot.legs[0].hip_position_current = Eigen::Vector3d(robot.body_position.x() + robot.BL / 2, robot.body_position.y() + robot.BW / 2, robot.body_position.z() );
    robot.legs[1].hip_position_current = Eigen::Vector3d(robot.body_position.x() + robot.BL / 2, robot.body_position.y() - robot.BW / 2, robot.body_position.z() );
    robot.legs[2].hip_position_current = Eigen::Vector3d(robot.body_position.x() - robot.BL / 2, robot.body_position.y() - robot.BW / 2, robot.body_position.z() );
    robot.legs[3].hip_position_current = Eigen::Vector3d(robot.body_position.x() - robot.BL / 2, robot.body_position.y() + robot.BW / 2, robot.body_position.z() );
    for (int i = 0; i < 4; ++i){
        robot.legs[i].height_current = robot.ideal_height; 
        robot.legs[i].height_next = robot.ideal_height; 
        robot.legs[i].shift_current = 0.03; // 初始偏移
        robot.legs[i].shift_next = 0.03; // 初始偏移        
        robot.legs[i].step_length_current = robot.initial_SL; // 初始步長
        robot.legs[i].step_length_next = robot.initial_SL; // 初始步長
        robot.legs[i].leg_type_current = LegType::HYBRID;
        robot.legs[i].leg_type_next = LegType::HYBRID;
        robot.legs[i].swing_type = SwingType::Rotate; 
    }
    robot.new_step_length = robot.initial_SL;
    robot.dS            = robot.velocity / robot.pub_rate;
    robot.incre_duty    = robot.dS / robot.initial_SL;

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

    // 最佳化初始最佳姿態
    // swing 加上 td prediction
    Initialize(robot, leg_model, false, 0, file2); 
    Send(robot);
    for(int i = 0; i < 10; ++i) {
        Send(robot);
    }
    csv(robot, leg_model, file2);
    // 加上control讓我可以控制什時候下一步
    // Print_current_Info(robot);
    while(1){
    //     // std::cout << "robot position: " << robot.body_position[0] << std::endl;
        Step(robot, leg_model, file2);
    //     // std::cout << "Send " <<  std::endl;
        Send(robot);
    //     // std::cout << "Store " <<  std::endl;
        csv(robot, leg_model, file2);
    }
    file2.close();
    return 0;
}




