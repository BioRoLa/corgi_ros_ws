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
    // robot.legs[0].eta_current[0], robot.legs[0].eta_current[1]
    // robot.legs[1].eta_current[0], robot.legs[1].eta_current[1]
    // robot.legs[2].eta_current[0], robot.legs[2].eta_current[1]
    // robot.legs[3].eta_current[0], robot.legs[3].eta_current[1]
    // robot.legs[0].swing_phase, robot.legs[1].swing_phase, robot.legs[2].swing_phase,
    // robot.legs[0].foothold_current[0], robot.legs[0].foothold_current[1], robot.legs[0].foothold_current[2]
    // robot.legs[1].foothold_current[0], robot.legs[1].foothold_current[1], robot.legs[0].foothold_current[2]
    // robot.legs[2].foothold_current[0], robot.legs[2].foothold_current[1], robot.legs[0].foothold_current[2]
    // robot.legs[3].foothold_current[0], robot.legs[3].foothold_current[1], robot.legs[0].foothold_current[2]
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
                    double hip_x, double stand_h) 
{
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
        if (sampleSector(C, leg.F_l_c, leg.G_c, R)) return true;
    }
    // B: L_r_c → [G_c, F_r_c]
    {
        auto C = leg.L_r_c;
        double R = std::abs(leg.F_r_c - C) + leg.r ;
        if (sampleSector(C, leg.G_c, leg.F_r_c, R)) return true;
    }
    // C: U_r_c → [H_r_c, F_r_c]
    {
        auto C = leg.U_r_c;
        double R = std::abs(leg.H_r_c - C) + leg.r ;
        if (sampleSector(C, leg.H_r_c, leg.F_r_c, R)) return true;
    }
    // D: U_l_c → [H_l_c, F_l_c]
    {
        auto C = leg.U_l_c;
        double R = std::abs(leg.H_l_c - C) + leg.r ;
        if (sampleSector(C, leg.H_l_c, leg.F_l_c, R)) return true;
    }

    // G-sector: G_c → [extend toward L_l_c, extend toward L_r_c]
    {
        auto C = leg.G_c;
        // 往 L_*_c 方向的反向延伸 leg.r
        auto dirL = (leg.L_l_c - C) / std::abs(leg.L_l_c - C);
        auto dirR = (leg.L_r_c - C) / std::abs(leg.L_r_c - C);
        std::complex<double> P0 = C - dirL * leg.r;
        std::complex<double> P1 = C - dirR * leg.r;
        if (sampleSector(C, P0, P1, leg.r)) return true;
    }

    // E 段：U_l_c↔H_l_c、U_r_c↔H_r_c 延伸 leg.r 後取樣直線
    {
        // 延伸端點
        auto dirL = (leg.H_l_c - leg.U_l_c) / std::abs(leg.H_l_c - leg.U_l_c);
        auto dirR = (leg.H_r_c - leg.U_r_c) / std::abs(leg.H_r_c - leg.U_r_c);
        auto extL = leg.U_l_c + dirL * leg.r;
        auto extR = leg.U_r_c + dirR * leg.r;
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
static void generateSegmentedHermite(Robot &robot, LegModel &leg,
                                     double theta_start, double beta_start,
                                     double theta_end,   double beta_end,
                                     const Eigen::Vector2d &q_via,
                                     unsigned N, int rim, double alpha,
                                     int leg_index)
{
    for (unsigned i = 0; i <= N; ++i) {
        double t_norm = static_cast<double>(i) / N; // 0 → 1
        double theta, beta;
        if (t_norm <= 0.5) {
            theta = cubicHermite(0.0, theta_start, 0.0,
                                  0.5, q_via(0),   0.0, t_norm);
            beta  = cubicHermite(0.0, beta_start,  0.0,
                                  0.5, q_via(1),   0.0, t_norm);
        } else {
            theta = cubicHermite(0.5, q_via(0), 0.0,
                                  1.0, theta_end, 0.0, t_norm);
            beta  = cubicHermite(0.5, q_via(1), 0.0,
                                  1.0, beta_end,  0.0, t_norm);
        }
        robot.legs[leg_index].swing_traj.push_back({ t_norm, theta, beta });
    }
}
void planSwing(Robot &robot, LegModel &leg,
               double theta_start, double beta_start,
               double theta_end,   double beta_end,
               unsigned N, int rim, double alpha,
               int leg_index)
{
    robot.legs[leg_index].swing_traj.clear();
    robot.legs[leg_index].swing_traj.reserve(N + 1);
    Eigen::Vector2d q_via;
    q_via(1) = 0.5 * (beta_start + beta_end);   // β midpoint

    q_via(0) = 17.0 *M_PI / 180.0;
    generateSegmentedHermite(robot, leg, theta_start, beta_start,
                                theta_end,   beta_end,
                                q_via, N, rim, alpha,leg_index);
}
// ———— 模擬一次 swing + touchdown 回傳狀態碼 ————
TouchdownStatus simulate_status(Robot &robot, LegModel &leg_model, int leg_index, double ideal_height) 
{   
    // 1) swing phase
    // find the end pose
    double hip_x_temp = robot.legs[leg_index].hip_position_current[0] + robot.swing_time * robot.legs[leg_index].step_length_current;
    double terrain_height_temp = robot.terrain->height(hip_x_temp);
    bool found = false;
    double h_max = ideal_height-terrain_height_temp-robot.clearence;
    double h_min = ideal_height-terrain_height_temp-robot.clearence_M;
    for (double h = h_max; h >= h_min; h -= 0.0005) {
        bool theta_bound_reached = false;
        for (double sh = -robot.legs[leg_index].step_length_current; sh <= robot.legs[leg_index].step_length_current; sh += 0.01) {
            LegModel leg_opt(true);
            auto eta_temp = leg_model.inverse({0.0, -h + leg_model.r}, "G");
            // 後退 shift
            for (double s1 = 0; s1 < sh; s1 += 0.001) {
                eta_temp = leg_model.move(eta_temp[0], eta_temp[1], { -0.001, 0.0 }, 0.0);
                if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
                    return THETA_VIOLATION;
                }
            }
            // 前進半步長
            for (double s = 0; s < 0.5 * robot.legs[leg_index].step_length_current; s += 0.001) {
                try {
                    eta_temp = leg_model.move(eta_temp[0], eta_temp[1], {-0.001, 0.0}, 0.0);
                } catch (std::runtime_error &e) {
                    return UN_CONVERGED;
                }
            
                if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
                    return THETA_VIOLATION;
                }
            }
            // 初始碰撞檢查
            leg_model.forward(eta_temp[0], eta_temp[1], true);
            if (checkCollision(robot, leg_model,eta_temp[0],eta_temp[1],hip_x_temp,h)){
                return COLLISION;
            }
            // 初始接觸深度檢查
            // use leg.contact_map(eta[0], eta[1], 0.0); and leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, true)));
            // 先測試 hip 所在的地形
            leg_model.contact_map(eta_temp[0], eta_temp[1], atan(robot.terrain->slopeAt(hip_x_temp, false)));
            if ((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x_temp, false))) < robot.clearence){
                return PENETRATION;
            }
            else if((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x_temp, false))) > robot.clearence_M ){
                // 測試下一個地形
                leg_model.contact_map(eta_temp[0], eta_temp[1], atan(robot.terrain->slopeAt(hip_x_temp, true)));
                if(!robot.terrain->check_position(hip_x_temp + leg_model.contact_p[0])){
                    if ((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x_temp, true)))< robot.clearence ){
                        return PENETRATION;
                    }
                    else if((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x_temp, true))) > robot.clearence_M ){
                        return FURTHER;
                    }
                }
                else{
                    return FURTHER;
                }
            }
            // TDL check stability
            auto eta_temp_swing = eta_temp;
            double check;
            // touchdown phase
            for (double t = 0.0; t <= (1.0 - robot.swing_time); t += robot.incre_duty) {
                hip_x_temp += robot.dS;
                check = false;
                try{
                    // if touching the same slope
                    double contact_a;
                    double contact_b;
                    leg_model.contact_map(eta_temp[0], eta_temp[1], atan(robot.terrain->slopeAt(hip_x_temp, false)));
                    contact_a = -leg_model.contact_p[1] + robot.terrain->height(leg_model.contact_p[0]);
                    leg_model.contact_map(eta_temp[0], eta_temp[1], atan(robot.terrain->slopeAt(hip_x_temp, true)));
                    contact_b = -leg_model.contact_p[1] + robot.terrain->height(leg_model.contact_p[0]);
                    if (contact_b> contact_a) {
                        // current terrain as the hip position
                        check = true;
                    } else {
                        // next or touch the both terrain as the sametime 
                        check = false;
                    }
                    if( check ){
                        // 當前所在斜率
                        eta_temp = leg_model.move(eta_temp[0], eta_temp[1], {robot.dS, 0.0}, atan(robot.terrain->slopeAt(hip_x_temp, false)));
                    }
                    else{
                        // 下一個地形斜率
                        eta_temp = leg_model.move(eta_temp[0], eta_temp[1], {robot.dS, 0.0}, atan(robot.terrain->slopeAt(hip_x_temp, true)));
                    }
                }
                catch(const std::exception& e){
                    return UN_CONVERGED;
                }
            
                if (eta_temp[0] < 17*M_PI/180.0 || eta_temp[0] > 160*M_PI/180.0){
                    return THETA_VIOLATION;
                }
                
                leg_model.forward(eta_temp[0], eta_temp[1], true);
                if (checkCollision(robot, leg_model,eta_temp[0],eta_temp[1],hip_x_temp,h)){
                    return COLLISION;
                }
                
                if(check){
                    leg_model.contact_map(eta_temp[0], eta_temp[1], atan(robot.terrain->slopeAt(hip_x_temp, false)));
                    if ((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x_temp, false)))< robot.clearence ){
                        return PENETRATION;
                    }
                    else if((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0])) > robot.clearence_M ){
                        return FURTHER;
                    }
                }
                else{
                    leg_model.contact_map(eta_temp[0], eta_temp[1], atan(robot.terrain->slopeAt(hip_x_temp, true)));
                    if ((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x_temp, true)))< robot.clearence ){
                        return PENETRATION;
                    }
                    else if((leg_model.contact_p[1] - robot.terrain->height(hip_x_temp + leg_model.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x_temp, true))) > robot.clearence_M ){
                        return FURTHER;
                    }
                        
                }
            } 

            leg_model.forward(eta_temp_swing[0], eta_temp_swing[1], false);
            if (check){
                leg_model.contact_map(eta_temp_swing[0], eta_temp_swing[1], atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_current[0] + robot.swing_time * robot.legs[leg_index].step_length_current, false)));
            }
            else{
                leg_model.contact_map(eta_temp_swing[0], eta_temp_swing[1], atan(robot.terrain->slopeAt(robot.legs[leg_index].hip_position_current[0] + robot.swing_time * robot.legs[leg_index].step_length_current, true)));
            }
        
            int rim_id = leg_model.rim;
            double alpha0 = leg_model.alpha;
            // swing traj
            // TD: add stability and collision check in swing phase
            std::cout << "Generating swing trajectory..." << std::endl;
            // eta_temp_swing[1] -= 2.0*M_PI;
            while (robot.legs[leg_index].eta_current[1]<eta_temp_swing[1]){
                eta_temp_swing[1] -= 2.0*M_PI;
            }
            int N = robot.swing_time* robot.legs[leg_index].step_length_current / robot.dS;
        // generate_swing_trajectory(robot, leg_model, leg_index, 
        //                             h, 
        //                             robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], eta_temp_swing[0], eta_temp_swing[1],
        //                             N, rim_id, alpha0);   
            planSwing(robot, leg_model, robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], eta_temp_swing[0], eta_temp_swing[1],
              N, rim_id, alpha0,leg_index);

            return LEGAL;
        }
    }
}
double clamp(double value, double min_val, double max_val)
{
  return std::min(std::max(value, min_val), max_val);
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

        // // go optimization
        // Opt_Hybid_terrain(robot, leg_model, leg_index);
        robot.legs[leg_index].height_current = robot.legs[leg_index].height_next;
        robot.legs[leg_index].shift_current = robot.legs[leg_index].shift_next;
        // TDL swing with opt result
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
        
        planSwing(robot, leg_model, robot.legs[leg_index].eta_current[0], robot.legs[leg_index].eta_current[1], swing_end[0], swing_end[1],
              N, rim_id, alpha0,leg_index);
        
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
        if (robot.terrain->slopeAt(robot.legs[leg_index].hip_position_next[0],false)!= 0.0){
            robot.legs[leg_index].eta_next[0] = 17 * (M_PI / 180.0);
        }
        switch(leg_index){
            case 0:
                robot.legs[0].foothold_current = Eigen::Vector3d(robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                            robot.BW / 2,
                                                            robot.legs[0].relative_foothold_current.y());
                break;
            case 1:
                robot.legs[1].foothold_current = Eigen::Vector3d(robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                            -robot.BW / 2,
                                                            robot.legs[0].relative_foothold_current.y());
                break;
            case 2:
                robot.legs[2].foothold_current = Eigen::Vector3d(-robot.BL / 2 + robot.legs[0].relative_foothold_current.x(),
                                                            -robot.BW / 2,
                                                            robot.legs[0].relative_foothold_current.y());
                break;
            case 3:
                robot.legs[3].foothold_current = Eigen::Vector3d(-robot.BL / 2 - robot.legs[0].relative_foothold_current.x(),
                                                            robot.BW / 2,
                                                            robot.legs[0].relative_foothold_current.y());
                break;
        }
    } 
    // read the next Swing phase traj
    else { 
        Hybrid_Swing_Step(robot, leg_model, leg_index);
    }
    
    // update 
    robot.legs[leg_index].hip_position_current = robot.legs[leg_index].hip_position_next;
    
}
void Wheel_Step(Robot &robot, LegModel &leg_model, int leg_index){
    robot.legs[leg_index].hip_position_next[0] += robot.dS;
    robot.legs[leg_index].duty += robot.incre_duty;   
    robot.legs[leg_index].swing_phase = 0;
    /* Keep duty in the range [0, 1] */
    if (robot.legs[leg_index].duty < 0) robot.legs[leg_index].duty += 1.0; 

    /* Calculate next foothold if entering swing phase*/
    // Enter TD
     if (robot.legs[leg_index].duty > 1.0){     
         // Keep gaitSelector->duty in the range [0, 1]
        robot.legs[leg_index].duty -= 1.0;
    }

    /* Calculate next gaitSelector->eta */
    // Update all motor commands
    double beta_adjustment = (robot.velocity / 0.119) * (M_PI / 180.0);
    robot.legs[leg_index].eta_next[0] = 17 * (M_PI / 180.0);
    robot.legs[leg_index].eta_next[1] = robot.legs[leg_index].eta_current[1] - beta_adjustment;

    // update 
    robot.legs[leg_index].hip_position_current = robot.legs[leg_index].hip_position_next;
}
// Whole process
void Step(Robot &robot, LegModel &leg_model, std::ofstream &file2){
    
    // update the body position
    robot.body_position[0] += robot.dS;
    for (int i=0 ; i<4 ; ++i){
        robot.legs[i].leg_type_current = robot.legs[i].leg_type_next;
        switch(robot.legs[i].leg_type_current){
            // update
            robot.legs[i].hip_position_next = robot.body_position + Eigen::Vector3d(robot.BL/2, robot.BW/2, 0);
            case 0:
                // std::cout << "Leg: "<< i << " ,WHEELED mode continue!"<<std::endl;
                Wheel_Step(robot, leg_model, i);
                break;
            case 1:
                // std::cout << "Leg: "<< i << " ,HYBRID mode continue!"<<std::endl;
                Hybrid_Step(robot, leg_model, i);
                if(robot.legs[i].eta_next[0]<18*M_PI/180.0 && robot.legs[i].swing_phase == 0){
                    std::cout << "Leg: "<< i << " ,Wheeled mode continue!"<<std::endl;
                    robot.legs[i].leg_type_next = LegType::WHEELED;
                }
                break;
            case 2:
                // std::cout << "Leg: "<< i << " ,LEGGED mode continue!"<<std::endl;
                break;
            default:
                // std::cout << "Leg: "<< i << " ,TRANSFORM!"<<std::endl;
                break;
                
        }
    }
    csv(robot, leg_model, file2);
}
// with forced duty
TouchdownStatus simulate_td_status(Robot &robot,LegModel &leg,double stand_h, double shift, int leg_index, bool check_td) 
{ 
    double hip_x = robot.legs[leg_index].hip_position_current[0];
    // std::cout << "hip_x: " << hip_x << std::endl;
    // initial from the given duty
    auto eta = leg.inverse({0.0, -stand_h + leg.r}, "G");
    // 1) 後退 shift
    for (double s1 = 0; s1 < shift; s1 += 0.001) {
        eta = leg.move(eta[0], eta[1], { -0.001, 0.0 }, 0.0);
        if (eta[0] < 17*M_PI/180.0 || eta[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
    }
    // 2) 前進半步長
    for (double s = 0; s < 0.5 * robot.legs[leg_index].step_length_current; s += 0.001) {
        try {
                eta = leg.move(eta[0], eta[1], {-0.001, 0.0}, 0.0);
        } catch (std::runtime_error &e) {
            return UN_CONVERGED;
        }
        
        if (eta[0] < 17*M_PI/180.0 || eta[0] > 160*M_PI/180.0){
            return THETA_VIOLATION;
        }
    }
    
    // 3) 初始碰撞檢查
    leg.forward(eta[0], eta[1], true);
    if (checkCollision(robot, leg,eta[0],eta[1],hip_x,stand_h)){
        return COLLISION;
    }
    // 4) 初始接觸深度檢查
    // use leg.contact_map(eta[0], eta[1], 0.0); and leg.contact_map(eta[0], eta[1], atan(terrain.slopeAt(hip_x, true)));
    // 先測試 hip 所在的地形
    leg.contact_map(eta[0], eta[1], atan(robot.terrain->slopeAt(hip_x, false)));
    if ((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, false))) < robot.clearence){
        return PENETRATION;
    }
    
    else if((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, false))) > robot.clearence_M ){
        // 測試下一個地形
        leg.contact_map(eta[0], eta[1], atan(robot.terrain->slopeAt(hip_x, true)));
        if(!robot.terrain->check_position(hip_x + leg.contact_p[0])){
            if ((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, true)))< robot.clearence ){
                return PENETRATION;
            }
            else if((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, true))) > robot.clearence_M ){
                return FURTHER;
            }
        }
        else{
            return FURTHER;
        }
    }
    if (check_td) {
        for (double t = robot.legs[leg_index].duty; t <= (1.0 - robot.swing_time); t += robot.incre_duty) {
            hip_x += robot.dS;
            double check = false;
            try{
                // if touching the same slope as hip is
                double contact_a;
                double contact_b;
                leg.contact_map(eta[0], eta[1], atan(robot.terrain->slopeAt(hip_x, false)));
                contact_a = -leg.contact_p[1] + robot.terrain->height(leg.contact_p[0]);
                leg.contact_map(eta[0], eta[1], atan(robot.terrain->slopeAt(hip_x, true)));
                contact_b = -leg.contact_p[1] + robot.terrain->height(leg.contact_p[0]);
                if (contact_b> contact_a) {
                    // current terrain as the hip position
                    check = true;
                } else {
                    // next or touch the both terrain as the sametime 
                    check = false;
                }
                if( check ){
                    // 當前所在斜率
                    eta = leg.move(eta[0], eta[1], {robot.dS, 0.0}, atan(robot.terrain->slopeAt(hip_x, false)));
                }
                else{
                    // 下一個地形斜率
                    eta = leg.move(eta[0], eta[1], {robot.dS, 0.0}, atan(robot.terrain->slopeAt(hip_x, true)));
                }
            }
            catch(const std::exception& e){
                return UN_CONVERGED;
            }
        
            if (eta[0] < 17*M_PI/180.0 || eta[0] > 160*M_PI/180.0){
                return THETA_VIOLATION;
            }
            
            leg.forward(eta[0], eta[1], true);
            if (checkCollision(robot, leg,eta[0],eta[1],hip_x,stand_h)){
                return COLLISION;
            }
            
            if(check){
                leg.contact_map(eta[0], eta[1], atan(robot.terrain->slopeAt(hip_x, false)));
                if ((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, false)))< robot.clearence ){
                    return PENETRATION;
                }
                else if((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0])) > robot.clearence_M ){
                    return FURTHER;
                }
            }
            else{
                leg.contact_map(eta[0], eta[1], atan(robot.terrain->slopeAt(hip_x, true)));
                if ((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, true)))< robot.clearence ){
                    return PENETRATION;
                }
                else if((leg.contact_p[1] - robot.terrain->height(hip_x + leg.contact_p[0]) )*cos(atan(robot.terrain->slopeAt(hip_x, true))) > robot.clearence_M ){
                    return FURTHER;
                }
                    
            }
        }
    }
    return LEGAL;
}
void optimizePose(Robot &robot, LegModel &leg_model, bool swing_index) {
    // 1. 先optimize 出一步 sw+td ()
    // for each hip, find the pose for the current position
    for(int i = 0; i < 4; ++i) {
        // 先假設當前腿的高度和 shift 都是 ideal
        double target = robot.legs[i].height_current;
        double h_max =  robot.clearence_M - robot.terrain->height(robot.legs[i].hip_position_current[0]);
        double h_min =  - robot.terrain->height(robot.legs[i].hip_position_current[0]);
        std::cout << "Leg: " << i << " , target height: " << target << std::endl;
        std::cout << "hip terrain height: " << robot.terrain->height(robot.legs[i].hip_position_current[0]) << std::endl;
        std::cout << "Leg: " << i << " , h_max: " << h_max << ", h_min: " << h_min << std::endl;
        bool found = false;
        double best_h= robot.legs[i].height_current, best_shift=-robot.legs[i].shift_current;
        //  std::min(h_max,0.34) std::max(h_min, 0.119)
        for (double h = std::min(0.34,0.34); h >= std::max(0.12, 0.120); h -= 0.0005){
            // std::cout << "Testing height: " << h << std::endl;
            bool theta_bound_reached = false;
            for (double sh = -robot.legs[i].step_length_current; sh <= robot.legs[i].step_length_current; sh += 0.01) {
                // std::cout << "Testing shift: " << sh << std::endl;
                LegModel leg(true);
                TouchdownStatus status = simulate_td_status(robot, leg, h, sh, i, swing_index==i? false: true);
                if (status == LEGAL) {
                    std::cout << "Found LEGAL"<< std::endl;
                    best_h = h;
                    found = true;
                    best_shift = sh;
                    break;
                }
                else if (status == THETA_VIOLATION) {
                    // std::cout << "THETA_VIOLATION" << std::endl;
                    theta_bound_reached = true;
                    break; 
                }
                else if (status == COLLISION) {
                    // std::cout << "COLLISION" << std::endl;
                    continue; 
                }
                else if (status == PENETRATION) {
                    // std::cout << "PENETRATION" << std::endl;
                    continue; 
                }
                else if (status == UN_CONVERGED) {
                    // std::cout << "UN_CONVERGED" << std::endl;
                    break; 
                }
                else if (status == FURTHER){
                    // std::cout << "FURTHER" << std::endl;
                    continue; 
                }
            }
            if (found) {break;}
            if (theta_bound_reached) {continue;}
        }
        if (found) {
            std::cout << "Found pose: stand_h = " << best_h
                    << ", shift = " << best_shift << std::endl;
            robot.legs[i].height_current = best_h;
            robot.legs[i].shift_current = best_shift;
        } else {
            std::cout << "No legal pose found within bounds." << std::endl;
        }
    }
}

// Hybrid
void Initialize(Robot &robot, LegModel &leg_model, bool swing_index, int set_type,std::ofstream &file2) {
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
            // optimizePose(robot, leg_model, swing_index);
            for (int i = 0; i < 4; ++i) {
                robot.legs[i].eta_next = find_pose(robot, leg_model, robot.legs[i].height_current, robot.legs[i].shift_current, robot.legs[i].step_length_current, robot.legs[i].duty, 0.0);
                // assume initial on the plain
                leg_model.contact_map(robot.legs[i].eta_next[0], robot.legs[i].eta_next[1], 0.0);
                // deal with the foot point
                if(i==0 || i==3){
                    robot.legs[i].relative_foothold_current = Eigen::Vector2d(-leg_model.contact_p[0], leg_model.contact_p[1]);
                } else {
                    robot.legs[i].relative_foothold_current = Eigen::Vector2d( leg_model.contact_p[0], leg_model.contact_p[1]);
                }
                // std::cout << "relative foothold: " << robot.legs[i].relative_foothold_current[0] << "," << robot.legs[i].relative_foothold_current[1] << std::endl;
            }
            
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
    csv(robot, leg_model, file2);
}



int main(int argc, char** argv) {
    std::ofstream file2("opt_0612_2.csv");
    file2 << "theta0,beta0,theta1,beta1,theta2,beta2,theta3,beta3,swing_phase0,swing_phase1,swing_phase2,swing_phase3,foothold0_x,foothold0_y,foothold0_z,foothold1_x,foothold1_y,foothold1_z,foothold2_x,foothold2_y,foothold2_z,foothold3_x,foothold3_y,foothold3_z\n";
    // four leg
    // 0. setup
    bool sim = true;
    LegModel leg_model(sim);
    double initial_SL = 0.25; // 初始步長
    double ideal_height = 0.149; // 初始高度
    // A. Create terrain
    double terrain_start_height = 0;
    Terrain terrain_zigzag(terrain_start_height);
    // terrain_zigzag.addPlain(0.1);     
    // in tan() ___/---\---/----\_________ 
    terrain_zigzag.addPlain(5);  
    terrain_zigzag.addSlope(0.2593, 0.27); //0.07 0.120
    terrain_zigzag.addPlain(0.4); 
    terrain_zigzag.addSlope(-0.2593, 0.27);  
    terrain_zigzag.addPlain(0.4); 
    terrain_zigzag.addSlope(0.2593, 0.27); 
    terrain_zigzag.addPlain(0.4); 
    terrain_zigzag.addSlope(-0.2593, 0.27); 
    terrain_zigzag.addPlain(5);  
    // B. Create robot and legs
    Robot robot;
    // double ideal_height = -terrain_start_height;
    robot.terrain = &terrain_zigzag;
    robot.body_position = Eigen::Vector3d(0.0, 0.0, 0.0); 
    robot.CoM_pos       = Eigen::Vector2d(0.0, 0.0); // CoM 偏移位置
    robot.velocity      = 0.05;   // m/s
    robot.swing_time    = 0.2;   // 占空比
    // set hip position
    robot.legs[0].hip_position_current = Eigen::Vector3d(robot.body_position.x() + robot.BL / 2, robot.body_position.y() + robot.BW / 2, robot.body_position.z() );
    robot.legs[1].hip_position_current = Eigen::Vector3d(robot.body_position.x() + robot.BL / 2, robot.body_position.y() - robot.BW / 2, robot.body_position.z() );
    robot.legs[2].hip_position_current = Eigen::Vector3d(robot.body_position.x() - robot.BL / 2, robot.body_position.y() - robot.BW / 2, robot.body_position.z() );
    robot.legs[3].hip_position_current = Eigen::Vector3d(robot.body_position.x() - robot.BL / 2, robot.body_position.y() + robot.BW / 2, robot.body_position.z() );
    for (int i = 0; i < 4; ++i){
        robot.legs[i].height_current = ideal_height; 
        robot.legs[i].height_next = ideal_height; 
        robot.legs[i].shift_current = 0.05; // 初始偏移
        robot.legs[i].shift_next = 0.05; // 初始偏移        
        robot.legs[i].step_length_current = initial_SL; // 初始步長
        robot.legs[i].step_length_next = initial_SL; // 初始步長
        robot.legs[i].leg_type_current = LegType::HYBRID;
        robot.legs[i].leg_type_next = LegType::HYBRID;
;
        // in plain -> Reserse?
        robot.legs[i].swing_type = SwingType::Rotate; 
    }
    robot.new_step_length = initial_SL;
    robot.dS            = robot.velocity / robot.pub_rate;
    robot.incre_duty    = robot.dS / initial_SL; // duty cycle 增量
    // initial_SL = swing+td
    
    // C. check setup
    robot.terrain->printAll();
    // Print_current_Info(robot);

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

    // 最佳化應該有權重
    Initialize(robot, leg_model, false, 0, file2); 
    for(int i = 0; i < 1000; ++i) {
        Send(robot);
    }
    

    // // // 2. separate to fourleg
    // while(robot.body_position[0]<0.5){
    //     std::cout << "robot position: " << robot.body_position[0] << std::endl;
    //     Step(robot, leg_model, file2);
    //     Send(robot);
    // }
    file2.close();
    return 0;
}






