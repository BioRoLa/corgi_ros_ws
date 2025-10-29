#include "corgi_odometry.hpp"

LegModel legmodel(SIM);

// Constants
constexpr double Z_POS_ANALYSIS_RATE = 1000.0;

// Variables
bool trigger = false;
corgi_msgs::msg::MotorStateStamped motor_state;
sensor_msgs::msg::Imu imu;
corgi_msgs::msg::ContactStateStamped contact_state;
std_msgs::msg::Float64 prev_z_COM;

std::vector<std::string> headers = {
    "estimate_z_position",
    "a.contact","b.contact","c.contact","d.contact",
    "a.score","b.score","c.score","d.score",
};
DataProcessor::CsvLogger logger;

std::string output_file_path;
std::string output_file_name = "";
Eigen::VectorXf state = Eigen::VectorXf::Zero(Z_POS_DATA_SIZE);

// Callbacks
void trigger_cb(const corgi_msgs::msg::TriggerStamped msg){
    trigger = msg.enable;

    if (RECORD_DATA){output_file_name = msg.output_filename;}

    if (trigger && output_file_name != "") {
        output_file_path = std::string(getenv("HOME")) + "/corgi_ws/corgi_ros_ws/output_data/" + output_file_name;

        int index = 1;
        std::string file_path_with_extension = output_file_path + "_pos_z.csv";
        while (logger.file_exists(file_path_with_extension)) {
            file_path_with_extension = output_file_path + "_" + std::to_string(index) + "_pos_z.csv";
            index++;
        }

        if (index != 1) {
            output_file_name += "_" + std::to_string(index-1) + "_pos_z.csv";
            output_file_path += "_" + std::to_string(index-1) + "_pos_z.csv";
        }
        else {
            output_file_name += "_pos_z.csv";
            output_file_path += "_pos_z.csv";
        }

        if (!logger.init) {

            // Initialize the CSV file.
            logger.initCSV(output_file_path, headers);

            RCLCPP_INFO(rclcpp::get_logger("CorgiOdometry"), "Saving data to %s\n", output_file_name.c_str());
        }
    }
    else {
        if(logger.init){
            logger.finalizeCSV();
            RCLCPP_INFO(rclcpp::get_logger("CorgiOdometry"), "Saved data to %s", output_file_name.c_str());
        }
    }
}

void motor_state_cb(const corgi_msgs::msg::MotorStateStamped state){
    motor_state = state;
}

void imu_cb(const sensor_msgs::msg::Imu msg){
    imu = msg;
}

void contact_cb(const corgi_msgs::msg::ContactStateStamped msg){
    contact_state = msg;
}

double estimate_z(double theta, double beta) {
    legmodel.forward(theta, beta);
    legmodel.contact_map(theta, beta);
    return -legmodel.contact_p[1];
}

// Function to compute Euler angles from a quaternion using ZYX order.
void quaternionToEuler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    // Normalize the quaternion (if not already normalized)
    Eigen::Quaterniond q_norm = q.normalized();

    // Calculate roll (x-axis rotation)
    roll = std::atan2(2.0 * (q_norm.w() * q_norm.x() + q_norm.y() * q_norm.z()),
                      1.0 - 2.0 * (q_norm.x() * q_norm.x() + q_norm.y() * q_norm.y()));

    // Calculate pitch (y-axis rotation)
    pitch = std::asin(2.0 * (q_norm.w() * q_norm.y() - q_norm.z() * q_norm.x()));

    // Calculate yaw (z-axis rotation)
    yaw = std::atan2(2.0 * (q_norm.w() * q_norm.z() + q_norm.x() * q_norm.y()),
                     1.0 - 2.0 * (q_norm.y() * q_norm.y() + q_norm.z() * q_norm.z()));
}

double median(std::vector<double> &v) {
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    if (n % 2 == 0) {
        return (v[n / 2 - 1] + v[n / 2]) / 2.0;
    } else {
        return v[n / 2];
    }
}

double low_pass_filter(double value, double prev_value, double cutoff_freq, double sample_rate) {
    double alpha = 1.0 / (1.0 + (cutoff_freq / sample_rate));
    return alpha * value + (1.0 - alpha) * prev_value;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("corgi_z_position");

    // ROS Publishers
    auto z_position_pub = nh.advertise<std_msgs::msg::Float64>("odometry/z_position_hip", 10);
    // ROS Subscribers
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", Z_POS_ANALYSIS_RATE, trigger_cb);
    auto motor_state_sub = nh.subscribe<corgi_msgs::msg::MotorStateStamped>("motor/state", Z_POS_ANALYSIS_RATE, motor_state_cb);
    auto imu_sub = nh.subscribe<sensor_msgs::msg::Imu>("imu", Z_POS_ANALYSIS_RATE, imu_cb);
    auto contact_sub = nh.subscribe<corgi_msgs::msg::ContactStateStamped>("odometry/contact", Z_POS_ANALYSIS_RATE, contact_cb);
    
    rclcpp::Rate rate(Z_POS_ANALYSIS_RATE);

    Eigen::Quaterniond q;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    prev_z_COM.data = 0.0;

    std::vector<corgi_msgs::msg::MotorState*> motor_state_modules = {
        &motor_state.module_a,
        &motor_state.module_b,
        &motor_state.module_c,
        &motor_state.module_d
    };

    std::vector<corgi_msgs::msg::ContactState*> contact_modules = {
        &contact_state.module_a,
        &contact_state.module_b,
        &contact_state.module_c,
        &contact_state.module_d
    };

    double z_leg[4];

    while (rclcpp::ok()){

        rclcpp::spin_some(node);

        q = {imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z};
        quaternionToEuler(q, roll, pitch, yaw);
        if(!SIM){
            pitch = -pitch;
            roll = -roll;
        }

        if(trigger){
            for (int i=0; i<4; i++){
    
                if (i == 1 || i == 2) {
                    z_leg[i] = estimate_z(motor_state_modules[i]->theta, motor_state_modules[i]->beta-pitch);
                }
                else {
                    z_leg[i] = estimate_z(motor_state_modules[i]->theta, motor_state_modules[i]->beta+pitch);
                }
            }

            std::vector<double> contact_heights;
            for (int i = 0; i < 4; i++) {
                if (contact_modules[i]->contact) {
                    contact_heights.push_back(z_leg[i]);
                }
            }

            std_msgs::msg::Float64 z_COM;
            if (contact_heights.empty()) {
                z_COM.data = prev_z_COM.data;
            } 
            else {
                switch(Z_POS_METHOD){
                    case AVG:{
                        double sum = std::accumulate(contact_heights.begin(), contact_heights.end(), 0.0);
                        z_COM.data = sum / contact_heights.size();
                        break;
                    }
                    case MID:
                        z_COM.data = median(contact_heights);
                        break;
                    case MAX:
                        z_COM.data = *std::max_element(contact_heights.begin(), contact_heights.end());
                        break;
                    case MIN:
                        z_COM.data = *std::min_element(contact_heights.begin(), contact_heights.end());
                        break;    
                    default:
                        RCLCPP_INFO(rclcpp::get_logger("CorgiOdometry"), "Invalid Z_POS_METHOD");
                }
            }
            z_COM.data = low_pass_filter(z_COM.data, prev_z_COM.data, 10, Z_POS_ANALYSIS_RATE);
            prev_z_COM.data = z_COM.data;

            z_position_pub.publish(z_COM);
            RCLCPP_INFO(rclcpp::get_logger("CorgiOdometry"), "z_COM: %f", z_COM);

            if (RECORD_DATA){
                state(0) = z_COM.data;
                for (int i = 0; i < 4; i++) {
                    state(1 + i) = contact_modules[i]->contact;
                }
                for (int i = 0; i < 4; i++) {
                    state(5 + i) = contact_modules[i]->score;
                }
                logger.logState(state);
            }

            rate.sleep();
        }
    }
    
    ros::shutdown();
    
    return 0;
}
