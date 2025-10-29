#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <unordered_map>
#include <cmath>
#include <limits>

typedef pcl::PointXYZRGB PointT;


struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

struct BinData {
    float r;
    PointT pt;
    bool valid = false;
};


/* Global variables */
ros::Publisher pub;
ros::Publisher normal_pub;
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener* tf_listener_;
std::vector<Eigen::Vector3f> cluster_centroids;
std::array<std::vector<Range>, 2> global_range;
corgi_msgs::msg::TriggerStamped trigger_msg;
int could_seq = 0;



void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input) {
    could_seq = input->header.seq;
    /* Step 1: Convert the ROS PointCloud2 message to PCL point cloud */
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);
    if (!cloud->isOrganized()) {
        RCLCPP_WARN(rclcpp::get_logger("CorgiStair"), "Point cloud is not organized. Skipping frame.");
        return;
    }

    
    /* Step 2: Apply ROI filtering */
    pcl::PassThrough<PointT> pass;
    pass.setKeepOrganized(true);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.20, 1.5);
    pass.filter(*cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.4, 0.4);
    pass.filter(*cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.5, 1.0);
    pass.filter(*cloud);

    /* Step 2.5: transform from camera coord to world coord */
    pcl_ros::transformPointCloud("map", *cloud, *cloud, tf_buffer_);


    /* Step 3: Downsample with VoxelGrid (1mm) */
    pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.001f, 0.001f, 0.001f);  // 1mm
    vg.filter(*cloud);

    /* Step 4: Keep only highest z point for each (x, y) grid */
    std::unordered_map<std::pair<int, int>, PointT, PairHash> xy_max_z_map;
    for (const auto& pt : cloud->points) {
        int x_mm = static_cast<int>(std::floor(pt.x * 1000));  // mm
        int y_mm = static_cast<int>(std::floor(pt.y * 1000));

        std::pair<int, int> key = {x_mm, y_mm};
        auto it = xy_max_z_map.find(key);

        if (it == xy_max_z_map.end() || pt.z > it->second.z) {
            xy_max_z_map[key] = pt;
        }
    }

    /* Step 5: Convert to (theta, z), keep nearest point per bin */
    std::unordered_map<std::pair<int, int>, PointT, PairHash> theta_z_map;
    for (const auto& kv : xy_max_z_map) {
        const PointT& pt = kv.second;

        float theta = std::atan2(pt.y, pt.x);  // radians
        int theta_deg = static_cast<int>(std::floor(theta * 1800.0 / M_PI));  // 0.1 deg resolution
        int z_mm = static_cast<int>(std::floor(pt.z * 1000));  // 1mm

        auto key = std::make_pair(theta_deg, z_mm);
        float dist_sq = pt.x * pt.x + pt.y * pt.y;  // ignore z

        auto it = theta_z_map.find(key);
        if (it == theta_z_map.end() || (pt.x * pt.x + pt.y * pt.y) < (it->second.x * it->second.x + it->second.y * it->second.y)) {
            theta_z_map[key] = pt;
        }
    }

    /* Step 5: Collect edge points */
    pcl::PointCloud<PointT>::Ptr stair_edge(new pcl::PointCloud<PointT>);
    for (const auto& kv : theta_z_map) {
        stair_edge->points.push_back(kv.second);
    }
    stair_edge->width = stair_edge->points.size();
    stair_edge->height = 1;
    stair_edge->is_dense = true;


    // Optional: publish or visualize edge_cloud
    RCLCPP_INFO(rclcpp::get_logger("CorgiStair"), "Extracted %zu edge points (stairs profile).", edge_cloud->points.size());
    // Example: publish or save
    // pcl::io::savePCDFileBinary("/tmp/edge_profile.pcd", *edge_cloud);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("plane_segmentation_node");
    auto cloud_sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloudCallback);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1, trigger_cb);
    pub = nh.advertise<sensor_msgs::msg::PointCloud2>("plane_segmentation", 1);
    normal_pub = nh.advertise<visualization_msgs::msg::MarkerArray>("visualization_normals", 1);
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    rclcpp::Rate rate(10);

    std::ofstream csv("plane_distances.csv");
    csv << "could_seq,";
    csv << "Trigger," ;
    csv << "Horizontal0,"; for (int i = 1; i < 10; ++i) csv << "Horizontal" << i << ",";
    csv << "Vertical0,";  for (int i = 1; i < 10; ++i) csv << "Vertical"   << i << ",";
    csv << "\n";

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        csv << could_seq << ",";
        csv << (int)trigger_msg.enable << ",";
        for (int i=0; i<10; i++) {
            if (i < global_range[0].size())
                csv << std::fixed << std::setprecision(4) << global_range[0][i].mean_distance;
            else
                csv << "0";
            csv << ",";
        }//end for
        for (int i=0; i<10; i++) {
            if (i < global_range[1].size())
                csv << std::fixed << std::setprecision(4) << global_range[1][i].mean_distance;
            else 
                csv << "0";
            csv << ",";
        }//end for
        csv << "\n";

        rate.sleep();
    }//end while

    csv.close();
    ros::shutdown();
    return 0;
}
