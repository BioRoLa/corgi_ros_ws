#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <cmath>

class StairDistance {
public:
  StairDistance() {
    // 訂閱 ZED 點雲
    sub_ = nh_.subscribe(
      "/zedxm/zed_node/point_cloud/cloud_registered",
      1, &StairDistance::cloudCb, this);

    // 發佈第一階踏板平面 inliers
    plane_pub_ = nh_.advertise<sensor_msgs::msg::PointCloud2>(
      "first_step_plane", 1);

    RCLCPP_INFO(rclcpp::get_logger("CorgiCamera"), "StairDistance node ready, will publish plane to /first_step_plane");
  }

private:
  rclcpp::Node nh_;
  ros::Subscriber sub_;
  ros::Publisher plane_pub_;

  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    // 1) 轉成 PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    // 2) 下採樣
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);
    vg.filter(*cloud);

    // 3) 去地面
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.05, 5.0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_ground(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*no_ground);
    if (no_ground->empty()) {
      RCLCPP_WARN(rclcpp::get_logger("CorgiCamera"), "No points after ground removal");
      return;
    }

    // 4) RANSAC 偵測第一個水平面
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(no_ground);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients coeff;
    seg.segment(*inliers, coeff);
    if (inliers->indices.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("CorgiCamera"), "No plane found");
      return;
    }

    // 5) 確認是水平面
    float a = coeff.values[0],
          b = coeff.values[1],
          c = coeff.values[2];
    float norm = std::sqrt(a*a + b*b + c*c);
    float nz = c / norm;
    if (std::fabs(nz) < 0.9f) {
      RCLCPP_WARN(rclcpp::get_logger("CorgiCamera"), "Plane not horizontal (nz=%.2f)", nz);
      return;
    }

    // 6) 擷取這個平面 inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(no_ground);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    // 7) 發佈 plane_cloud 到 first_step_plane
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*plane_cloud, out_msg);
    out_msg.header = msg->header;
    plane_pub_.publish(out_msg);

    // 8) 計算 plane inliers 質心，算出水平距離
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*plane_cloud, centroid);
    float cx = centroid[0], cy = centroid[1], cz = centroid[2];
    float horizontal_dist = std::sqrt(cx*cx + cy*cy);

    RCLCPP_INFO(rclcpp::get_logger("CorgiCamera"), "First step centroid (%.3f, %.3f, %.3f) → horizontal dist: %.3f m",
             cx, cy, cz, horizontal_dist);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("stair_distance");
  StairDistance node;
  rclcpp::spin(node);
  return 0;
}
