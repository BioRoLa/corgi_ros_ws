#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>
#include <unordered_map>
#include <random>

#include <corgi_msgs/msg/trigger_stamped.hpp>
#include <corgi_msgs/msg/stair_planes.hpp>
#include "plane_segmentation.hpp"
#include "plane_tracker.hpp"


// static tf2_ros::TransformBroadcaster tf_broadcaster;

PlaneSegmentation* plane_segmentation;
PlaneDistances plane_distances;
PlaneTracker plane_tracker;
corgi_msgs::msg::TriggerStamped trigger_msg;
ros::Publisher plane_pub;
corgi_msgs::msg::StairPlanes plane_msg;
int cloud_seq = 0;
Eigen::Vector3d h_normal;

void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    cloud_seq = msg->header.seq;
    /* Convert the ROS PointCloud2 message to PCL point cloud */
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    if (!cloud->isOrganized()) {
        RCLCPP_WARN(rclcpp::get_logger("CorgiStair"), "Point cloud is not organized. Skipping frame.");
        return;
    }

    plane_distances = plane_segmentation->segment_planes(cloud);

    /* Update & publish all planes */
    plane_tracker.update(plane_distances);
    plane_msg.horizontal = plane_tracker.get_horizontal_averages();
    plane_msg.vertical = plane_tracker.get_vertical_averages();
    Eigen::Vector3d v_normal = plane_tracker.get_vertical_normal();
    plane_msg.v_normal.x = v_normal.x();
    plane_msg.v_normal.y = v_normal.y();
    plane_msg.v_normal.z = v_normal.z();
    plane_pub.publish(plane_msg);

    // /* Correct Z axis */
    // // 這個四元數就是：把 h_normal 轉回 z 軸 的旋轉（姿態偏差）
    // h_normal = plane_distances.h_normal;
    // Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    // Eigen::Quaterniond q_corr = Eigen::Quaterniond::FromTwoVectors(h_normal, z_axis);

    // geometry_msgs::msg::TransformStamped tf_corr;
    // tf_corr.header.stamp = rclcpp::Time::now();
    // tf_corr.header.frame_id = "zedxm_left_camera_frame";
    // tf_corr.child_frame_id = "zedxm_camera_correct";

    // // 此旋轉會「轉正」camera 的姿態，使其對齊地面方向
    // tf_corr.transform.translation.x = 0.0;
    // tf_corr.transform.translation.y = 0.0;
    // tf_corr.transform.translation.z = 0.0;
    // tf_corr.transform.rotation.x = q_corr.x();
    // tf_corr.transform.rotation.y = q_corr.y();
    // tf_corr.transform.rotation.z = q_corr.z();
    // tf_corr.transform.rotation.w = q_corr.w();

    // tf_broadcaster.sendTransform(tf_corr);
}//end cloud_cb

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("plane_segmentation_node");
    auto cloud_sub   = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloud_cb);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1, trigger_cb);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::msg::TransformStamped camera_transform, last_camera_transform, camera_transform_tmp;
    camera_transform.transform.translation.x = 0.0;
    camera_transform.transform.translation.y = 0.0;
    camera_transform.transform.translation.z = 0.0;

    plane_segmentation = new PlaneSegmentation;
    plane_segmentation->init_tf();
    plane_pub = nh.advertise<corgi_msgs::msg::StairPlanes>("/stair_planes", 1);;
    rclcpp::Rate rate(10);

    std::ofstream plane_csv("plane_distances.csv");
    plane_csv << "Time,";
    plane_csv << "Trigger,";
    plane_csv << "Horizontal0,"; for (int i = 1; i < 10; ++i) plane_csv << "Horizontal" << i << ",";
    plane_csv << "Vertical0,";  for (int i = 1; i < 10; ++i) plane_csv << "Vertical"   << i << ",";
    plane_csv << "\n";
    std::ofstream stair_csv("stair_planes.csv");
    stair_csv << "Time,";
    stair_csv << "Trigger," ;
    stair_csv << "Camera_x,";
    stair_csv << "Camera_z,";
    stair_csv << "Horizontal0,"; for (int i = 1; i < 10; ++i) stair_csv << "Horizontal" << i << ",";
    stair_csv << "Vertical0,";  for (int i = 1; i < 10; ++i) stair_csv << "Vertical"   << i << ",";
    stair_csv << "\n";

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (tfBuffer.canTransform("map", "zedxm_camera_center", rclcpp::Time(0), rclcpp::Duration(0.0))) {
            try {
                camera_transform_tmp = tfBuffer.lookupTransform("map", "zedxm_camera_center", rclcpp::Time(0));
                // if (std::abs(camera_transform.transform.translation.x - camera_transform_tmp.transform.translation.x) < 0.1 
                //     && std::abs(camera_transform.transform.translation.y - camera_transform_tmp.transform.translation.y) < 0.1 
                //     && std::abs(camera_transform.transform.translation.z - camera_transform_tmp.transform.translation.z) < 0.1) {
                    last_camera_transform = camera_transform; // update camera pose
                    camera_transform = camera_transform_tmp; // update camera pose
                // }//end if
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1.0, "TF lookup failed even after canTransform: %s", ex.what());
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "TF not available at this moment");
        }

        /* plane_csv */
        plane_csv << rclcpp::Time::now() << ",";
        plane_csv << (int)trigger_msg.enable << ",";
        for (int i=0; i<10; i++) {
            if (i < plane_distances.horizontal.size())
                plane_csv << std::fixed << std::setprecision(4) << plane_distances.horizontal[i];
            else
                plane_csv << "0";
            plane_csv << ",";
        }//end for
        for (int i=0; i<10; i++) {
            if (i < plane_distances.vertical.size())
                plane_csv << std::fixed << std::setprecision(4) << plane_distances.vertical[i];
            else 
                plane_csv << "0";
            plane_csv << ",";
        }//end for
        plane_csv << "\n";

        /* stair_csv */
        stair_csv << rclcpp::Time::now() << ",";
        stair_csv << (int)trigger_msg.enable << ",";
        stair_csv << camera_transform.transform.translation.x << ",";
        stair_csv << camera_transform.transform.translation.z << ",";
        for (int i=0; i<10; i++) {
            if (i < plane_msg.horizontal.size())
                stair_csv << std::fixed << std::setprecision(4) << plane_msg.horizontal[i];
            else
                stair_csv << "0";
            stair_csv << ",";
        }//end for
        for (int i=0; i<10; i++) {
            if (i < plane_msg.vertical.size())
                stair_csv << std::fixed << std::setprecision(4) << plane_msg.vertical[i];
            else 
                stair_csv << "0";
            stair_csv << ",";
        }//end for
        stair_csv << "\n";

        rate.sleep();
    }//end while

    plane_csv.close();
    stair_csv.close();
    ros::shutdown();
    return 0;
}//end main
