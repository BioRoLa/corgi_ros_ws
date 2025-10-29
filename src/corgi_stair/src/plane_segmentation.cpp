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
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>
#include <unordered_map>
#include <random>
#include <algorithm>

#include <Eigen/Dense>
#include "plane_segmentation.hpp"


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT_no_color;

PlaneSegmentation::PlaneSegmentation() :
    /* Initializer List */
    normals_(new pcl::PointCloud<pcl::Normal>)
{
    // Initialize
    pass_.setKeepOrganized(true);
    normal_estimator_.setNormalEstimationMethod(normal_estimator_.AVERAGE_3D_GRADIENT);
    // normal_estimator_.setNormalEstimationMethod(normal_estimator_.AVERAGE_DEPTH_CHANGE);
    // normal_estimator_.setNormalEstimationMethod(normal_estimator_.COVARIANCE_MATRIX);
    normal_estimator_.setMaxDepthChangeFactor(0.01f);
    normal_estimator_.setNormalSmoothingSize(10.0f);

    histogram_csv.open("histogram.csv");

}//end PlaneSegmentation


void PlaneSegmentation::init_tf() {
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    pub = nh.advertise<sensor_msgs::msg::PointCloud2>("plane_segmentation", 1);
    normal_pub = nh.advertise<visualization_msgs::msg::MarkerArray>("visualization_normals", 1);
    normal_pub2 = nh.advertise<sensor_msgs::msg::PointCloud2>("normal_points", 1);
    plane_pub = nh.advertise<visualization_msgs::msg::MarkerArray>("visualization_plane", 1);
}//end init_tf


PlaneDistances PlaneSegmentation::segment_planes(pcl::PointCloud<PointT>::Ptr cloud) {
    this->setInputCloud(cloud);
    this->computeNormals();
    this->group_by_normals();
    // std::vector<double> h_plane_distances = this->segment_by_distances(centroid_z, h_point_idx);
    // std::vector<double> v_plane_distances = this->segment_by_distances(-centroid_x, v_point_idx);   // align to +x direction in world frame
    auto [h_plane_distances, h_plane_point_indices] = segment_by_distances(centroid_z, h_point_idx);
    auto [v_plane_distances, v_plane_point_indices] = segment_by_distances(-centroid_x, v_point_idx);
    // std::vector<double> h_plane_distances2 = find_height_by_v_plane(v_plane_distances, v_plane_point_indices);

    // size_t n = std::max(h_plane_distances.size(), h_plane_distances2.size());
    // std::cout << std::fixed << std::setprecision(4);
    // std::cout << "Index | h_plane_distances | h_plane_distances2\n";
    // std::cout << "-----------------------------------------------\n";

    // for (size_t i = 0; i < n; ++i) {
    //     double v1 = (i < h_plane_distances.size()) ? h_plane_distances[i] : std::numeric_limits<double>::quiet_NaN();
    //     double v2 = (i < h_plane_distances2.size()) ? h_plane_distances2[i] : std::numeric_limits<double>::quiet_NaN();

    //     std::cout << std::setw(5) << i << " | "
    //               << std::setw(17) << v1 << " | "
    //               << std::setw(17) << v2 << "\n";
    // }

// v_plane_point_indices[i] 是第 i 個垂直平面的點 index 集合


    /* Visualize in rviz */
    // this->visualize_planes();
    // this->visualize_normal();
    // this->visualize_normal_in_space();
    // this->visualize_CubePlanes(h_plane_distances, v_plane_distances);

    
    // Eigen::Vector3d base_link_pos;
    // try {
    //     geometry_msgs::msg::TransformStamped transformStamped = 
    //         tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0), rclcpp::Duration(1.0));
        
    //     base_link_pos.x() = transformStamped.transform.translation.x;
    //     base_link_pos.y() = transformStamped.transform.translation.y;
    //     base_link_pos.z() = transformStamped.transform.translation.z;
    // } catch (tf2::TransformException &ex) {
    //     RCLCPP_WARN(rclcpp::get_logger("CorgiStair"), "TF error: %s", ex.what());
    // }
    // double v_d = base_link_pos.dot(centroid_z);
    // double h_d = base_link_pos.dot(centroid_x);
    // for (double& d : h_plane_distances) {
    //     // d -= v_d;
    // }
    // for (double& d : v_plane_distances) {
    //     // d = -d + h_d;
    // }
    
    // if (h_plane_distances.size() >= 2 && v_plane_distances.size() >= 1) {
    //     Eigen::Vector3d dir = centroid_z.cross(centroid_x); // 交線方向
    //     Eigen::Matrix2f A;
    //     A << centroid_z.x(), centroid_z.z(),
    //         centroid_x.x(), centroid_x.z();
    //     Eigen::Vector2d b(h_plane_distances[1], v_plane_distances[0]);
    //     Eigen::Vector2d xz = A.inverse() * b;
    //     Eigen::Vector3d p0(xz.x(), 0, xz.y()); // 交線在ground上的投影
    //     double dist = std::abs(centroid_z.dot(p0) - h_plane_distances[0]) / centroid_z.norm();    // 交線與ground距離
    //     std::cout << "stair height: " << dist << std::endl;
    // }


    return {h_plane_distances, v_plane_distances, centroid_z, -centroid_x};
}//end segment_planes


void PlaneSegmentation::setInputCloud(pcl::PointCloud<PointT>::Ptr cloud) {
    /* Apply ROI filtering */
    pass_.setInputCloud(cloud);
    pass_.setFilterFieldName("x");
    pass_.setFilterLimits(0.10, 1.0);
    pass_.filter(*cloud);
    pass_.setFilterFieldName("y");
    pass_.setFilterLimits(-0.4, 0.4);
    pass_.filter(*cloud);
    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(-0.5, 1.0);
    pass_.filter(*cloud);
    /* Transform from camera coord to world coord */
    pcl_ros::transformPointCloud("map", *cloud, *cloud, tf_buffer_);
    /* Set cloud */
    cloud_ = cloud;
    normal_estimator_.setInputCloud(cloud_);
}//end setInputCloud


void PlaneSegmentation::computeNormals() {
    normal_estimator_.compute(*normals_);
    for (auto& normal : normals_->points) {
        double nx = normal.normal_x;
        double ny = normal.normal_y;
        double nz = normal.normal_z;

        double abs_nx = std::abs(nx);
        double abs_ny = std::abs(ny);
        double abs_nz = std::abs(nz);

        if (abs_nz >= abs_nx) { // z為主方向
            if (nz < 0) {   // 翻轉為 +z
                normal.normal_x *= -1;
                normal.normal_y *= -1;
                normal.normal_z *= -1;
            }//end if
        } else {    // x為主方向
            if (nx > 0) {   // 翻轉為 -x
                normal.normal_x *= -1;
                normal.normal_y *= -1;
                normal.normal_z *= -1;
            }//end if
        }//end if else
    }//end for
}//end computeNormals


void PlaneSegmentation::group_by_normals() {
    centroid_z = Eigen::Vector3d(0, 0, 1);  // initial normal vector for horizontal plane
    centroid_x = Eigen::Vector3d(-1, 0, 0); // initial normal vector for vertical   plane
    h_point_idx.clear();
    v_point_idx.clear();

    for (size_t i = 0; i < normals_->size(); i++) {
        Eigen::Vector3d normal(normals_->points[i].normal_x,
                            normals_->points[i].normal_y,
                            normals_->points[i].normal_z);
        double cos_z = normal.dot(centroid_z);
        double cos_x = normal.dot(centroid_x);

        if (cos_z > cos_threshold_) {   // belongs to horizontal planes
            h_point_idx.push_back(i);
        } else if (cos_x > cos_threshold_) {    // belongs to vertical planes
            v_point_idx.push_back(i);
        }//end if else
    }//end for

    // Update centroid
    centroid_z = h_point_idx.size()>0? computeCentroid(h_point_idx) : centroid_z;
    centroid_x = v_point_idx.size()>0? computeCentroid(v_point_idx) : centroid_x;
}//end group_by_normals


std::pair<std::vector<double>, std::vector<std::vector<int>>> PlaneSegmentation::segment_by_distances(Eigen::Vector3d centroid, const std::vector<int>& indices) {
    const double bin_width = 0.001; // 1mm
    const int one_bin_point_threshold = 100;    // 100 points
    const int total_point_threshold   = 1000;    // 5000 points
    const double merge_threshold = 0.03;    // 5cm

    /* Calculate distances */
    std::vector<double> distances;
    std::vector<int> valid_indices;
    for (int i : indices) {
        const pcl::Normal& n = normals_->points[i];
        if (!std::isnan(n.normal_x) && !std::isnan(n.normal_y) && !std::isnan(n.normal_z)) {
            Eigen::Vector3d position(cloud_->points[i].x,
                                    cloud_->points[i].y,
                                    cloud_->points[i].z);
            double d = centroid.dot(position);  // projective distance
            distances.push_back(d);
            valid_indices.push_back(i);
        }//end if
    }//end for
    if (distances.empty()) return {};

    /* Create histogram by distances */
    // Calculate range of histogram
    auto [min_it, max_it] = std::minmax_element(distances.begin(), distances.end());
    double min_val = *min_it;
    double max_val = *max_it;
    int bin_count = static_cast<int>((max_val - min_val) / bin_width) + 1;
    std::vector<int> histogram(bin_count, 0);
    std::vector<double> bin_values(bin_count, 0);
    std::vector<std::vector<int>> bin_point_indices(bin_count);
    // Histogram 
    for (size_t i = 0; i < distances.size(); ++i) {
        double d = distances[i];
        int bin = static_cast<int>((d - min_val) / bin_width);
        histogram[bin]++;
        bin_values[bin] += d;
        bin_point_indices[bin].push_back(valid_indices[i]);
    }//end for
    
    /* 找出連續高密度 bin */
    bool in_range = false;
    std::vector<double> total_counts;
    std::vector<double> mean_distances;
    std::vector<std::vector<int>> candidate_plane_indices;
    double sum_count;
    double sum_value;
    int max_bin_index;
    int max_bin_value;
    std::vector<int> accumulated_indices;

    for (int i = 0; i < bin_count; ++i) {   // from smallest distance
        if (histogram[i] >= one_bin_point_threshold) {
            if (!in_range) {
                in_range = true;
                sum_count = histogram[i];
                sum_value = bin_values[i];
                max_bin_index = i;
                max_bin_value = histogram[i];
                accumulated_indices = bin_point_indices[i];
            } else {
                sum_count += histogram[i];
                sum_value += bin_values[i];
                if (histogram[i] > max_bin_value) {
                    max_bin_index = i;
                    max_bin_value = histogram[i];
                }
                accumulated_indices.insert(accumulated_indices.end(), bin_point_indices[i].begin(), bin_point_indices[i].end());
            }//end if else
        } else if (in_range) {
            if (sum_count >= total_point_threshold) {
                total_counts.push_back(sum_count);
                mean_distances.push_back(sum_value / sum_count);
                // mean_distances.push_back(bin_values[max_bin_index] / histogram[max_bin_index]);
                candidate_plane_indices.push_back(accumulated_indices);
            }//end if
            in_range = false;
        }//end if else
    }//end for
    if (in_range) {
        if (sum_count >= total_point_threshold) {
            total_counts.push_back(sum_count);
            mean_distances.push_back(sum_value / sum_count);
            // mean_distances.push_back(bin_values[max_bin_index] / histogram[max_bin_index]);
            candidate_plane_indices.push_back(accumulated_indices);
        }//end if
        in_range = false;
    }//end if

    /* Write to histogram csv */
    for (int i = 0; i < histogram.size(); ++i) {
        histogram_csv << histogram[i] << ",";
    }
    histogram_csv << "\n";
    for (int i = 0; i < histogram.size(); ++i) {
        histogram_csv << min_val+bin_width*i << ",";
    }
    histogram_csv << "\n";


    // std::vector<Eigen::Vector3d> points;
    // Eigen::Vector3d p_centroid(0, 0, 0);
    // if (mean_distances.size() >= 1 ) {
    //     for (int i : indices) {
    //         const pcl::Normal& n = normals_->points[i];
    //         if (!std::isnan(n.normal_x) && !std::isnan(n.normal_y) && !std::isnan(n.normal_z)) {
    //             Eigen::Vector3d position(cloud_->points[i].x,
    //                                     cloud_->points[i].y,
    //                                     cloud_->points[i].z);
    //             double d = centroid.dot(position);  // projective distance
    //             if (d > mean_distances[0]-bin_width && d < mean_distances[0]+bin_width) {
    //                 p_centroid += position;
    //                 points.push_back(position);
    //             }
    //         }//end if
    //     }//end for
    // }
    // if (points.size() >= 1) {
    //     p_centroid /= static_cast<double>(points.size());
    //     // std::cout << "p_centroid: " << p_centroid << std::endl;
    //     Eigen::MatrixXf A(points.size(), 3);
    //     for (size_t i = 0; i < points.size(); ++i) {
    //         A.row(i) = points[i] - p_centroid;
    //     }
    //     Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //     Eigen::Vector3d normal = svd.matrixV().col(2);  // 最小奇異值對應方向（即平面法向）
    //     std::cout << "before normal: " << centroid << std::endl;
    //     std::cout << "after  normal: " << normal << std::endl;
    //     std::cout << "before d: " << mean_distances[0] << std::endl;
    //     std::cout << "after  d: " << -normal.dot(p_centroid) << std::endl;
    // }

    
    /* Only keep max bin in a range (merge_threshold) */
    std::vector<bool> keep(mean_distances.size(), true);
    for (size_t i = 0; i < mean_distances.size(); i++) {
        for (size_t j = i + 1; j < mean_distances.size(); j++) {
            double dist = std::abs(mean_distances[i] - mean_distances[j]);
            if (dist < merge_threshold) {
                if (total_counts[i] >= total_counts[j]) {
                    keep[j] = false;
                } else {
                    keep[i] = false;
                }
            }
        }
    }
    std::vector<double> final_distances;
    std::vector<std::vector<int>> final_indices;
    for (size_t i = 0; i < mean_distances.size(); i++) {
        if (keep[i]) {
            final_distances.push_back(mean_distances[i]);
            final_indices.push_back(candidate_plane_indices[i]);
        }
    }

    return {final_distances, final_indices}; // from smalleset to largest
}//end segment_by_distances

std::vector<double> PlaneSegmentation::find_height_by_v_plane(const std::vector<double>& plane_distances, const std::vector<std::vector<int>>& plane_indices) {
    std::vector<double> avg_heights;

    for (int i=0; i<plane_indices.size(); i++) {
        double plane_distance = plane_distances[i];
        double lower = plane_distance - 0.03;
        double upper = plane_distance + 0.03;

        const std::vector<int>& indices = plane_indices[i];
        std::vector<double> highest_values(cloud_->width, -INFINITY);
        for (int idx : indices) {
            int col = idx % cloud_->width; // col index in organized point cloud
            const PointT& point = cloud_->points[idx];
            double distance = -centroid_x.dot(Eigen::Vector3d(point.x, point.y, point.z));
            if (distance >= lower && distance <= upper) {
                double height = centroid_z.dot(Eigen::Vector3d(point.x, point.y, point.z));
                if (height > highest_values[col]) {
                    highest_values[col] = height;
                }//end if
            }//end if
        }//end for

        // Compute average of valid highest_values
        double sum = 0.0;
        int count = 0;
        for (double val : highest_values) {
            if (val != -INFINITY) {
                sum += val;
                ++count;
            }
        }
        avg_heights.push_back((count > 0) ? (sum / count) : std::numeric_limits<double>::quiet_NaN());
    }//end for

    return avg_heights;
}//end find_height_by_v_plane

Eigen::Vector3d PlaneSegmentation::computeCentroid(const std::vector<int>& indices) {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int idx : indices) {
        const pcl::Normal& n = normals_->points[idx];
        centroid += Eigen::Vector3d(n.normal_x, n.normal_y, n.normal_z);
    }//end for
    if (!indices.empty()) {
        centroid.normalize();
    }//end if 
    return centroid;
}//end computeCentroid


void PlaneSegmentation::visualize_planes() {
    pcl::PointCloud<PointT>::Ptr colored_cloud(new pcl::PointCloud<PointT>(*cloud_));
    int idx = 0;
    for (size_t i = 0; i < cloud_->size(); ++i) {
            // 沒有有效 normal 的點，塗成黑色
            colored_cloud->points[i].r = 0;
            colored_cloud->points[i].g = 0;
            colored_cloud->points[i].b = 0;
    }
    for (int i : h_point_idx) {
        colored_cloud->points[i].r = 0;
        colored_cloud->points[i].g = 0;
        colored_cloud->points[i].b = 255;
    }
    for (int i : v_point_idx) {
        colored_cloud->points[i].r = 255;
        colored_cloud->points[i].g = 0;
        colored_cloud->points[i].b = 0;
    }

    /* Publish the result */
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header.frame_id = "map";
    pub.publish(output);

}//end visualize_planes


void PlaneSegmentation::visualize_normal() {
    // 可視化 Marker
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker_template;
    marker_template.header.frame_id = "map";
    marker_template.type = visualization_msgs::msg::Marker::ARROW;
    marker_template.action = visualization_msgs::msg::Marker::ADD;
    marker_template.scale.x = 0.005;
    marker_template.scale.y = 0.010;
    marker_template.scale.z = 0.010;
    marker_template.color.r = 0.0;
    marker_template.color.g = 1.0;
    marker_template.color.b = 0.0;
    marker_template.color.a = 1.0;
    marker_template.pose.orientation.x = 0.0;
    marker_template.pose.orientation.y = 0.0;
    marker_template.pose.orientation.z = 0.0;
    marker_template.pose.orientation.w = 1.0;  // 這是必要的！不能為 0
    // marker_template.lifetime = rclcpp::Duration(0.1);

    // 空間分格子平均
    float grid_size = 0.05;
    std::unordered_map<std::tuple<int, int, int>, pcl::PointNormal, boost::hash<std::tuple<int, int, int>>> grid_map;
    for (size_t i = 0; i < cloud_->size(); ++i) {
        const auto& pt = cloud_->points[i];
        const auto& nm = normals_->points[i];
        if (!pcl::isFinite(pt) || !pcl::isFinite(nm))
            continue;
        int gx = static_cast<int>(std::floor(pt.x / grid_size));
        int gy = static_cast<int>(std::floor(pt.y / grid_size));
        int gz = static_cast<int>(std::floor(pt.z / grid_size));
        auto key = std::make_tuple(gx, gy, gz);
        if (grid_map.find(key) == grid_map.end()) {
            pcl::PointNormal ptn;
            ptn.x = pt.x; ptn.y = pt.y; ptn.z = pt.z;
            ptn.normal_x = nm.normal_x; ptn.normal_y = nm.normal_y; ptn.normal_z = nm.normal_z;
            grid_map[key] = ptn;
        }
    }

    int id = 0;
    /* Point normal */
    for (const auto& kv : grid_map) {
        const auto& pt = kv.second;

        visualization_msgs::msg::Marker arrow = marker_template;
        arrow.id = id++;

        geometry_msgs::msg::Point start, end;
        start.x = pt.x;
        start.y = pt.y;
        start.z = pt.z;
        end.x = pt.x + 0.03 * pt.normal_x;
        end.y = pt.y + 0.03 * pt.normal_y;
        end.z = pt.z + 0.03 * pt.normal_z;
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        marker_array.markers.push_back(arrow);
    }

    /* Plane normal */
    marker_template.scale.x = 0.025;
    marker_template.scale.y = 0.050;
    marker_template.scale.z = 0.050;
    // 水平面法向量：centroid_z
    if (h_point_idx.size() > 0) {
        visualization_msgs::msg::Marker arrow = marker_template;
        arrow.id = id++;
        geometry_msgs::msg::Point start, end;
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0;
        end.x = 0.15 * centroid_z.x();
        end.y = 0.15 * centroid_z.y();
        end.z = 0.15 * centroid_z.z();
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        // 用藍色表示水平面 normal
        arrow.color.r = 0.0;
        arrow.color.g = 0.0;
        arrow.color.b = 1.0;

        marker_array.markers.push_back(arrow);
    }//end if
    // 垂直面法向量：centroid_x
    if (v_point_idx.size() > 0) {
        visualization_msgs::msg::Marker arrow = marker_template;
        arrow.id = id++;
        geometry_msgs::msg::Point start, end;
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0;
        end.x = 0.15 * centroid_x.x();
        end.y = 0.15 * centroid_x.y();
        end.z = 0.15 * centroid_x.z();
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        // 用紅色表示垂直面 normal
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;

        marker_array.markers.push_back(arrow);
    }//end if

    // 刪除多餘的舊 marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;
    for (int i = id; i < last_marker_count_; ++i) {
        delete_marker.id = i;
        marker_array.markers.push_back(delete_marker);
    }
    last_marker_count_ = id;  // 記住這次用了幾個 marker

    normal_pub.publish(marker_array);
}//end visualize_normal


void PlaneSegmentation::visualize_CubePlanes(const std::vector<double>& h_plane_distances, const std::vector<double>& v_plane_distances) {
    visualization_msgs::msg::MarkerArray marker_array;
    Eigen::Vector3d n_z = centroid_z.normalized();
    Eigen::Vector3d n_x = -centroid_x.normalized();
    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Quaterniond q_z = Eigen::Quaterniond::FromTwoVectors(z_axis, n_z);
    Eigen::Quaterniond q_x = Eigen::Quaterniond::FromTwoVectors(z_axis, n_x);
    // 顏色設定
    std_msgs::msg::ColorRGBA blue, red;
    blue.b = 1.0; blue.a = 0.3;
    red.r = 1.0; red.a = 0.3;


    int start_id = 0;
    for (size_t i = 0; i < h_plane_distances.size(); ++i) {
        double d = h_plane_distances[i];
        Eigen::Vector3d center = d * n_z;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Time::now();
        marker.id = start_id + i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        marker.pose.orientation.x = q_z.x();
        marker.pose.orientation.y = q_z.y();
        marker.pose.orientation.z = q_z.z();
        marker.pose.orientation.w = q_z.w();

        marker.scale.x = 1.0;  // width
        marker.scale.y = 1.0;  // height
        marker.scale.z = 0.001; // thickness

        marker.color = blue;

        marker_array.markers.push_back(marker);
    }
    for (int i = h_plane_distances.size(); i < 10; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Time::now();
        marker.id = start_id + i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }

    start_id = 100;
    for (size_t i = 0; i < v_plane_distances.size(); ++i) {
        double d = v_plane_distances[i];
        Eigen::Vector3d center = d * n_x;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Time::now();
        marker.id = start_id + i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        marker.pose.orientation.x = q_x.x();
        marker.pose.orientation.y = q_x.y();
        marker.pose.orientation.z = q_x.z();
        marker.pose.orientation.w = q_x.w();

        marker.scale.x = 1.0;  // width
        marker.scale.y = 1.0;  // height
        marker.scale.z = 0.001; // thickness

        marker.color = red;

        marker_array.markers.push_back(marker);
    }
    for (int i = v_plane_distances.size(); i < 10; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Time::now();
        marker.id = start_id + i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }

    plane_pub.publish(marker_array);
}

void PlaneSegmentation::visualize_normal_in_space() {
    pcl::PointCloud<PointT>::Ptr cloud_msg(new pcl::PointCloud<PointT>);
    cloud_msg->header.frame_id = "map";  // 或使用你自己的 frame
    cloud_msg->height = 1;
    cloud_msg->is_dense = false;

    std::set<int> h_idx_set(h_point_idx.begin(), h_point_idx.end());
    std::set<int> v_idx_set(v_point_idx.begin(), v_point_idx.end());

    for (size_t i = 0; i < normals_->size(); ++i) {
        const auto& n = normals_->points[i];
        if (!pcl::isFinite(n)) continue;

        PointT pt;
        pt.x = n.normal_x;
        pt.y = n.normal_y;
        pt.z = n.normal_z;

        if (h_idx_set.count(i)) {
            pt.r = 0; pt.g = 0; pt.b = 255;  // 藍
        } else if (v_idx_set.count(i)) {
            pt.r = 255; pt.g = 0; pt.b = 0;  // 紅
        } else {
            pt.r = 128; pt.g = 128; pt.b = 128;  // 灰
        }

        cloud_msg->points.push_back(pt);
    }

    cloud_msg->width = cloud_msg->points.size();
    pcl_conversions::toPCL(rclcpp::Time::now(), cloud_msg->header.stamp);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_msg, output);
    normal_pub2.publish(output);
}