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

#include <dbscan.hpp>
#include <corgi_msgs/msg/trigger_stamped.hpp>


/* Define */
#define INTERGRAL_IMAGE_NORMAL_ESTIMATION 1
#define CLUSTER_NORMAL 1
#define VISUALIZE_NORMAL 1 
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT_no_color;

/* Structure */
struct Color {
    uint8_t r, g, b;
};

struct NormalPoint {
    Eigen::Vector3f position;  // 空間座標
    Eigen::Vector3f normal;    // 單位法向量
    int clusterID = -1;
    double distance_proj = 0.0; // 投影距離
    int planeID = -1; // 平面ID
};

struct Range {
    double start;
    double end;
    int total_count;
    double mean_distance;
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
std::vector<int> global_histogram;

/* K-mean */
// 顏色視覺化輔助（你可以在顯示點雲時使用）
Color GetColor(int clusterID, int planeID) {
    int shade = planeID % 10; // 深淺 (最多支援100階)
    shade = std::max(0, 255 - shade * 50);

    if (clusterID == 0 && planeID != -1) {
        return Color{0, 0, shade}; // 藍色漸層
    } else if (clusterID == 1 && planeID != -1) {
        return Color{shade, 0, 0}; // 紅色漸層
    } else {
        return Color{128, 128, 128}; // default 灰色
    }
}

std::array<std::vector<Range>, 2> group_by_plane_distance(std::vector<NormalPoint>& points) {
    const double bin_width = 0.001; // 1mm
    const int one_bin_point_threshold = 100;    // 100 points
    const int total_point_threshold   = 1000;    // 5000 points
    const double merge_threshold = 0.05;    // 5cm
    const int num_clusters = cluster_centroids.size();

    std::array<std::vector<Range>, 2> return_range;
    // Calculate plane distance
    std::vector<std::vector<double>> distances(num_clusters);
    for (NormalPoint& p : points) {
        if (p.clusterID == -1) continue; // Skip unclassified points
        int clusterID = p.clusterID;
        double d = cluster_centroids[clusterID].dot(p.position);  // 投影距離
        p.distance_proj = d;
        distances[clusterID].push_back(d);
    }//end for

    // Calculate histogram for each cluster
    for (int c=0; c<num_clusters; c++) {
        if (distances[c].empty()) continue;

        // Calculate max & min distances
        auto [min_it, max_it] = std::minmax_element(distances[c].begin(), distances[c].end());
        double min_val = *min_it;
        double max_val = *max_it;

        int bin_count = static_cast<int>((max_val - min_val) / bin_width) + 1;
        std::vector<int> histogram(bin_count, 0);
        std::vector<std::vector<double>> bin_values(bin_count);

        // Calculate histogram
        for (double d : distances[c]) {
            int bin = static_cast<int>((d - min_val) / bin_width);
            histogram[bin]++;
            bin_values[bin].push_back(d);
        }
        
        if (c==1) {
            global_histogram = histogram;
        }
        

        // 找出連續高密度 bin
        bool in_range = false;
        Range current;
        std::vector<double> current_values;
        std::vector<Range> raw_ranges;
        for (int i = 0; i < bin_count; ++i) {
            if (histogram[i] >= one_bin_point_threshold) {
                if (!in_range) {
                    in_range = true;
                    current.start = min_val + i * bin_width;
                    current.total_count = histogram[i];
                    current_values = bin_values[i];
                } else {
                    current.total_count += histogram[i];
                    current_values.insert(current_values.end(), bin_values[i].begin(), bin_values[i].end());
                }
                current.end = min_val + (i + 1) * bin_width;
            } else if (in_range) {
                if (current.total_count >= total_point_threshold) {
                    double sum = std::accumulate(current_values.begin(), current_values.end(), 0.0f);
                    current.mean_distance = sum / current_values.size();
                    raw_ranges.push_back(current);
                }//end if
                in_range = false;
                current_values.clear();
            }//end if else
        }
        if (in_range) {
            double sum = std::accumulate(current_values.begin(), current_values.end(), 0.0f);
            current.mean_distance = sum / current_values.size();
            raw_ranges.push_back(current);
        }


        // Step 2: 過濾過近的範圍，只保留每對中點數較多的
        std::vector<bool> keep(raw_ranges.size(), true);
        for (size_t i = 0; i < raw_ranges.size(); ++i) {
            for (size_t j = i + 1; j < raw_ranges.size(); ++j) {
                double dist = std::abs(raw_ranges[i].mean_distance - raw_ranges[j].mean_distance);
                if (dist < merge_threshold) {
                    if (raw_ranges[i].total_count >= raw_ranges[j].total_count) {
                        keep[j] = false;
                    } else {
                        keep[i] = false;
                    }
                }
            }
        }
        std::vector<Range> final_ranges;
        for (size_t i = 0; i < raw_ranges.size(); ++i) {
            if (keep[i]) {
                final_ranges.push_back(raw_ranges[i]);
            }
        }


        // 根據平均距離排序，並分配 planeID（全局唯一）
        if (c==0) { // horizontal plane: small -> large
            std::sort(final_ranges.begin(), final_ranges.end(), [](const Range& a, const Range& b) {
                return a.mean_distance < b.mean_distance;
            });
        } else {    // vertical plane: large -> small
            std::sort(final_ranges.begin(), final_ranges.end(), [](const Range& a, const Range& b) {
                return a.mean_distance > b.mean_distance;
            });
        }

        std::map<float, int> distance_to_planeID;
        for (int i = 0; i < final_ranges.size(); ++i) {
            distance_to_planeID[final_ranges[i].mean_distance] = i;
        }

        // 分配 planeID 給每個點
        for (auto& p : points) {
            if (p.clusterID != c) continue;
            for (const auto& range : final_ranges) {
                if (p.distance_proj >= range.start && p.distance_proj <= range.end) {
                    p.planeID = distance_to_planeID[range.mean_distance];
                    break;
                }
            }
        }
        return_range[c] = final_ranges;

        // Print cluster info
        std::cout << "Cluster " << c << " histogram peaks:\n";
        for (const auto& range : final_ranges) {
            std::cout << "Plane approx in [" << range.start << ", " << range.end << "]\n";
            std::cout << " → plane_ID: "<< distance_to_planeID[range.mean_distance]
                      << ", count: " << range.total_count
                      << ", mean distance: " << range.mean_distance << "\n";
        }
        std::cout << std::endl;
    }//end for

    return return_range;
}//end group_by_plane_distance


// 計算 1 - cos(a, b)
double angleDistance(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return 1 - a.dot(b);
}


// 簡單版 KMeans
void kmeansNormals(std::vector<NormalPoint>& points, int k, int max_iter = 100) {
    const int N = points.size();
    if (N == 0) return;

    std::vector<Eigen::Vector3f> centroids;
    centroids.reserve(k);

    // 1. 初始化：隨機挑 k 個點當初始中心
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, N - 1);
    for (int i = 0; i < k; ++i)
    {
        centroids.push_back(points[dist(rng)].normal);
    }

    for (int iter = 0; iter < max_iter; ++iter)
    {
        bool changed = false;

        // 2. 分配每個點到最近中心
        for (auto& p : points)
        {
            float min_dist = std::numeric_limits<float>::max();
            int best_cluster = -1;
            for (int c = 0; c < k; ++c)
            {
                float dist_sq = angleDistance(p.normal, centroids[c]);
                if (dist_sq < min_dist)
                {
                    min_dist = dist_sq;
                    best_cluster = c;
                }
            }

            if (p.clusterID != best_cluster)
            {
                changed = true;
                p.clusterID = best_cluster;
            }
        }

        // 3. 重新計算每個中心
        std::vector<Eigen::Vector3f> new_centroids(k, Eigen::Vector3f::Zero());
        std::vector<int> counts(k, 0);
        for (const auto& p : points)
        {
            new_centroids[p.clusterID] += p.normal;
            counts[p.clusterID] += 1;
        }

        for (int c = 0; c < k; ++c)
        {
            if (counts[c] > 0)
            {
                new_centroids[c] /= static_cast<float>(counts[c]);
                new_centroids[c].normalize(); // Normal vector保持單位長度
            }
            else
            {
                // 這個 cluster 沒有人？隨便重新挑一個
                new_centroids[c] = points[dist(rng)].normal;
            }
        }

        centroids = new_centroids;

        if (!changed)
            break; // 收斂了
    }
    cluster_centroids = centroids;
}//end kmeansNormals


void group_by_normals(std::vector<NormalPoint>& points, int max_iter = 2) {
    // 1. 初始化：設定初始中心
    std::vector<Eigen::Vector3f> centroids;
    centroids.reserve(2);
    Eigen::Vector3f c1(0, 0, 1);
    Eigen::Vector3f c2(-1, 0, 0);
    centroids.push_back(c1);  // Horizontal plane
    centroids.push_back(c2); // Vertical plane

    double thresholds_angle[2] = {10.0, 10.0};  // 30度, 20度
    for (int iter = 0; iter < max_iter; iter++) {
        double threshold = std::cos(thresholds_angle[iter] * M_PI / 180.0);  
        bool changed = false;

        // 2. 分配每個點到最近中心
        for (auto& p : points) {
            double angle1 = p.normal.dot(centroids[0]);
            double angle2 = p.normal.dot(centroids[1]);
            int best_cluster = -1;
            if (angle1 > angle2) {
                if (angle1 > threshold) {
                    best_cluster = 0;
                }//end if
            } else {
                if (angle2 > threshold) {
                    best_cluster = 1;
                }//end if
            }//end if else

            if (p.clusterID != best_cluster) {
                changed = true;
                p.clusterID = best_cluster;
            }//end if
        }//end for

        // 3. 重新計算每個中心
        std::vector<Eigen::Vector3f> new_centroids(2, Eigen::Vector3f::Zero());
        std::vector<int> counts(2, 0);
        for (const auto& p : points) {
            if (p.clusterID != -1) {
                new_centroids[p.clusterID] += p.normal;
                counts[p.clusterID] += 1;
            }//end if
        }//end for

        for (int c = 0; c < 2; ++c) {
            if (counts[c] > 0) {
                new_centroids[c] /= static_cast<float>(counts[c]);
                new_centroids[c].normalize(); // Normal vector保持單位長度
            } else {
                // 這個 cluster 沒有人？重新設定為初始中心
                new_centroids[c] = c==0? c1 : c2;
            }//end if
        }//end for

        centroids = new_centroids;
        if (!changed)
            break; // 收斂了
    }//end for
    cluster_centroids = centroids;
}//end group_by_normals


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
    // pcl::VoxelGrid<PointT> voxel;
    // voxel.setInputCloud(cloud);
    // voxel.setLeafSize(0.01f, 0.01f, 0.01f);  // 1cm
    // voxel.filter(*cloud);

    // Set x,y,z range
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


    /* Step 3: Find normal vector for each point */
    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    #if INTERGRAL_IMAGE_NORMAL_ESTIMATION
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    // ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    // ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    #else // too slow
    pcl::PointCloud<PointT_no_color>::Ptr cloud_no_color(new pcl::PointCloud<PointT_no_color>);
    pcl::fromROSMsg(*input, *cloud_no_color);
    pcl::NormalEstimation<PointT_no_color, pcl::Normal> ne;
    pcl::search::KdTree<PointT_no_color>::Ptr tree(new pcl::search::KdTree<PointT_no_color>);
    ne.setInputCloud(cloud_no_color);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 設置搜索半徑
    ne.compute(*normals);
    #endif


    #if CLUSTER_NORMAL
    /* Step 4: Implementing DBSCAN for clustering */
    // pcl::PointCloud<PointT>::Ptr normal_clouds(new pcl::PointCloud<PointT>);
    // // vector<Point> normal_points;
    // for (size_t i = 0; i < cloud->size(); ++i) {
    //     if (!std::isnan(normals->points[i].normal_x) && !std::isnan(normals->points[i].normal_y) && !std::isnan(normals->points[i].normal_z)) {
    //         PointT point;
    //         point.x = normals->points[i].normal_x;
    //         point.y = normals->points[i].normal_y;
    //         point.z = normals->points[i].normal_z;
    //         // point.clusterID = UNCLASSIFIED;
    //         // normal_points.push_back(point);
    //         normal_clouds->points.push_back(point);
    //     } else {
    //         // 法線無效，可以選擇跳過該點或給予默認值
    //         // 這裡選擇跳過無效法線的點
    //         continue;
    //     }
    // }
    // 把 normal 抓出來
    std::vector<NormalPoint> normal_points;
    for (size_t i = 0; i < normals->size(); ++i)
    {
        const auto& n = normals->points[i];
        if (!std::isnan(n.normal_x) && !std::isnan(n.normal_y) && !std::isnan(n.normal_z))
        {
            Eigen::Vector3f normal_vec(n.normal_x, n.normal_y, n.normal_z);
            Eigen::Vector3f position_vec(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            normal_vec.normalize();
            normal_points.push_back({position_vec, normal_vec, -1, 0.0, -1});
        }
    }
    // DBSCAN ds(2, 0.1*0.1, normal_points); // minimum number of cluster, distance for clustering(metre^2), points
    // ds.run();
    // 執行 KMeans (分成3群)
    int k = 3;
    // kmeansNormals(normal_points, k);
    group_by_normals(normal_points);
    std::cout << "cluster_centroid0: \n" << cluster_centroids[0] << std::endl;
    std::cout << "cluster_centroid1: \n" << cluster_centroids[1] << std::endl;
    std::array<std::vector<Range>, 2> plane_ranges = group_by_plane_distance(normal_points);
    global_range[0] = plane_ranges[0];
    global_range[1] = plane_ranges[1];
    // pcl::VoxelGrid<PointT> vg;
    // vg.setInputCloud(normal_clouds);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);  // 設定 voxel 的大小
    // vg.filter(*normal_clouds);
    // std::cout << "Input cloud size: " << normals->size() << std::endl;
    // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(normal_clouds);
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(0.1);  // Set tolerance (e.g., 0.1 meters)
    // ec.setMinClusterSize(10);  // Minimum number of points to form a cluster
    // ec.setMaxClusterSize(10000);  // Maximum number of points in a cluster
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(normal_clouds);
    // std::vector<pcl::PointIndices> cluster_indices;
    // ec.extract(cluster_indices);

    
    // Step 5: Visualize the clusters (use different colors)
    pcl::PointCloud<PointT>::Ptr colored_cloud(new pcl::PointCloud<PointT>(*cloud));
    int idx = 0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!std::isnan(normals->points[i].normal_x) && !std::isnan(normals->points[i].normal_y) && !std::isnan(normals->points[i].normal_z)) {
            int cluster_id = normal_points[idx].clusterID;
            int plane_id   = normal_points[idx].planeID;
            // auto color = cluster_colors[cluster_id];
            Color color = GetColor(cluster_id, plane_id);
            colored_cloud->points[i].r = color.r;
            colored_cloud->points[i].g = color.g;
            colored_cloud->points[i].b = color.b;
            idx ++;
        } else {
            // 沒有有效 normal 的點，塗成黑色
            colored_cloud->points[i].r = 0;
            colored_cloud->points[i].g = 0;
            colored_cloud->points[i].b = 0;
        }
    }
    #else // OrganizedMultiPlaneSegmentation by all normals
    /* Step 4: Plane segmentation using normal vector */
    // Setting
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(10);
    mps.setAngularThreshold(0.017453 * 10.0); // 20 degrees in radians
    mps.setDistanceThreshold(0.02);          // 5cm
    mps.setInputCloud(cloud);
    mps.setInputNormals(normals);

    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    // mps.segment(regions); // 原始版本僅需 regions，無需 refine
    // mps.segmentAndRefine(regions); // refine
    mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);


    /* Step 5: Merge similar planes based on normal and distance */
    const float normal_threshold = std::cos(40.0 * M_PI / 180.0); // 10度以内允許合併
    const float distance_threshold = 0.10; // 平面到原點距離誤差5cm以内允許合併

    std::vector<bool> merged(model_coefficients.size(), false);
    std::vector<int> plane_labels(cloud->size(), -1); // 記錄每個點屬於哪個合併後的平面
    int plane_id = 0;
    for (size_t i = 0; i < model_coefficients.size(); ++i)
    {
        if (merged[i])
            continue;

        // 這是 base 平面
        const auto& coeff_base = model_coefficients[i];
        Eigen::Vector3f normal_base(coeff_base.values[0], coeff_base.values[1], coeff_base.values[2]);
        normal_base.normalize();
        float d_base = coeff_base.values[3] / normal_base.norm();

        // 對其他平面找可合併的
        merged[i] = true;
        for (size_t j = i + 1; j < model_coefficients.size(); ++j) {
            if (merged[j])
                continue;

            const auto& coeff_other = model_coefficients[j];
            Eigen::Vector3f normal_other(coeff_other.values[0], coeff_other.values[1], coeff_other.values[2]);
            normal_other.normalize();
            float d_other = coeff_other.values[3] / normal_other.norm();

            float dot_product = std::abs(normal_base.dot(normal_other));
            float distance_diff = std::abs(d_base - d_other);

            if (dot_product > normal_threshold && distance_diff < distance_threshold)
            {
                merged[j] = true;

                // 將 j 平面的 inlier 也歸類到 i 平面
                for (int idx : inlier_indices[j].indices)
                    plane_labels[idx] = plane_id;
            }
        }

        // 把自己也標上 plane_id
        for (int idx : inlier_indices[i].indices)
            plane_labels[idx] = plane_id;

        ++plane_id;
    }

    // 重新上色
    pcl::PointCloud<PointT>::Ptr colored_cloud(new pcl::PointCloud<PointT>(*cloud));
    for (size_t idx = 0; idx < plane_labels.size(); ++idx)
    {
        if (plane_labels[idx] >= 0 && colored_cloud->points[idx].z > -1.20)
        {
            uint8_t r = static_cast<uint8_t>((plane_labels[idx] * 53) % 255);
            uint8_t g = static_cast<uint8_t>((plane_labels[idx] * 97) % 255);
            uint8_t b = static_cast<uint8_t>((plane_labels[idx] * 223) % 255);

            colored_cloud->points[idx].r = r;
            colored_cloud->points[idx].g = g;
            colored_cloud->points[idx].b = b;
        }
    }
    #endif // CLUSTER_NORMAL

    /* Publish the result */
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header = input->header;
    output.header.frame_id = "map";
    pub.publish(output);


    #if VISUALIZE_NORMAL    // 可視化法線
    // 可視化 Marker
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker_template;
    marker_template.header.frame_id = "map";
    marker_template.type = visualization_msgs::msg::Marker::ARROW;
    marker_template.action = visualization_msgs::msg::Marker::ADD;
    marker_template.scale.x = 0.01;
    marker_template.scale.y = 0.002;
    marker_template.scale.z = 0.002;
    marker_template.color.r = 0.0;
    marker_template.color.g = 1.0;
    marker_template.color.b = 0.0;
    marker_template.color.a = 1.0;
    marker_template.pose.orientation.x = 0.0;
    marker_template.pose.orientation.y = 0.0;
    marker_template.pose.orientation.z = 0.0;
    marker_template.pose.orientation.w = 1.0;  // 這是必要的！不能為 0
    marker_template.lifetime = rclcpp::Duration(0.1);

    // 空間分格子平均
    float grid_size = 0.10f;
    std::unordered_map<std::tuple<int, int, int>, pcl::PointNormal, boost::hash<std::tuple<int, int, int>>> grid_map;
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];
        const auto& nm = normals->points[i];
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
    for (const auto& kv : grid_map) {
        const auto& pt = kv.second;

        visualization_msgs::msg::Marker arrow = marker_template;
        arrow.id = id++;

        geometry_msgs::msg::Point start, end;
        start.x = pt.x;
        start.y = pt.y;
        start.z = pt.z;
        end.x = pt.x + 0.05 * pt.normal_x;
        end.y = pt.y + 0.05 * pt.normal_y;
        end.z = pt.z + 0.05 * pt.normal_z;
        arrow.points.push_back(start);
        arrow.points.push_back(end);

        marker_array.markers.push_back(arrow);
    }
    normal_pub.publish(marker_array);
    #endif // VISUALIZE_NORMAL
}//end cloudCallback

void trigger_cb(const corgi_msgs::msg::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("plane_segmentation_node");
    auto cloud_sub = nh.subscribe("/zedxm/zed_node/point_cloud/cloud_registered", 1, cloudCallback);
    auto trigger_sub = nh.subscribe<corgi_msgs::msg::TriggerStamped>("trigger", 1, trigger_cb);
    pub = nh.advertise<sensor_msgs::msg::PointCloud2>("plane_segmentation", 1);
    normal_pub = nh.advertise<visualization_msgs::msg::MarkerArray>("visualization_normals", 1);
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    rclcpp::Rate rate(10);

    std::ofstream csv("plane_distances.csv");
    std::ofstream csv_histogram("histogram.csv");
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

        for (int i=0; i<global_histogram.size(); i++) {
            csv_histogram << global_histogram[i] << ",";
        }
        csv_histogram << "\n";


        rate.sleep();
    }//end while

    csv.close();
    ros::shutdown();
    return 0;
}//end main
