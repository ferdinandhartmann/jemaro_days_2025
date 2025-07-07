#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2_ros/static_transform_broadcaster.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <vector>
#include <limits>
#include <cmath>
#include <iostream> // For debug prints

class ObstacleDetector : public rclcpp::Node
{
public:
    explicit ObstacleDetector(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("obstacle_detector", options)
    {
        initializeParameters();

        rclcpp::QoS qos = rclcpp::SensorDataQoS().best_effort();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&ObstacleDetector::cloudCallback, this, std::placeholders::_1));

        obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacles_topic_, 10);
        clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(clusters_topic_, 10);
        high_intensity_obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("high_intensity_ransac", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_boxes", 10);

        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
        // publish_timer = create_wall_timer(100ms,    // rate
        //   [&](){callback_time();});
        publish_timer = create_wall_timer(
            std::chrono::milliseconds(100), // Publish every 100ms
            std::bind(&ObstacleDetector::add_obstacles_and_publish_map, this));

        initialize_map();
        add_obstacles_and_publish_map();
        // broadcast_map_frame();
    }

private:
    void initialize_map()
    {
        map_.info.resolution = 1.0; // 1 m per cell
        map_.info.width = 70;
        map_.info.height = 70;     
        map_.info.origin.position.x = 0.0;
        map_.info.origin.position.y = 0.0;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.w = 1.0;

        // Initialize all cells as free (0)
        map_.data.resize(map_.info.width * map_.info.height, 0);
    }

    void add_obstacles_and_publish_map()
    {
        // Add obstacles at known positions
        draw_rect(20, 30, 10, 5, 100);
        draw_rect(50, 40, 10, 5, 100);
        draw_rect(10, 10, 10, 5, 100);

        map_.header.stamp = this->now();
        map_.header.frame_id = "custom_map";
        publisher_->publish(map_);
        // RCLCPP_INFO(this->get_logger(), "Map with obstacles published");
    }

    void draw_rect(int x_start, int y_start, int width, int height, int value)
    {
        for (int y = y_start; y < y_start + height; y++)
        {
            for (int x = x_start; x < x_start + width; x++)
            {
                if (x >= 0 && x < static_cast<int>(map_.info.width) &&
                    y >= 0 && y < static_cast<int>(map_.info.height))
                {
                    int index = y * map_.info.width + x;
                    map_.data[index] = value; // Mark as occupied
                }
            }
        }
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points", cloud.size());

        auto obstacles = performRansacSegmentation(cloud);
        RCLCPP_INFO(this->get_logger(), "RANSAC removed %zu points, %zu points remain", cloud.size() - obstacles->size(), obstacles->size());

        publishObstacles(obstacles, msg->header);

        auto high_intensity_obstacles = filterHighIntensityObstacles(obstacles);
        RCLCPP_INFO(this->get_logger(), "Filtered high-intensity obstacles: %zu points", high_intensity_obstacles->size());

        publishHighIntensityObstacles(high_intensity_obstacles, msg->header);

        auto cluster_indices = performClustering(high_intensity_obstacles);
        RCLCPP_INFO(this->get_logger(), "Detected %zu clusters", cluster_indices.size());

        publishClustersAndMarkers(cluster_indices, high_intensity_obstacles, msg->header);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr performRansacSegmentation(const pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_distance_threshold_);

        // Other useful settings you can set:
        seg.setMaxIterations(80); // Maximum number of RANSAC iterations (default: 50)
        // seg.setProbability(0.99);   // Probability of finding a valid model (default: 0.99)
        seg.setEpsAngle(0.2);       // Angle epsilon for model fitting (for normals, in radians)
        seg.setAxis(Eigen::Vector3f(0, 0, 1)); // Preferred axis for model (e.g., z-axis for ground plane)
        // seg.setNormalDistanceWeight(0.1); // Weight for normal distance (if using normals)
        // seg.setInputNormals(normals); // Set input normals (if available and needed)

        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setInputCloud(cloud.makeShared());
        seg.segment(*inliers, *coeff);

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(true);

        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*obstacles);
        return obstacles;
    }

    void publishObstacles(const pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles, const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> obstacles_rgb;
        obstacles_rgb.width = obstacles->size();
        obstacles_rgb.height = 1;
        obstacles_rgb.is_dense = obstacles->is_dense;
        obstacles_rgb.points.reserve(obstacles->size());

        float min_intensity = std::numeric_limits<float>::max();
        float max_intensity = std::numeric_limits<float>::lowest();
        for (const auto &pt : obstacles->points)
        {
            if (pt.intensity < min_intensity)
                min_intensity = pt.intensity;
            if (pt.intensity > max_intensity)
                max_intensity = pt.intensity;
        }
        float intensity_range = max_intensity - min_intensity;
        if (intensity_range == 0.0f)
            intensity_range = 1.0f; // Prevent division by zero

        for (const auto &pt : obstacles->points)
        {
            pcl::PointXYZRGB rgb;
            rgb.x = pt.x;
            rgb.y = pt.y;
            rgb.z = pt.z;
            // Normalize intensity to [0, 1]
            float norm_intensity = (pt.intensity - min_intensity) / intensity_range;
            // Map intensity to color (e.g., blue->green->red)
            rgb.r = 255;
            rgb.g = static_cast<uint8_t>(255 * norm_intensity);
            rgb.b = static_cast<uint8_t>(255 * norm_intensity);
            obstacles_rgb.points.push_back(rgb);
        }
        sensor_msgs::msg::PointCloud2 obstacles_msg;
        pcl::toROSMsg(obstacles_rgb, obstacles_msg);
        obstacles_msg.header = header;
        obstacles_pub_->publish(obstacles_msg);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterHighIntensityObstacles(const pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles)
    {
        float min_intensity = std::numeric_limits<float>::max();
        float max_intensity = std::numeric_limits<float>::lowest();

        for (const auto &pt : obstacles->points)
        {
            if (pt.intensity < min_intensity)
                min_intensity = pt.intensity;
            if (pt.intensity > max_intensity)
                max_intensity = pt.intensity;
        }

        RCLCPP_DEBUG(this->get_logger(), "Intensity range: [%f, %f], Threshold: %f", min_intensity, max_intensity, intensity_threshold_);

        pcl::PointCloud<pcl::PointXYZI>::Ptr high_intensity_obstacles(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto &pt : obstacles->points)
        {
            if (pt.intensity > intensity_threshold_)
            {
                high_intensity_obstacles->points.push_back(pt);
            }
        }
        high_intensity_obstacles->width = high_intensity_obstacles->points.size();
        high_intensity_obstacles->height = 1;
        high_intensity_obstacles->is_dense = true;
        return high_intensity_obstacles;
    }

    void publishHighIntensityObstacles(const pcl::PointCloud<pcl::PointXYZI>::Ptr &high_intensity_obstacles, const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> high_intensity_obstacles_rgb;
        high_intensity_obstacles_rgb.width = high_intensity_obstacles->size();
        high_intensity_obstacles_rgb.height = 1;
        for (const auto &pt : high_intensity_obstacles->points)
        {
            pcl::PointXYZRGB rgb;
            rgb.x = pt.x;
            rgb.y = pt.y;
            rgb.z = pt.z;
            rgb.r = 0;
            rgb.g = 255;
            rgb.b = 0;
            high_intensity_obstacles_rgb.points.push_back(rgb);
        }

        sensor_msgs::msg::PointCloud2 high_intensity_obstacles_msg;
        pcl::toROSMsg(high_intensity_obstacles_rgb, high_intensity_obstacles_msg);
        high_intensity_obstacles_msg.header = header;
        high_intensity_obstacles_pub_->publish(high_intensity_obstacles_msg);
    }

    std::vector<pcl::PointIndices> performClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles)
    {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(obstacles);

        std::vector<pcl::PointIndices> all_cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(obstacles);
        ec.extract(all_cluster_indices);

        std::vector<pcl::PointIndices> cluster_indices;
        for (const auto &indices : all_cluster_indices)
        {
            // Calculate the bounding box dimensions of the cluster
            float min_pt[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            float max_pt[3] = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

            for (int idx : indices.indices)
            {
                const auto &pt = obstacles->points[idx];
                min_pt[0] = std::min(min_pt[0], pt.x);
                min_pt[1] = std::min(min_pt[1], pt.y);
                min_pt[2] = std::min(min_pt[2], pt.z);
                max_pt[0] = std::max(max_pt[0], pt.x);
                max_pt[1] = std::max(max_pt[1], pt.y);
                max_pt[2] = std::max(max_pt[2], pt.z);
            }

            float size_x = max_pt[0] - min_pt[0];
            float size_y = max_pt[1] - min_pt[1];
            float size_z = max_pt[2] - min_pt[2];

            // Check if the cluster size exceeds the maximum allowed size
            if (size_x <= max_cluster_size_meters_ && size_y <= max_cluster_size_meters_ && size_z <= max_cluster_size_meters_)
            {
                cluster_indices.push_back(indices);
            }
        }
        return cluster_indices;
    }

    void publishClustersAndMarkers(const std::vector<pcl::PointIndices> &cluster_indices, const pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles, const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> clusters_rgb;
        clusters_rgb.height = 1;
        visualization_msgs::msg::MarkerArray markers;
        size_t id = 0;

        std::vector<std::pair<size_t, double>> cluster_intensities;
        for (size_t c = 0; c < cluster_indices.size(); ++c)
        {
            const auto &indices = cluster_indices[c];
            double total_intensity = 0.0;
            for (int i : indices.indices)
            {
                total_intensity += obstacles->points[i].intensity;
            }
            double avg_intensity = total_intensity / indices.indices.size();
            cluster_intensities.emplace_back(c, avg_intensity);
        }

        // Sort clusters by average intensity in descending order
        std::sort(cluster_intensities.begin(), cluster_intensities.end(), [](const auto &a, const auto &b)
                  { return a.second > b.second; });

        // Process and publish the top x clusters
        int max_clusters = std::min(max_cluster_publish_, static_cast<int>(cluster_intensities.size()));
        size_t clusters_to_publish = std::min(cluster_intensities.size(), static_cast<size_t>(max_clusters));

        RCLCPP_INFO(this->get_logger(), "Publishing %zu clusters as markers", clusters_to_publish);

        for (size_t i = 0; i < clusters_to_publish; ++i)
        {
            size_t cluster_index = cluster_intensities[i].first;
            const auto &indices = cluster_indices[cluster_index];
            uint8_t r = (i * 53) % 255;
            uint8_t g = (i * 97) % 255;
            uint8_t b = (i * 151) % 255;
            float min_pt[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            float max_pt[3] = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

            for (int idx : indices.indices)
            {
                const auto &pt = obstacles->points[idx];
                pcl::PointXYZRGB rgb;
                rgb.x = pt.x;
                rgb.y = pt.y;
                rgb.z = pt.z;
                rgb.r = r;
                rgb.g = g;
                rgb.b = b;
                clusters_rgb.points.push_back(rgb);
                min_pt[0] = std::min(min_pt[0], pt.x);
                min_pt[1] = std::min(min_pt[1], pt.y);
                min_pt[2] = std::min(min_pt[2], pt.z);
                max_pt[0] = std::max(max_pt[0], pt.x);
                max_pt[1] = std::max(max_pt[1], pt.y);
                max_pt[2] = std::max(max_pt[2], pt.z);
            }

            // Publish marker for the cluster
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = this->get_clock()->now();
            m.ns = "obstacle_boxes";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = (min_pt[0] + max_pt[0]) / 2.0;
            m.pose.position.y = (min_pt[1] + max_pt[1]) / 2.0;
            m.pose.position.z = (min_pt[2] + max_pt[2]) / 2.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = (max_pt[0] - min_pt[0]);
            m.scale.y = (max_pt[1] - min_pt[1]);
            m.scale.z = (max_pt[2] - min_pt[2]);
            m.color.a = 0.85;
            m.color.r = static_cast<float>(r) / 255.0f;
            m.color.g = static_cast<float>(g) / 255.0f;
            m.color.b = static_cast<float>(b) / 255.0f;
            markers.markers.push_back(m);
        }

        clusters_rgb.width = clusters_rgb.points.size();
        sensor_msgs::msg::PointCloud2 clusters_msg;
        pcl::toROSMsg(clusters_rgb, clusters_msg);
        clusters_msg.header = header;
        clusters_pub_->publish(clusters_msg);
        marker_pub_->publish(markers);
    }

    void initializeParameters()
    {
        declare_parameter<std::string>("input_topic", "/points_downsampled");
        declare_parameter<std::string>("obstacles_topic", "/obstacles");
        declare_parameter<std::string>("clusters_topic", "/obstacle_clusters");
        declare_parameter<int>("max_cluster_publish", 5);
        declare_parameter<std::string>("frame_id", "livox_frame");
        declare_parameter<double>("cluster_tolerance", 0.5);
        declare_parameter<int>("min_cluster_size", 10);
        declare_parameter<int>("max_cluster_size", 25000);
        declare_parameter<int>("max_cluster_size_select", 25);
        declare_parameter<double>("ransac_distance_threshold", 0.2);
        declare_parameter<double>("intensity_threshold", 100.0);
        declare_parameter<double>("max_cluster_size_meters", 5.0); 

        get_parameter("input_topic", input_topic_);
        get_parameter("obstacles_topic", obstacles_topic_);
        get_parameter("clusters_topic", clusters_topic_);
        get_parameter("max_cluster_publish", max_cluster_publish_);
        get_parameter("frame_id", frame_id_);
        get_parameter("cluster_tolerance", cluster_tolerance_);
        get_parameter("min_cluster_size", min_cluster_size_);
        get_parameter("max_cluster_size", max_cluster_size_);
        get_parameter("max_cluster_size_select", max_cluster_size_select_);
        get_parameter("ransac_distance_threshold", ransac_distance_threshold_);
        get_parameter("intensity_threshold", intensity_threshold_);
        get_parameter("max_cluster_size_meters", max_cluster_size_meters_); 
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr high_intensity_obstacles_pub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer;

    nav_msgs::msg::OccupancyGrid map_;

    std::string input_topic_;
    std::string obstacles_topic_;
    std::string clusters_topic_;
    std::string frame_id_;
    double fov_deg_;
    int max_cluster_publish_;

    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double ransac_distance_threshold_;
    int max_cluster_size_select_;
    double intensity_threshold_;
    double max_cluster_size_meters_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}
