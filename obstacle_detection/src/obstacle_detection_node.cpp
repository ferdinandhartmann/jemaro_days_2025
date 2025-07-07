#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
  explicit ObstacleDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("obstacle_detector", options)
  {
    declare_parameter<std::string>("input_topic", "/points_downsampled");
    declare_parameter<std::string>("obstacles_topic", "/obstacles");
    declare_parameter<std::string>("clusters_topic", "/obstacle_clusters");
    declare_parameter<double>("fov_deg", 60.0);
    declare_parameter<int>("max_cluster_publish", 5);
    declare_parameter<std::string>("frame_id", "livox_frame");

    get_parameter("input_topic", input_topic_);
    get_parameter("obstacles_topic", obstacles_topic_);
    get_parameter("clusters_topic", clusters_topic_);
    get_parameter("fov_deg", fov_deg_);
    get_parameter("max_cluster_publish", max_cluster_publish_);
    get_parameter("frame_id", frame_id_);

    // rclcpp::QoS qos(rclcpp::KeepLast(10));
    // qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    rclcpp::QoS qos = rclcpp::SensorDataQoS().best_effort();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        std::bind(&ObstacleDetector::cloudCallback, this, std::placeholders::_1));

    obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacles_topic_, 10);
    clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(clusters_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_boxes", 10);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::cout << "Received point cloud with timestamp: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::cout << "Converted PointCloud2 to PCL PointCloud with " << cloud.size() << " points." << std::endl;

    // RANSAC plane segmentation
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coeff);
    std::cout << "RANSAC segmentation found " << inliers->indices.size() << " inliers." << std::endl;

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZI>);
    extract.filter(*obstacles);
    std::cout << "Extracted obstacles cloud with " << obstacles->size() << " points." << std::endl;

    // Publish obstacle cloud with uniform color
    pcl::PointCloud<pcl::PointXYZRGB> obstacles_rgb;
    obstacles_rgb.width = obstacles->size();
    obstacles_rgb.height = 1;
    for (const auto & pt : obstacles->points) {
        pcl::PointXYZRGB rgb;
        rgb.x = pt.x; rgb.y = pt.y; rgb.z = pt.z;
        rgb.r = 255; rgb.g = 0; rgb.b = 0;
        obstacles_rgb.points.push_back(rgb);
    }
    sensor_msgs::msg::PointCloud2 obstacles_msg;
    pcl::toROSMsg(obstacles_rgb, obstacles_msg);
    obstacles_msg.header.frame_id = frame_id_;
    obstacles_msg.header.stamp = msg->header.stamp;
    obstacles_pub_->publish(obstacles_msg);
    std::cout << "Published obstacles cloud." << std::endl;

    // Clustering
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(obstacles);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(obstacles);
    ec.extract(cluster_indices);
    std::cout << "Found " << cluster_indices.size() << " clusters." << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB> clusters_rgb;
    clusters_rgb.height = 1;
    visualization_msgs::msg::MarkerArray markers;
    size_t id = 0;
    for (size_t c = 0; c < cluster_indices.size(); ++c) {
        const auto & indices = cluster_indices[c];
        uint8_t r = (c * 53) % 255;
        uint8_t g = (c * 97) % 255;
        uint8_t b = (c * 151) % 255;
        float min_pt[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        float max_pt[3] = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};
        for (int i : indices.indices) {
            const auto & pt = obstacles->points[i];
            pcl::PointXYZRGB rgb;
            rgb.x = pt.x; rgb.y = pt.y; rgb.z = pt.z;
            rgb.r = r; rgb.g = g; rgb.b = b;
            clusters_rgb.points.push_back(rgb);
            min_pt[0] = std::min(min_pt[0], pt.x);
            min_pt[1] = std::min(min_pt[1], pt.y);
            min_pt[2] = std::min(min_pt[2], pt.z);
            max_pt[0] = std::max(max_pt[0], pt.x);
            max_pt[1] = std::max(max_pt[1], pt.y);
            max_pt[2] = std::max(max_pt[2], pt.z);
        }
        std::cout << "Cluster " << c << " has " << indices.indices.size() << " points." << std::endl;
        if (c < static_cast<size_t>(max_cluster_publish_)) {
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
            m.color.a = 0.5;
            // Assign a unique color for each cluster
            m.color.r = static_cast<float>(r) / 255.0f;
            m.color.g = static_cast<float>(g) / 255.0f;
            m.color.b = static_cast<float>(b) / 255.0f;
            markers.markers.push_back(m);
        }
    }
    clusters_rgb.width = clusters_rgb.points.size();
    sensor_msgs::msg::PointCloud2 clusters_msg;
    pcl::toROSMsg(clusters_rgb, clusters_msg);
    clusters_msg.header.frame_id = frame_id_;
    clusters_msg.header.stamp = msg->header.stamp;
    clusters_pub_->publish(clusters_msg);
    marker_pub_->publish(markers);
    std::cout << "Published clusters and markers." << std::endl;
}
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string input_topic_;
  std::string obstacles_topic_;
  std::string clusters_topic_;
  std::string frame_id_;
  double fov_deg_;
  int max_cluster_publish_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}
