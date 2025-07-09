#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

#include "pointcloud_downsampling.h"

namespace pointcloud_downsampling
{
std::string SUBSCRIBED_TOPIC, PUBLISHED_TOPIC, PUB_CLOUD_FRAME, PUBLISHED_TOPIC_CUTOFF;
float LEAF_SIZE_X, LEAF_SIZE_Y, LEAF_SIZE_Z;
float LEAF_SIZE_X_CUTOFF, LEAF_SIZE_Y_CUTOFF, LEAF_SIZE_Z_CUTOFF;
bool DOWNSAMPLE_ALL_DATA;

PointcloudDownsampling::PointcloudDownsampling(const rclcpp::NodeOptions & options)
: Node("pointcloud_downsampling", options)
{
  RCLCPP_INFO(get_logger(), "Start PointcloudDownsampling!");

  getParams();
  rclcpp::QoS qos = rclcpp::SensorDataQoS().best_effort();
  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(PUBLISHED_TOPIC, qos);

  rclcpp::QoS qos_cutoff = rclcpp::SensorDataQoS().best_effort();
  pointcloud_pub_cutoff_ = create_publisher<sensor_msgs::msg::PointCloud2>(PUBLISHED_TOPIC_CUTOFF, qos_cutoff);
  
  rclcpp::QoS qos_profile(10);
  qos_profile.reliable();
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    SUBSCRIBED_TOPIC, qos_profile,
    std::bind(&PointcloudDownsampling::voxelFiltering, this, std::placeholders::_1));
}

PointcloudDownsampling::~PointcloudDownsampling() {}

void PointcloudDownsampling::voxelFiltering(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud_filtered;
    // Filter points within specified x, y, z cutoffs
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud_z;
  for (const auto & point : pcl_cloud) {
    if (
        point.x >= 0.0 && point.x <= 100.0 && point.y >= -3.5 && point.y <= 3.8 && point.z <= -0.9) {
      pcl_cloud_z.push_back(point);
    }
    }


  pcl::PointCloud<pcl::PointXYZI> pcl_cloud_z_filtered;

  // Voxel filtering for cut off version
  pcl::VoxelGrid<pcl::PointXYZI> sor_cutoff;
  sor_cutoff.setInputCloud(pcl_cloud_z.makeShared());
  sor_cutoff.setLeafSize(LEAF_SIZE_X_CUTOFF, LEAF_SIZE_Y_CUTOFF, LEAF_SIZE_Z_CUTOFF);
  sor_cutoff.setDownsampleAllData(DOWNSAMPLE_ALL_DATA);
  sor_cutoff.filter(pcl_cloud_z_filtered);

  sensor_msgs::msg::PointCloud2 ros2_cloud_filtered_cutoff;
  pcl::toROSMsg(pcl_cloud_z_filtered, ros2_cloud_filtered_cutoff);
  RCLCPP_DEBUG(get_logger(), "before filter of cutoff cloud: %d", msg->width);
  RCLCPP_DEBUG(get_logger(), "after filter  of cutoff cloud: %d", ros2_cloud_filtered_cutoff.width);

  ros2_cloud_filtered_cutoff.header.frame_id = PUB_CLOUD_FRAME;
  ros2_cloud_filtered_cutoff.header.stamp = get_clock()->now();
  ros2_cloud_filtered_cutoff.is_dense = true;
//   pointcloud_pub_cutoff_->publish(std::move(ros2_cloud_filtered_cutoff));

  sensor_msgs::msg::PointCloud2 ros2_cloud_cutoff;
  pcl::toROSMsg(pcl_cloud_z, ros2_cloud_cutoff);
  ros2_cloud_cutoff.header.frame_id = PUB_CLOUD_FRAME;
  ros2_cloud_cutoff.header.stamp = get_clock()->now();
  ros2_cloud_cutoff.is_dense = true;
  pointcloud_pub_cutoff_->publish(std::move(ros2_cloud_cutoff));





  
  // Voxel filtering for normal version
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(pcl_cloud.makeShared());
  sor.setLeafSize(LEAF_SIZE_X, LEAF_SIZE_Y, LEAF_SIZE_Z);
  sor.setDownsampleAllData(DOWNSAMPLE_ALL_DATA);
  sor.filter(pcl_cloud_filtered);

  sensor_msgs::msg::PointCloud2 ros2_cloud_filtered;
  pcl::toROSMsg(pcl_cloud_filtered, ros2_cloud_filtered);
  RCLCPP_DEBUG(get_logger(), "before filter: %d", msg->width);
  RCLCPP_DEBUG(get_logger(), "after filter: %d", ros2_cloud_filtered.width);

  ros2_cloud_filtered.header.frame_id = PUB_CLOUD_FRAME;
  ros2_cloud_filtered.header.stamp = get_clock()->now();
  ros2_cloud_filtered.is_dense = true;

  pointcloud_pub_->publish(std::move(ros2_cloud_filtered));
}

void PointcloudDownsampling::getParams()
{
  declare_parameter<std::string>("sub_topic", "/cloud_registered_body");
  declare_parameter<std::string>("pub_topic", "/cloud_registered_body_downsampling");
  declare_parameter<std::string>("pub_topic_cutoff", "/cloud_registered_body_downsampling_cutoff");
  declare_parameter<std::string>("pub_cloud_frame", "livox_frame");
  declare_parameter<float>("leaf_size_x", 0.2);
  declare_parameter<float>("leaf_size_y", 0.2);
  declare_parameter<float>("leaf_size_z", 0.2);
  declare_parameter<float>("leaf_size_x_cutoff", 0.2);
  declare_parameter<float>("leaf_size_y_cutoff", 0.2);
  declare_parameter<float>("leaf_size_z_cutoff", 0.2);
  declare_parameter<bool>("downsample_all_data", false);

  get_parameter_or<std::string>("sub_topic", SUBSCRIBED_TOPIC, "/cloud_registered_body");
  get_parameter_or<std::string>("pub_topic", PUBLISHED_TOPIC, "/cloud_registered_body_downsampling");
  get_parameter_or<std::string>(
    "pub_topic_cutoff", PUBLISHED_TOPIC_CUTOFF, "/cloud_registered_body_downsampling_cutoff");
  get_parameter_or<std::string>("pub_cloud_frame", PUB_CLOUD_FRAME, "livox_frame");
  get_parameter_or<float>("leaf_size_x", LEAF_SIZE_X, 0.2);
  get_parameter_or<float>("leaf_size_y", LEAF_SIZE_Y, 0.2);
  get_parameter_or<float>("leaf_size_z", LEAF_SIZE_Z, 0.2);
  get_parameter_or<float>("leaf_size_x_cutoff", LEAF_SIZE_X_CUTOFF, 0.2);
  get_parameter_or<float>("leaf_size_y_cutoff", LEAF_SIZE_Y_CUTOFF, 0.2);
  get_parameter_or<float>("leaf_size_z_cutoff", LEAF_SIZE_Z_CUTOFF, 0.2);
  get_parameter_or<bool>("downsample_all_data", DOWNSAMPLE_ALL_DATA, false);

  RCLCPP_INFO(this->get_logger(), "subscribed_topic %s", SUBSCRIBED_TOPIC.c_str());
  RCLCPP_INFO(this->get_logger(), "published_topic %s", PUBLISHED_TOPIC.c_str());
  RCLCPP_INFO(this->get_logger(), "published_topic_cutoff %s", PUBLISHED_TOPIC_CUTOFF.c_str());
  RCLCPP_INFO(this->get_logger(), "leaf_size_x %f", LEAF_SIZE_X);
  RCLCPP_INFO(this->get_logger(), "leaf_size_y %f", LEAF_SIZE_Y);
  RCLCPP_INFO(this->get_logger(), "leaf_size_z %f", LEAF_SIZE_Z);
  RCLCPP_INFO(this->get_logger(), "downsample_all_data %s", DOWNSAMPLE_ALL_DATA ? "true" : "false");
}
}  // namespace pointcloud_downsampling

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_downsampling::PointcloudDownsampling)
