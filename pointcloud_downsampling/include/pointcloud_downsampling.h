#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud_downsampling
{
class PointcloudDownsampling : public rclcpp::Node
{
public:
  explicit PointcloudDownsampling(const rclcpp::NodeOptions & options);

  ~PointcloudDownsampling() override;

private:
  void getParams();

  void voxelFiltering(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_cutoff_;
};
}  // namespace pointcloud_downsampling
