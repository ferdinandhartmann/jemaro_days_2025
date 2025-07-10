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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

class ObstacleDetector : public rclcpp::Node
{
public:
    explicit ObstacleDetector(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("obstacle_detector", options),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(*tf_buffer_)
    {
        initializeParameters();

        last_road_points_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        rclcpp::QoS qos = rclcpp::SensorDataQoS().best_effort();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&ObstacleDetector::cloudCallback, this, std::placeholders::_1));

        obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacles_topic_, 10);
        clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(clusters_topic_, 10);
        high_intensity_obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("high_intensity_ransac", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_boxes", 10);
        road_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("road_points", 10);

        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/jemaro_map", rclcpp::QoS(10).reliable());

        // publish_timer = create_wall_timer(
        //     std::chrono::milliseconds(100), // Publish every 100ms
        //     std::bind(&ObstacleDetector::add_obstacles_and_publish_map, this));

        hull_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("road_hull", 10);

        initialize_map();
        // broadcast_map_frame();
    }

private:
    // Add at top of class:
    struct Track
    {
        int id;
        Eigen::Vector2f centroid;
        int age;    // #frames seen
        int missed; // #frames missed in a row
    };

    using CloudI = pcl::PointCloud<pcl::PointXYZI>;
    using CloudIPtr = CloudI::Ptr;

    void initialize_map()
    {
        map_.info.resolution = resolution_; // 1 m per cell
        map_.info.width = width_;
        map_.info.height = height_;
        map_.info.origin.position.x = offset_x_;
        map_.info.origin.position.y = offset_y_;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.w = 1.0;

        // Initialize all cells as unknown (50)
        map_.data.resize(map_.info.width * map_.info.height, 50);
    }

    void add_obstacles_and_publish_map()
    {
        updateTracks(last_clusters_);

        static int call_count = 0;
        call_count++;

        // Only clear the map every 5 calls
        if (call_count % clear_map_interval_ == 0)
        {
            std::fill(map_.data.begin(), map_.data.end(), 50);
        }

        // // draw road points on the map as free space
        // for (const auto &pt_in : *last_road_points_)
        // {
        //     // transform to map frame
        //     geometry_msgs::msg::PointStamped pin, pout;
        //     pin.header.frame_id = frame_id_;
        //     pin.header.stamp = rclcpp::Time(0);
        //     pin.point.x = pt_in.x;
        //     pin.point.y = pt_in.y;
        //     pin.point.z = pt_in.z;

        //     try
        //     {
        //         pout = tf_buffer_->transform(pin, "map", tf2::durationFromSec(0.05));
        //     }
        //     catch (const tf2::TransformException &e)
        //     {
        //         continue; // skip those you can’t transform
        //     }

        //     int mx = static_cast<int>((pout.point.x - map_.info.origin.position.x) / map_.info.resolution);
        //     int my = static_cast<int>((pout.point.y - map_.info.origin.position.y) / map_.info.resolution);

        //     // Draw a square around the point
        //     for (int dx = -inflation_radius_freespace_; dx <= inflation_radius_freespace_; ++dx)
        //     {
        //         for (int dy = -inflation_radius_freespace_; dy <= inflation_radius_freespace_; ++dy)
        //         {
        //             int nx = mx + dx;
        //             int ny = my + dy;

        //             if (nx >= 0 && nx < static_cast<int>(map_.info.width) &&
        //                 ny >= 0 && ny < static_cast<int>(map_.info.height))
        //             {
        //                 int index = ny * map_.info.width + nx;
        //                 map_.data[index] = 0; // Free
        //             }
        //         }
        //     }
        // }

        // For each track, associate it with the closest cluster in last_clusters_ (if available)
        for (size_t track_idx = 0; track_idx < tracks_.size(); ++track_idx)
        {
            const auto &tr = tracks_[track_idx];
            // for every track that’s old enough:
            if (tr.age < min_age_frames_)
                continue;

            // Find the closest cluster to this track's centroid
            size_t best_cluster = 0;
            float best_dist = std::numeric_limits<float>::max();
            for (size_t c = 0; c < last_clusters_.size(); ++c)
            {
                Eigen::Vector2f cluster_centroid = computeCentroid(last_clusters_[c]);
                float dist = (tr.centroid - cluster_centroid).norm();
                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_cluster = c;
                }
            }

            // Use the best matching cluster for this track
            const auto &cluster = last_clusters_.empty() ? pcl::PointCloud<pcl::PointXYZI>() : last_clusters_[best_cluster];

            pcl::PointCloud<pcl::PointXYZI> transformed_cluster;
            for (const auto &point : cluster)
            {
                pcl::PointXYZI pt = point;
                // Transform point from cluster frame to map frame using tf2
                geometry_msgs::msg::PointStamped pt_in, pt_out; // Ensure PointStamped is recognized
                pt_in.header.frame_id = frame_id_;
                pt_in.header.stamp = rclcpp::Time(0);
                pt_in.point.x = pt.x;
                pt_in.point.y = pt.y;
                pt_in.point.z = pt.z;

                try
                {
                    pt_out = tf_buffer_->transform(pt_in, "map", tf2::durationFromSec(0.05));
                    pt.x = pt_out.point.x;
                    pt.y = pt_out.point.y;
                    pt.z = pt_out.point.z;
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                    // If transform fails, keep original point
                }
                transformed_cluster.push_back(pt);
            }


            // draw on map
            for (const auto &point : transformed_cluster)
            {
                // Convert point to map grid coordinates
                int map_x = static_cast<int>((point.x - map_.info.origin.position.x) / map_.info.resolution);
                int map_y = static_cast<int>((point.y - map_.info.origin.position.y) / map_.info.resolution);

                // Draw a square around the point
                for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx)
                {
                    for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy)
                    {
                        int nx = map_x + dx;
                        int ny = map_y + dy;

                        if (nx >= 0 && nx < static_cast<int>(map_.info.width) &&
                            ny >= 0 && ny < static_cast<int>(map_.info.height))
                        {
                            int index = ny * map_.info.width + nx;
                            map_.data[index] = 100; // Occupied
                        }
                    }
                }
            }
        }

        map_.header.stamp = this->now();
        map_.header.frame_id = "map";
        map_publisher_->publish(map_);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points", cloud.size());

        auto [obstacles, road_points] = performRansacSegmentation(cloud);
        last_road_points_ = road_points;
        RCLCPP_INFO(this->get_logger(), "RANSAC removed %zu points, %zu points remain", cloud.size() - obstacles->size(), obstacles->size());

        publishObstacles(obstacles, msg->header);

        // publish the road points in their own topic
        sensor_msgs::msg::PointCloud2 road_msg;
        pcl::toROSMsg(*road_points, road_msg);
        road_msg.header = msg->header;
        road_pub_->publish(road_msg);

        auto high_intensity_obstacles = filterHighIntensityObstacles(obstacles);
        RCLCPP_INFO(this->get_logger(), "Filtered high-intensity: %zu points", high_intensity_obstacles->size());

        publishHighIntensityObstacles(high_intensity_obstacles, msg->header);

        auto cluster_indices = performClustering(high_intensity_obstacles);
        RCLCPP_INFO(this->get_logger(), "Detected %zu clusters", cluster_indices.size());

        last_clusters_ = publishClustersAndMarkers(cluster_indices, high_intensity_obstacles, msg->header);

        add_obstacles_and_publish_map();
    }

    // New helper to compute centroid:
    Eigen::Vector2f computeCentroid(const pcl::PointCloud<pcl::PointXYZI> &cloud)
    {
        Eigen::Vector4f c;
        pcl::compute3DCentroid(cloud, c);
        return {c.x(), c.y()};
    }

    // After publishClustersAndMarkers (i.e. when clusters_out ready):
    void updateTracks(const std::vector<pcl::PointCloud<pcl::PointXYZI>> &clusters)
    {
        // RCLCPP_INFO(this->get_logger(), "updateTracks: Starting with %zu clusters and %zu existing tracks", clusters.size(), tracks_.size());

        std::vector<Eigen::Vector2f> cents;
        for (auto &cl : clusters)
        {
            cents.push_back(computeCentroid(cl));
        }

        // RCLCPP_INFO(this->get_logger(), "updateTracks: Computed %zu centroids", cents.size());

        // Flag arrays
        std::vector<bool> used_new(cents.size(), false);
        std::vector<bool> used_old(tracks_.size(), false);

        // 1) Match new centroids to existing tracks
        for (size_t i = 0; i < cents.size(); ++i)
        {
            float best_d = max_match_dist_;
            int best_t = -1;
            for (size_t t = 0; t < tracks_.size(); ++t)
            {
                if (used_old[t])
                    continue;
                float d = (tracks_[t].centroid - cents[i]).norm();
                if (d < best_d)
                {
                    best_d = d;
                    best_t = t;
                }
            }
            if (best_t >= 0)
            {
                // RCLCPP_INFO(this->get_logger(), "updateTracks: Matched centroid %zu to track %d with distance %f", i, best_t, best_d);
                tracks_[best_t].centroid = cents[i];
                tracks_[best_t].age++;
                tracks_[best_t].missed = 0;
                used_new[i] = used_old[best_t] = true;
            }
        }

        // 2) unmatched new → spawn
        for (size_t i = 0; i < cents.size(); ++i)
        {
            if (!used_new[i])
            {
                // RCLCPP_INFO(this->get_logger(), "updateTracks: Spawning new track for centroid %zu", i);
                tracks_.push_back({next_track_id_++, cents[i], 1, 0});
            }
        }

        used_old.resize(tracks_.size(), false);

        // 3) unmatched old → increment missed
        for (size_t t = 0; t < tracks_.size(); ++t)
        {
            if (!used_old[t])
            {
                // RCLCPP_INFO(this->get_logger(), "updateTracks: Incrementing missed count for track %d", tracks_[t].id);
                tracks_[t].missed++;
            }
        }

        // 4) prune:
        size_t before_prune = tracks_.size();
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [&](const Track &tr)
                           {
                               return tr.missed > max_missed_frames_;
                           }),
            tracks_.end());
        size_t after_prune = tracks_.size();

        RCLCPP_INFO(this->get_logger(), "updateTracks: Pruned %zu tracks, %zu remaining", before_prune - after_prune, after_prune);
    }

    std::pair<ObstacleDetector::CloudIPtr, ObstacleDetector::CloudIPtr>
    performRansacSegmentation(const CloudI &cloud)
    {
        // Set up RANSAC
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_distance_threshold_);
        seg.setMaxIterations(80);
        seg.setEpsAngle(0.2);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setInputCloud(cloud.makeShared());

        // Compute inliers (the plane = “road”)
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coeff);

        // Extract the road (inliers)
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);

        CloudIPtr road(new CloudI);
        extract.setNegative(false);
        extract.filter(*road);

        // Extract the obstacles (outliers)
        CloudIPtr obstacles(new CloudI);
        extract.setNegative(true);
        extract.filter(*obstacles);

        return {obstacles, road};
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

    std::vector<pcl::PointIndices>
    performClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles)
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

        // --- Build 2D convex hull of the road ---
        pcl::PointCloud<pcl::PointXYZI>::Ptr road_hull(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<pcl::Vertices> hull_polygons;
        {
            pcl::ConvexHull<pcl::PointXYZI> chull;
            chull.setDimension(2); // Only XY
            chull.setInputCloud(last_road_points_);
            chull.reconstruct(*road_hull, hull_polygons);
        }

        publishRoadHull(road_hull);

        // --- Check each cluster's centroid ---
        std::vector<pcl::PointIndices> cluster_indices;
        for (const auto &indices : all_cluster_indices)
        {
            float min_pt[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
            float max_pt[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
            float total_intensity = 0.0;
            float max_intensity = -FLT_MAX;
            float cx = 0, cy = 0;

            for (int idx : indices.indices)
            {
                const auto &pt = obstacles->points[idx];
                min_pt[0] = std::min(min_pt[0], pt.x);
                min_pt[1] = std::min(min_pt[1], pt.y);
                min_pt[2] = std::min(min_pt[2], pt.z);
                max_pt[0] = std::max(max_pt[0], pt.x);
                max_pt[1] = std::max(max_pt[1], pt.y);
                max_pt[2] = std::max(max_pt[2], pt.z);

                cx += pt.x;
                cy += pt.y;

                total_intensity += pt.intensity;
                max_intensity = std::max(max_intensity, pt.intensity);
            }

            const float size_x = max_pt[0] - min_pt[0];
            const float size_y = max_pt[1] - min_pt[1];
            const float size_z = max_pt[2] - min_pt[2];
            const float avg_intensity = total_intensity / indices.indices.size();
            const float centroid_x = cx / indices.indices.size();
            const float centroid_y = cy / indices.indices.size();

            // Basic cluster filtering (your original conditions)
            if (size_x > max_cluster_size_x_ ||
                size_y > max_cluster_size_y_ ||
                size_z < min_cluster_size_z_ ||
                max_pt[2] > max_cluster_z_height_ ||
                indices.indices.size() > static_cast<size_t>(max_selected_cluster_points_))
            {
                continue;
            }

            // Create point for centroid
            pcl::PointXYZI centroid_pt;
            centroid_pt.x = centroid_x;
            centroid_pt.y = centroid_y;
            centroid_pt.z = 0.0; // Z doesn't matter here
            // Scale down the road hull by a constant factor
            pcl::PointCloud<pcl::PointXYZI>::Ptr scaled_hull(new pcl::PointCloud<pcl::PointXYZI>);
            float scale_factor = 0.9; // Adjust this factor to make the hull smaller
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*road_hull, centroid);

            for (const auto &pt : road_hull->points)
            {
                pcl::PointXYZI scaled_pt;
                scaled_pt.x = centroid[0] + scale_factor * (pt.x - centroid[0]);
                scaled_pt.y = centroid[1] + scale_factor * (pt.y - centroid[1]);
                scaled_pt.z = pt.z; // Keep original Z
                scaled_hull->points.push_back(scaled_pt);
            }

            // Check if centroid is inside the scaled road hull
            pcl::PointCloud<pcl::PointXYZI>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            centroid_cloud->push_back(centroid_pt);

            pcl::CropHull<pcl::PointXYZI> crop;
            crop.setDim(2);
            crop.setInputCloud(centroid_cloud);
            crop.setHullCloud(scaled_hull);
            crop.setHullIndices(hull_polygons);
            crop.setCropOutside(true); // Keep points inside hull

            pcl::PointCloud<pcl::PointXYZI> filtered;
            crop.filter(filtered);

            if (!filtered.empty())
            {
                cluster_indices.push_back(indices); // Centroid is inside
            }
        }

        // RCLCPP_INFO(rclcpp::get_logger("obstacle_detector"), "Clusters min z: %f, max z: %f");

        return cluster_indices;
    }



    std::vector<pcl::PointCloud<pcl::PointXYZI>> publishClustersAndMarkers(const std::vector<pcl::PointIndices> &cluster_indices, const pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles, const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> clusters_rgb;
        clusters_rgb.height = 1;
        visualization_msgs::msg::MarkerArray markers;
        size_t id = 0;

        // std::vector<std::pair<size_t, double>> cluster_intensities;
        // for (size_t c = 0; c < cluster_indices.size(); ++c)
        // {
        //     const auto &indices = cluster_indices[c];
        //     double total_intensity = 0.0;
        //     for (int i : indices.indices)
        //     {
        //         total_intensity += obstacles->points[i].intensity;
        //     }
        //     double avg_intensity = total_intensity / indices.indices.size();
        //     cluster_intensities.emplace_back(c, avg_intensity);
        // }

        // // Sort clusters by average intensity in descending order
        // std::sort(cluster_intensities.begin(), cluster_intensities.end(), [](const auto &a, const auto &b)
        //           { return a.second > b.second; });

        // // Process and publish the top x clusters
        // int max_clusters = std::min(max_cluster_publish_, static_cast<int>(cluster_intensities.size()));
        int max_clusters = 100000;
        // size_t clusters_to_publish = std::min(cluster_intensities.size(), static_cast<size_t>(max_clusters));
        size_t clusters_to_publish = std::min(cluster_indices.size(), static_cast<size_t>(max_clusters));

        // RCLCPP_INFO(this->get_logger(), "Publishing %zu clusters as markers", clusters_to_publish);

        // Publish up to clusters_to_publish clusters (in order, not sorted)
        for (size_t i = 0; i < clusters_to_publish; ++i)
        {
            const auto &indices = cluster_indices[i];
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
            m.header.frame_id = "map";
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

        // Collect and return the actual clusters
        std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters_out;
        for (size_t i = 0; i < clusters_to_publish; ++i)
        {
            const auto &indices = cluster_indices[i];
            pcl::PointCloud<pcl::PointXYZI> cluster_cloud;
            for (int idx : indices.indices)
            {
                cluster_cloud.points.push_back(obstacles->points[idx]);
            }
            cluster_cloud.width = cluster_cloud.points.size();
            cluster_cloud.height = 1;
            cluster_cloud.is_dense = true;
            clusters_out.push_back(cluster_cloud);
        }
        return clusters_out;
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
        declare_parameter<int>("max_selected_cluster_points", 25);
        declare_parameter<double>("ransac_distance_threshold", 0.2);
        declare_parameter<double>("intensity_threshold", 100.0);
        declare_parameter<double>("max_cluster_size_x", 5.0);
        declare_parameter<double>("max_cluster_size_y", 5.0);
        declare_parameter<double>("min_cluster_size_z", 5.0);
        declare_parameter<double>("max_cluster_z_height", 0.5);
        declare_parameter<double>("inflation_radius", 2.0);
        declare_parameter<double>("inflation_radius_freespace", 1.0);
        declare_parameter<int>("clear_map_interval", 1);

        declare_parameter<float>("max_match_dist", 1.0);
        declare_parameter<int>("max_missed_frames", 10);
        declare_parameter<int>("min_age_frames", 2);
        declare_parameter<int>("width", 2000);
        declare_parameter<int>("height", 2000);
        declare_parameter<float>("offset_x", 600.0);
        declare_parameter<float>("offset_y", 400.0);
        declare_parameter<float>("resolution", 0.25);

        get_parameter("input_topic", input_topic_);
        get_parameter("obstacles_topic", obstacles_topic_);
        get_parameter("clusters_topic", clusters_topic_);
        get_parameter("max_cluster_publish", max_cluster_publish_);
        get_parameter("frame_id", frame_id_);
        get_parameter("cluster_tolerance", cluster_tolerance_);
        get_parameter("min_cluster_size", min_cluster_size_);
        get_parameter("max_cluster_size", max_cluster_size_);
        get_parameter("max_selected_cluster_points", max_selected_cluster_points_);
        get_parameter("ransac_distance_threshold", ransac_distance_threshold_);
        get_parameter("intensity_threshold", intensity_threshold_);
        get_parameter("max_cluster_size_x", max_cluster_size_x_);
        get_parameter("max_cluster_size_y", max_cluster_size_y_);
        get_parameter("min_cluster_size_z", min_cluster_size_z_);
        get_parameter("max_cluster_z_height", max_cluster_z_height_);
        get_parameter("inflation_radius", inflation_radius_);
        get_parameter("inflation_radius_freespace", inflation_radius_freespace_);
        get_parameter("clear_map_interval", clear_map_interval_);

        get_parameter("max_match_dist", max_match_dist_);
        get_parameter("max_missed_frames", max_missed_frames_);
        get_parameter("min_age_frames", min_age_frames_);
        get_parameter("width", width_);
        get_parameter("height", height_);
        get_parameter("offset_x", offset_x_);
        get_parameter("offset_y", offset_y_);
        get_parameter("resolution", resolution_);
    }

    void publishRoadHull(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &road_hull)
    {
        if (!hull_marker_pub_)
            return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = this->now();
        marker.ns = "road_hull";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05; // Line width

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        for (const auto &pt : road_hull->points)
        {
            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z; // keep original Z (or use 0)
            marker.points.push_back(p);
        }

        // Close the loop
        if (!road_hull->points.empty())
        {
            geometry_msgs::msg::Point first;
            first.x = road_hull->points[0].x;
            first.y = road_hull->points[0].y;
            first.z = road_hull->points[0].z;
            marker.points.push_back(first);
        }

        hull_marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr high_intensity_obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr road_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hull_marker_pub_;


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
    int max_selected_cluster_points_;
    double intensity_threshold_;
    double max_cluster_size_x_;
    double max_cluster_size_y_;
    double min_cluster_size_z_;
    double max_cluster_z_height_;

    double inflation_radius_; // Radius for inflation in map cells

    double inflation_radius_freespace_;
    int clear_map_interval_;

    std::vector<Track> tracks_;
    int next_track_id_ = 0;
    float max_match_dist_; // meters
    int max_missed_frames_;
    int min_age_frames_;

    int width_;        // 150 cells in width
    int height_;       // 150 cells in height
    float resolution_; // 0.5 m per cell
    float offset_x_;   // Offset in x direction for map origin
    float offset_y_;   // Offset in y direction for map origin

    std::vector<pcl::PointCloud<pcl::PointXYZI>> last_clusters_; // Store the last clusters
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_road_points_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}
