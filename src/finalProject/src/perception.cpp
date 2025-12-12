#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
        // 1. Initialize TF2 Buffer and Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. Subscriptions
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&PerceptionNode::mapCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));
        
        // amcl_pose_sub_ is not strictly needed for the /amcl_pose topic itself
        // because we will use the tf listener to get the transform from 'base_link' to 'map'.
        // However, subscribing to it ensures we know when localization starts working.
        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&PerceptionNode::amclPoseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Perception Node Initialized. Waiting for map...");
    }

private:
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr static_map_;
    bool map_received_ = false;

    // TF2 members
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;

    // --- Callbacks ---

    void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
    {
        if (!map_received_) {
            static_map_ = msg;
            map_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Static map received. Map size: %d x %d (resolution: %.3f)",
                        static_map_->info.width, static_map_->info.height, static_map_->info.resolution);
        }
    }

    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
    {
        // This callback is mainly to confirm localization is running.
        // The actual pose for scan processing is retrieved via TF.
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        if (!map_received_) {
            return; // Cannot detect changes without the map
        }
        
        geometry_msgs::msg::TransformStamped transform;
        try {
            // Get the transform from the robot's laser frame (scan_msg->header.frame_id) 
            // to the map frame (static_map_->header.frame_id)
            transform = tf_buffer_->lookupTransform(
                static_map_->header.frame_id, 
                scan_msg->header.frame_id,
                scan_msg->header.stamp, 
                100ms // Timeout
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                        scan_msg->header.frame_id.c_str(), static_map_->header.frame_id.c_str(), ex.what());
            return;
        }

        std::vector<geometry_msgs::msg::Point> anomaly_points;

        // 3. Process each laser point
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
            
            // Ignore points outside min/max range or invalid
            if (std::isinf(range) || std::isnan(range) || range < scan_msg->range_min || range > scan_msg->range_max) {
                continue;
            }

            // Calculate angle for this reading
            double angle = scan_msg->angle_min + (double)i * scan_msg->angle_increment;

            // Convert polar (range, angle) to Cartesian (x, y) in the laser frame
            geometry_msgs::msg::PointStamped laser_point;
            laser_point.header.frame_id = scan_msg->header.frame_id;
            laser_point.point.x = range * std::cos(angle);
            laser_point.point.y = range * std::sin(angle);
            laser_point.point.z = 0.0;

            // Transform point from laser frame to map frame
            geometry_msgs::msg::PointStamped map_point_stamped;
            tf2::doTransform(laser_point, map_point_stamped, transform);
            geometry_msgs::msg::Point map_point = map_point_stamped.point;
            
            // 4. Check the corresponding cell in the static map
            int map_cell_value = getMapCell(map_point.x, map_point.y);

            // Anomaly Detection Logic:
            // - A sensor reading that lands on a cell marked as 'FREE' (0) 
            //   in the static map is a new obstacle.
            if (map_cell_value == 0) {
                anomaly_points.push_back(map_point);
            }
        }

        // 5. Report Anomalies (Simple Clustering/Filtering)
        if (!anomaly_points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Detected %lu anomalous points!", anomaly_points.size());
            
            // Simple thresholding: require at least N points to form a cluster (a human)
            const size_t MIN_ANOMALY_POINTS = 50; 
            if (anomaly_points.size() > MIN_ANOMALY_POINTS) {
                
                // For a more robust solution, you would use clustering (e.g., DBSCAN) here.
                // For this project, we'll use a simple centroid calculation for all points.
                // Since there are two humans, a better approach is needed, but 
                // for a basic "a change has occurred" check, this works.
                
                double avg_x = 0, avg_y = 0;
                for (const auto& p : anomaly_points) {
                    avg_x += p.x;
                    avg_y += p.y;
                }
                avg_x /= anomaly_points.size();
                avg_y /= anomaly_points.size();

                RCLCPP_ERROR(this->get_logger(), 
                             "MAJOR CHANGE DETECTED! Potential new location at: (%.2f, %.2f) in map frame.", 
                             avg_x, avg_y);
                
                // Since there are TWO humans, you would need to implement a simple 
                // clustering algorithm (like k-means with k=2 or a proximity-based clusterer) 
                // to find the two separate locations.
                
                // Once detected, you should stop navigating and report the location(s).
                // For demonstration, we just report the centroid and stop the node.
                rclcpp::shutdown();
            }
        }
    }

    // Helper function to get the value of a cell in the OccupancyGrid
    int getMapCell(double x, double y)
    {
        if (!static_map_) return -1;

        // Convert map coordinates (m) to grid cell indices
        double origin_x = static_map_->info.origin.position.x;
        double origin_y = static_map_->info.origin.position.y;
        double resolution = static_map_->info.resolution;

        // x_cell = floor((x_world - x_origin) / resolution)
        int i = static_cast<int>(std::floor((x - origin_x) / resolution));
        int j = static_cast<int>(std::floor((y - origin_y) / resolution));

        // Check bounds
        if (i < 0 || i >= static_map_->info.width || j < 0 || j >= static_map_->info.height) {
            return -1; // Out of bounds
        }

        // Calculate 1D index: index = i + j * width
        int index = i + j * static_map_->info.width;

        // Map data values: 100 (Occupied), 0 (Free), -1 (Unknown)
        return static_map_->data[index];
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}