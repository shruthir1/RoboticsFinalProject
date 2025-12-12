#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/utils.h>

#include <vector>
#include <cmath>
#include <thread>

// -------------------- Cluster Struct --------------------
struct HumanCluster {
    float x_sum = 0.0;
    float y_sum = 0.0;
    int count = 0;

    float min_x = 1e9, max_x = -1e9;
    float min_y = 1e9, max_y = -1e9;

    void add(float x, float y) {
        x_sum += x;
        y_sum += y;
        count++;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }

    float cx() const { return x_sum / count; }
    float cy() const { return y_sum / count; }

    float width() const {
        float dx = max_x - min_x;
        float dy = max_y - min_y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

// -------------------- Perception Node --------------------
class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {

        rclcpp::QoS map_qos(1);
        map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos,
            std::bind(&PerceptionNode::mapCallback, this, std::placeholders::_1));

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&PerceptionNode::poseCallback, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));

        humans_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/humans_moved", 10);

        RCLCPP_INFO(get_logger(), "Perception node started.");
        waitForPose();
    }

private:
    // -------------------- ROS --------------------
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr humans_pub_;

    // -------------------- State --------------------
    nav_msgs::msg::OccupancyGrid static_map_;
    geometry_msgs::msg::Pose robot_pose_;
    bool has_map_ = false;
    bool has_pose_ = false;

    // -------------------- Callbacks --------------------
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!has_map_) {
            static_map_ = *msg;
            has_map_ = true;
            RCLCPP_INFO(get_logger(), "Static map received.");
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
        has_pose_ = true;
    }

    // -------------------- Core Scan Logic --------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!has_map_ || !has_pose_) return;

        std::vector<std::pair<float,float>> dynamic_pts;

        float rx = robot_pose_.position.x;
        float ry = robot_pose_.position.y;
        float yaw = tf2::getYaw(robot_pose_.orientation);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float r = scan->ranges[i];
            if (!std::isfinite(r) || r < 0.5 || r > 8.0) continue;

            float angle = yaw + scan->angle_min + i * scan->angle_increment;
            float wx = rx + r * std::cos(angle);
            float wy = ry + r * std::sin(angle);

            if (isWall(wx, wy)) continue;

            dynamic_pts.emplace_back(wx, wy);
        }

        auto clusters = clusterPoints(dynamic_pts);
        publishHumans(clusters);
    }

    // -------------------- Static Filter --------------------
    bool isWall(float wx, float wy) {
        int mx = (wx - static_map_.info.origin.position.x) / static_map_.info.resolution;
        int my = (wy - static_map_.info.origin.position.y) / static_map_.info.resolution;

        if (mx < 0 || my < 0 ||
            mx >= (int)static_map_.info.width ||
            my >= (int)static_map_.info.height)
            return true;

        const int R = 3;
        for (int dx = -R; dx <= R; dx++) {
            for (int dy = -R; dy <= R; dy++) {
                int nx = mx + dx;
                int ny = my + dy;
                if (nx < 0 || ny < 0 ||
                    nx >= (int)static_map_.info.width ||
                    ny >= (int)static_map_.info.height)
                    continue;

                int idx = ny * static_map_.info.width + nx;
                if (static_map_.data[idx] > 50)
                    return true;
            }
        }
        return false;
    }

    // -------------------- Clustering --------------------
    std::vector<HumanCluster> clusterPoints(
        const std::vector<std::pair<float,float>>& pts) {

        std::vector<HumanCluster> clusters;
        if (pts.empty()) return clusters;

        const float CLUSTER_DIST = 0.5;

        HumanCluster current;
        current.add(pts[0].first, pts[0].second);

        for (size_t i = 1; i < pts.size(); i++) {
            float dx = pts[i].first - pts[i-1].first;
            float dy = pts[i].second - pts[i-1].second;
            float d = std::sqrt(dx*dx + dy*dy);

            if (d < CLUSTER_DIST) {
                current.add(pts[i].first, pts[i].second);
            } else {
                if (current.count >= 6 && current.width() <= 1.0)
                    clusters.push_back(current);
                current = HumanCluster();
                current.add(pts[i].first, pts[i].second);
            }
        }

        if (current.count >= 6 && current.width() <= 1.0)
            clusters.push_back(current);

        return clusters;
    }

    // -------------------- Publish --------------------
    void publishHumans(const std::vector<HumanCluster>& clusters) {
        if (clusters.empty()) return;

        geometry_msgs::msg::PoseArray msg;
        msg.header.frame_id = "map";

        for (const auto& c : clusters) {
            geometry_msgs::msg::Pose p;
            p.position.x = c.cx();
            p.position.y = c.cy();
            p.orientation.w = 1.0;
            msg.poses.push_back(p);
        }

        humans_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published %zu moved human(s).", clusters.size());
    }

    // -------------------- Wait for Pose --------------------
    void waitForPose() {
        RCLCPP_INFO(get_logger(), "Waiting for AMCL pose...");
        while (!has_pose_) {
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

// -------------------- Main --------------------
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
