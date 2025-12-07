#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PerceptionNode : public rclcpp::Node{
public:
  PerceptionNode() : Node("perception_node"), scan_count_(0) {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));

    humans_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/humans_moved", 10);

    RCLCPP_INFO(get_logger(), "PerceptionNode started (fake detection mode).");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    (void)msg;  // unused for now

    // Fake detection: after 10 scans, publish a "moved human" pose once
    scan_count_++;
    if (scan_count_ == 10) {
      geometry_msgs::msg::PoseArray arr;
      arr.header.stamp = this->now();
      arr.header.frame_id = "map";  // important: Nav2 expects map frame

      geometry_msgs::msg::Pose p;
      p.position.x = 6.5;   // <- change to something inside your warehouse map
      p.position.y = -15.0; //    so you see robot move there
      p.position.z = 0.0;
      p.orientation.w = 1.0; // facing default direction

      arr.poses.push_back(p);
      humans_pub_->publish(arr);

      RCLCPP_WARN(get_logger(),
                  "PerceptionNode: FAKE moved-human pose published at (%.2f, %.2f)",
                  p.position.x, p.position.y);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr humans_pub_;
  int scan_count_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
