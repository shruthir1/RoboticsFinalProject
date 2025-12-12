#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Cell {
  int x;
  int y;
};

struct Human {
  double x;
  double y;
  int size;
};

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PerceptionNode::mapCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));

    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&PerceptionNode::amclCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&PerceptionNode::process, this));

    RCLCPP_INFO(get_logger(), "BFS Perception Node Started");
  }

private:
  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_;

  // Difference grid
  std::vector<int8_t> diff_grid_;
  std::vector<bool> visited_;

  const int FREE_THRESH = 20;
  const int MIN_COMPONENT_SIZE = 25;

  // ---------------- Callbacks ----------------

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = msg;
    diff_grid_.assign(map_->info.width * map_->info.height, 0);
    visited_.assign(map_->info.width * map_->info.height, false);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_ = msg;
  }

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    amcl_ = msg;
  }

  // ---------------- Utilities ----------------

  double getYaw(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(
      2.0 * (q.w * q.z),
      1.0 - 2.0 * q.z * q.z
    );
  }

  bool worldToMap(double wx, double wy, int &mx, int &my) {
    const auto &info = map_->info;
    mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    my = static_cast<int>((wy - info.origin.position.y) / info.resolution);
    return mx >= 0 && my >= 0 &&
           mx < static_cast<int>(info.width) &&
           my < static_cast<int>(info.height);
  }

  int idx(int x, int y) {
    return y * map_->info.width + x;
  }

  // ---------------- Main Processing ----------------

  void process() {
    if (!map_ || !scan_ || !amcl_) {
      RCLCPP_INFO(get_logger(), "Waiting for map / scan / amcl...");
      return;
    }

    std::fill(diff_grid_.begin(), diff_grid_.end(), 0);
    std::fill(visited_.begin(), visited_.end(), false);

    populateDifferenceGrid();
    auto humans = bfsConnectedComponents();

    if (humans.size() < 2) {
      RCLCPP_WARN(get_logger(), "Detected less than 2 humans");
      return;
    }

    std::sort(humans.begin(), humans.end(),
              [](const Human &a, const Human &b) {
                return a.size > b.size;
              });

    RCLCPP_INFO(get_logger(), "======= FINAL HUMAN POSITIONS =======");
    RCLCPP_INFO(get_logger(), "humanDetected[0] = (%.2f, %.2f)",
                humans[0].x, humans[0].y);
    RCLCPP_INFO(get_logger(), "humanDetected[1] = (%.2f, %.2f)",
                humans[1].x, humans[1].y);
    RCLCPP_INFO(get_logger(), "====================================");
  }

  // ---------------- Step 1: Difference Grid ----------------

  void populateDifferenceGrid() {
    double rx = amcl_->pose.pose.position.x;
    double ry = amcl_->pose.pose.position.y;
    double yaw = getYaw(amcl_->pose.pose.orientation);

    for (size_t i = 0; i < scan_->ranges.size(); i += 2) {
      double r = scan_->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < scan_->range_min || r > scan_->range_max) continue;

      double angle = scan_->angle_min + i * scan_->angle_increment;
      double wx = rx + r * std::cos(yaw + angle);
      double wy = ry + r * std::sin(yaw + angle);

      int mx, my;
      if (!worldToMap(wx, wy, mx, my)) continue;

      int map_val = map_->data[idx(mx, my)];
      if (map_val >= 0 && map_val <= FREE_THRESH) {
        diff_grid_[idx(mx, my)] = 1;
      }
    }
  }

  // ---------------- Step 2: BFS ----------------

  std::vector<Human> bfsConnectedComponents() {
    std::vector<Human> humans;

    for (int y = 0; y < (int)map_->info.height; y++) {
      for (int x = 0; x < (int)map_->info.width; x++) {
        int id = idx(x, y);
        if (diff_grid_[id] == 1 && !visited_[id]) {
          Human h = bfsFromCell(x, y);
          if (h.size >= MIN_COMPONENT_SIZE)
            humans.push_back(h);
        }
      }
    }
    return humans;
  }

  Human bfsFromCell(int sx, int sy) {
    std::queue<Cell> q;
    q.push({sx, sy});
    visited_[idx(sx, sy)] = true;

    int count = 0;
    double sum_x = 0.0, sum_y = 0.0;

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!q.empty()) {
      Cell c = q.front(); q.pop();
      count++;

      sum_x += c.x;
      sum_y += c.y;

      for (int i = 0; i < 4; i++) {
        int nx = c.x + dx[i];
        int ny = c.y + dy[i];
        if (nx < 0 || ny < 0 ||
            nx >= (int)map_->info.width ||
            ny >= (int)map_->info.height)
          continue;

        int nid = idx(nx, ny);
        if (diff_grid_[nid] == 1 && !visited_[nid]) {
          visited_[nid] = true;
          q.push({nx, ny});
        }
      }
    }

    double wx = map_->info.origin.position.x +
                (sum_x / count + 0.5) * map_->info.resolution;
    double wy = map_->info.origin.position.y +
                (sum_y / count + 0.5) * map_->info.resolution;

    return {wx, wy, count};
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
