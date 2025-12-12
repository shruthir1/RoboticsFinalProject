#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <thread>
#include <algorithm>

// -------------------- Small helpers --------------------
static inline double dist2(double ax, double ay, double bx, double by) {
  double dx = ax - bx;
  double dy = ay - by;
  return dx*dx + dy*dy;
}

struct CandidateCluster {
  double cx = 0.0;
  double cy = 0.0;
  int count = 0;

  double min_x = 1e9, max_x = -1e9;
  double min_y = 1e9, max_y = -1e9;

  void add(double x, double y) {
    cx += x;
    cy += y;
    count++;
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }

  void finalize() {
    if (count > 0) {
      cx /= count;
      cy /= count;
    }
  }

  double diag() const {
    double dx = max_x - min_x;
    double dy = max_y - min_y;
    return std::sqrt(dx*dx + dy*dy);
  }

  double aspect() const {
    double w = std::max(1e-6, max_x - min_x);
    double h = std::max(1e-6, max_y - min_y);
    double a = w / h;
    return (a < 1.0) ? (1.0 / a) : a; // >= 1
  }
};

struct Track {
  bool active = false;
  bool locked = false;
  double x = 0.0;
  double y = 0.0;

  int hits = 0;                 // how many times matched
  int consecutive = 0;          // consecutive matches
  rclcpp::Time last_seen;       // last time we saw it
};

class PerceptionNode : public rclcpp::Node {
public:
  PerceptionNode() : Node("perception_node") {

    // QoS for static map
    rclcpp::QoS map_qos(1);
    map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    static_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      std::bind(&PerceptionNode::staticMapCallback, this, std::placeholders::_1));

    pose_subscriber = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&PerceptionNode::amclCallback, this, std::placeholders::_1));

    scan_subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));

    humans_publisher = create_publisher<geometry_msgs::msg::PoseArray>("/humans_moved", 10);

    RCLCPP_INFO(get_logger(), "Perception node started (2-human lock + final print).");
    waitForInitialPose();
  }

private:
  // ---------------- ROS ----------------
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr humans_publisher;

  // ---------------- State ----------------
  nav_msgs::msg::OccupancyGrid initial_map;
  geometry_msgs::msg::Pose robot_pose;

  bool has_initial_map = false;
  bool has_pose = false;

  // final output (exactly two)
  std::vector<geometry_msgs::msg::Pose> humanDetected;
  bool finished_ = false;

  // two tracks
  Track tracks_[2];

  // ---------------- Tuning knobs ----------------
  const double MIN_RANGE = 0.6;
  const double MAX_RANGE = 8.0;

  // clustering + shape filters
  const double CLUSTER_TOL = 0.45;    // meters between consecutive points
  const int    MIN_POINTS  = 8;       // must have >= this many scan points
  const double MAX_DIAG    = 1.0;     // cluster diagonal (meters)
  const double MAX_ASPECT  = 3.0;     // bounding box aspect ratio (>=1)

  // static filter
  const int WALL_RADIUS_CELLS = 3;    // neighborhood radius in map cells
  const int STATIC_OCC_THRESH = 50;   // map cell occupancy threshold

  // tracking
  const double ASSOC_DIST = 1.2;      // association distance to existing track
  const int    LOCK_CONSEC = 5;       // consecutive matches to lock
  const double TRACK_TIMEOUT = 2.0;   // seconds without matches -> drop track
  const double EMA_ALPHA = 0.35;      // smoothing for track updates

  // ---------------- Callbacks ----------------
  void staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!has_initial_map) {
      initial_map = *msg;
      has_initial_map = true;
      RCLCPP_INFO(get_logger(), "Static map stored.");
    }
  }

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    robot_pose = msg->pose.pose;
    has_pose = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (finished_) return;
    if (!has_initial_map || !has_pose) return;

    // 1) Build candidate clusters from scan points that are NOT near static obstacles
    std::vector<CandidateCluster> candidates = detectCandidatesFromScan(*scan);

    // 2) Update 2 tracks using candidates
    updateTracks(candidates);

    // 3) If both tracks are locked -> finalize and print once
    if (tracks_[0].locked && tracks_[1].locked) {
      finalizeAndPrint();
      finished_ = true;
    }
  }

  // ---------------- Core: detect candidates ----------------
  std::vector<CandidateCluster> detectCandidatesFromScan(const sensor_msgs::msg::LaserScan& scan) {
    std::vector<CandidateCluster> out;

    const double rx = robot_pose.position.x;
    const double ry = robot_pose.position.y;
    const double yaw = tf2::getYaw(robot_pose.orientation);

    bool have_current = false;
    CandidateCluster current;

    auto flush_current = [&]() {
      if (!have_current) return;
      current.finalize();

      // filters (Python-style + extra shape filters)
      if (current.count >= MIN_POINTS &&
          current.diag() <= MAX_DIAG &&
          current.aspect() <= MAX_ASPECT) {
        out.push_back(current);
      }

      current = CandidateCluster();
      have_current = false;
    };

    for (size_t i = 0; i < scan.ranges.size(); i++) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < MIN_RANGE || r > MAX_RANGE) {
        flush_current();
        continue;
      }

      const double angle = yaw + scan.angle_min + (double)i * scan.angle_increment;
      const double wx = rx + (double)r * std::cos(angle);
      const double wy = ry + (double)r * std::sin(angle);

      // static-map neighborhood rejection
      if (isWall(wx, wy)) {
        flush_current();
        continue;
      }

      // scan-order clustering (distance to previous accepted point)
      if (!have_current) {
        current.add(wx, wy);
        have_current = true;
      } else {
        // distance from last point in cluster: approximate using cluster mean of last added point
        // (we can store last point, but this keeps code simple)
        // Better: store last_x/last_y. We'll do that:
        // We'll track last accepted point explicitly.
      }
    }

    // The above loop needs last-point tracking; implement properly below.
    // Re-run with correct last-point tracking:
    out.clear();
    have_current = false;
    current = CandidateCluster();

    double last_x = 0.0, last_y = 0.0;

    for (size_t i = 0; i < scan.ranges.size(); i++) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < MIN_RANGE || r > MAX_RANGE) {
        flush_current();
        continue;
      }

      const double angle = yaw + scan.angle_min + (double)i * scan.angle_increment;
      const double wx = rx + (double)r * std::cos(angle);
      const double wy = ry + (double)r * std::sin(angle);

      if (isWall(wx, wy)) {
        flush_current();
        continue;
      }

      if (!have_current) {
        current.add(wx, wy);
        last_x = wx; last_y = wy;
        have_current = true;
      } else {
        const double d = std::sqrt(dist2(wx, wy, last_x, last_y));
        if (d <= CLUSTER_TOL) {
          current.add(wx, wy);
          last_x = wx; last_y = wy;
        } else {
          flush_current();
          current.add(wx, wy);
          last_x = wx; last_y = wy;
          have_current = true;
        }
      }
    }

    flush_current();

    return out;
  }

  // ---------------- Static neighborhood filter ----------------
  bool isWall(double wx, double wy) {
    // world -> map grid
    int mx = (int)((wx - initial_map.info.origin.position.x) / initial_map.info.resolution);
    int my = (int)((wy - initial_map.info.origin.position.y) / initial_map.info.resolution);

    if (mx < 0 || my < 0 ||
        mx >= (int)initial_map.info.width ||
        my >= (int)initial_map.info.height) {
      return true; // conservative: outside map is "wall"
    }

    for (int dx = -WALL_RADIUS_CELLS; dx <= WALL_RADIUS_CELLS; dx++) {
      for (int dy = -WALL_RADIUS_CELLS; dy <= WALL_RADIUS_CELLS; dy++) {
        int nx = mx + dx;
        int ny = my + dy;
        if (nx < 0 || ny < 0 ||
            nx >= (int)initial_map.info.width ||
            ny >= (int)initial_map.info.height) {
          continue;
        }
        int idx = ny * initial_map.info.width + nx;
        int v = initial_map.data[idx];
        if (v > STATIC_OCC_THRESH) return true;
      }
    }
    return false;
  }

  // ---------------- Tracking + locking ----------------
  void updateTracks(const std::vector<CandidateCluster>& cands) {
    const rclcpp::Time now = this->now();

    // Mark all tracks as not matched initially
    bool matched_track[2] = {false, false};

    // Greedy association: sort candidates by count (strongest first)
    std::vector<size_t> order(cands.size());
    for (size_t i = 0; i < cands.size(); i++) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b){
      return cands[a].count > cands[b].count;
    });

    for (size_t oi = 0; oi < order.size(); oi++) {
      const auto& c = cands[order[oi]];

      // Find best matching track
      int best_t = -1;
      double best_d2 = 1e18;

      for (int t = 0; t < 2; t++) {
        if (!tracks_[t].active) continue;
        double d2 = dist2(c.cx, c.cy, tracks_[t].x, tracks_[t].y);
        if (d2 < best_d2) {
          best_d2 = d2;
          best_t = t;
        }
      }

      // If a match is close enough and track not already matched, update it
      if (best_t != -1 && !matched_track[best_t] && std::sqrt(best_d2) <= ASSOC_DIST) {
        // EMA update
        tracks_[best_t].x = (1.0 - EMA_ALPHA) * tracks_[best_t].x + EMA_ALPHA * c.cx;
        tracks_[best_t].y = (1.0 - EMA_ALPHA) * tracks_[best_t].y + EMA_ALPHA * c.cy;
        tracks_[best_t].hits++;
        tracks_[best_t].consecutive++;
        tracks_[best_t].last_seen = now;
        matched_track[best_t] = true;

        if (!tracks_[best_t].locked && tracks_[best_t].consecutive >= LOCK_CONSEC) {
          tracks_[best_t].locked = true;
          RCLCPP_WARN(get_logger(), "Locked Track %d at (%.2f, %.2f)", best_t, tracks_[best_t].x, tracks_[best_t].y);
        }

        continue;
      }

      // Otherwise, consider starting a new track (in an inactive slot)
      for (int t = 0; t < 2; t++) {
        if (!tracks_[t].active) {
          tracks_[t].active = true;
          tracks_[t].locked = false;
          tracks_[t].x = c.cx;
          tracks_[t].y = c.cy;
          tracks_[t].hits = 1;
          tracks_[t].consecutive = 1;
          tracks_[t].last_seen = now;
          matched_track[t] = true;

          RCLCPP_INFO(get_logger(), "Started Track %d at (%.2f, %.2f) (count=%d)", t, c.cx, c.cy, c.count);
          break;
        }
      }
    }

    // Handle unmatched tracks: decay / timeout
    for (int t = 0; t < 2; t++) {
      if (!tracks_[t].active) continue;

      if (!matched_track[t]) {
        // missed this scan
        tracks_[t].consecutive = 0;
        double dt = (now - tracks_[t].last_seen).seconds();
        if (dt > TRACK_TIMEOUT) {
          RCLCPP_INFO(get_logger(), "Dropping Track %d (timeout)", t);
          tracks_[t] = Track(); // reset
        }
      }
    }
  }

  // ---------------- Finalize output ----------------
  void finalizeAndPrint() {
    humanDetected.clear();

    // Make a stable order (e.g., sort by x then y)
    std::vector<std::pair<double,double>> pts = {
      {tracks_[0].x, tracks_[0].y},
      {tracks_[1].x, tracks_[1].y}
    };
    std::sort(pts.begin(), pts.end(), [](auto& a, auto& b){
      if (a.first != b.first) return a.first < b.first;
      return a.second < b.second;
    });

    for (int i = 0; i < 2; i++) {
      geometry_msgs::msg::Pose p;
      p.position.x = pts[i].first;
      p.position.y = pts[i].second;
      p.orientation.w = 1.0;
      humanDetected.push_back(p);
    }

    // Print final results
    RCLCPP_WARN(get_logger(), "==============================");
    RCLCPP_WARN(get_logger(), "FINAL humanDetected (2):");
    RCLCPP_WARN(get_logger(), "  Human 1: (%.2f, %.2f)", humanDetected[0].position.x, humanDetected[0].position.y);
    RCLCPP_WARN(get_logger(), "  Human 2: (%.2f, %.2f)", humanDetected[1].position.x, humanDetected[1].position.y);
    RCLCPP_WARN(get_logger(), "==============================");

    // Publish once
    geometry_msgs::msg::PoseArray arr;
    arr.header.frame_id = "map";
    arr.poses = humanDetected;
    humans_publisher->publish(arr);
  }

  // ---------------- Wait for pose ----------------
  void waitForInitialPose() {
    RCLCPP_INFO(get_logger(), "Waiting for AMCL pose...");
    while (!has_pose) {
      rclcpp::spin_some(get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(get_logger(), "Initial pose received.");
  }
};

// -------------------- main --------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
