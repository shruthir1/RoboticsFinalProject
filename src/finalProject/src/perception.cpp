#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <vector>
#include <iterator>

#include <iostream>



class PerceptionNode : public rclcpp::Node{
public:
  PerceptionNode() : Node("perception_node"), scan_count_(0) {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));
      std::cout << "scan sub check" << std::endl;

          // Subscribe to odometry to obtain the robot's current pose
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&PerceptionNode::amclCallback, this, std::placeholders::_1));
    std::cout << "amcl sub check" << std::endl;

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PerceptionNode::mapCallback, this, std::placeholders::_1));
        std::cout << "map sub check" << std::endl;

    local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10,
      std::bind(&PerceptionNode::localMapCallback, this, std::placeholders::_1));
              std::cout << "local sub map check" << std::endl;

    
    global_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", 10,
      std::bind(&PerceptionNode::mapCallback, this, std::placeholders::_1));
              std::cout << "global sub map check" << std::endl;


    humans_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/humans_moved", 10);


    // initialize robot pose (identity orientation)
    RCLCPP_INFO(get_logger(), "PerceptionNode started (fake detection mode).");
  }

private:
  void humanFinder(const geometry_msgs::msg::Pose rPose){

    float rx = rPose.position.x;
    float ry = rPose.position.y;
    
    int8_t mapData[globalMap.data.size()];

    int phantomCount = 0;

    for(size_t i=0; i < latestScan.ranges.size(); i++){
      

      float angle = latestScan.angle_min + (i * latestScan.angle_increment) - 1.5708; //add robot yaw??? 
      //LIDAR IS OFF IN 90* DIRECTION FROM ROBOT FRONT; ROTATE 90 DEGREES CLOCKWISE 
      float range = latestScan.ranges[i];

      if(range < latestScan.range_min || range >= 5 ){
        continue; // invalid range
      } else {

              // Convert to Cartesian coordinates in robot frame
          float x_r = rx + (range * cos(angle));
          float y_r = ry + (range * sin(angle));

           std::cout << "scan point at x_m:" << x_r << " || y_m:" << y_r << " || robotX: " << rx << "|| robotY:" << ry << " || angle increment" << latestScan.angle_increment << " || Free cell:" << freeCell(globalMap, x_r, y_r) << "|| Range:" << range << " || Angle:" << angle << " || rangemin:" << latestScan.range_min << " || rangeMax: " << latestScan.range_max << std::endl;

          if(freeCell(latestMap, static_cast<int>(x_r), static_cast<int>(y_r)) == true){ {

          // if((x_r <= 1.6 && x_r >= 0.8) && (y_r >= -1.6 && y_r <= -0.8)){
            std::cout << "phantom ping at x_m:" << x_r << " || y_m:" << y_r << std::endl;

          }
      }
    }
   }
   falsifyAllInputs();
  }

  void falsifyAllInputs(){
    mapRecieved = false;
    localMapRecieved = false;
    poseCallbackRecieved = false;
    scanCallbackRecieved = false;


  }

  void validChecker(){
    if(mapRecieved && localMapRecieved && poseCallbackRecieved && scanCallbackRecieved){
      std::cout << "All inputs recieved" << std::endl;

      humanFinder(robot_pose_);

      
    } else {
      std::cout << "Waiting for all inputs..." << std::endl;
    }
  }

  bool freeCell(const nav_msgs::msg::OccupancyGrid map, int x, int y){

    int mX = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    int mY = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);

    if(mX < 0 || mY < 0 || mX >= static_cast<int>(map.info.width) || mY >= static_cast<int>(map.info.height)){
      return false; // out of bounds
    }

    int8_t cellValue = map.data[mY * map.info.width + mX];

    return (cellValue == 0); // free if value is 0

  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    (void)msg;  // unused for now

    // Fake detection: after 10 scans, publish a "moved human" pose once
    scan_count_++;
    latestScan = *msg;

    scanCallbackRecieved = true;

    validChecker();
    // std::cout << "scancheck" << std::endl;

    // std::cout << "finderfinish" << std::endl;


    // if (scan_count_ == 10) {
    //   geometry_msgs::msg::PoseArray arr;
    //   arr.header.stamp = this->now();
    //   arr.header.frame_id = "map";  // important: Nav2 expects map frame

    //   geometry_msgs::msg::Pose p;
    //   p.position.x = 6.5;   // <- change to something inside your warehouse map
    //   p.position.y = -15.0; //    so you see robot move there
    //   p.position.z = 0.0;
    //   p.orientation.w = 1.0; // facing default direction

    //   arr.poses.push_back(p);
    //   humans_pub_->publish(arr);

    //   RCLCPP_WARN(get_logger(),
    //               "PerceptionNode: FAKE moved-human pose published at (%.2f, %.2f)",
    //               p.position.x, p.position.y);

    // }
  }

  // amcl callback: store the robot pose for use in processing
  void amclCallback(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> msg){
  
    robot_pose_ = msg->pose.pose; // copy pose (position + orientation)
    poseCallbackRecieved = true;
    validChecker();

    std::cout << "Updated robot pose from /amcl_pose:" << robot_pose_.position.x << ", " << robot_pose_.position.y << std::endl;
  }

  void localMapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg){
    localMapRecieved = true;
    localMap = *msg;
    validChecker();

  }

  void mapCallback(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg){
        std::cout << "globalmapcheck" << std::endl;

    if(initialMapRecieved != true){
      initialMapRecieved = true;
      globalMap = *msg;
    }

    mapRecieved = true;
    latestMap = *msg;
    validChecker();


  }

  bool initialMapRecieved = false;

  bool mapRecieved = false;
  bool localMapRecieved = false;
  bool poseCallbackRecieved = false;
  bool scanCallbackRecieved = false;

  sensor_msgs::msg::LaserScan latestScan;

  nav_msgs::msg::OccupancyGrid latestMap;
  nav_msgs::msg::OccupancyGrid globalMap;

  nav_msgs::msg::OccupancyGrid localMap;


  geometry_msgs::msg::Point anticipatedHumans[2];
  geometry_msgs::msg::Point detectedHumans[2];

  int humanCount = 0;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr humans_pub_;
  int scan_count_;

  // current robot pose (from /odom)
  geometry_msgs::msg::Pose robot_pose_;
  // std::mutex robot_pose_mutex_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
