#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> //laser messages
#include <geometry_msgs/msg/pose_array.hpp> //array of human pos msg type
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <vector>
#include <cmath>
#include <iostream>

//implementing clustering can help with reducing noise (hopefully) 
struct HumanCluster {
    //sum of all x and y y coordinates in the cluster respectively
    //the point of this is for the centeroid calculations 
    float x_sum = 0.0;
    float y_sum = 0.0;
    int count = 0; //count how many points we've seen -> also for centroid calc 

    //centroid x calculation 
    float cx() const { 
        return x_sum / count; 
    }

    //centroid y calculation
    float cy() const { 
        return y_sum / count;
     }
};

 
class PerceptionNode : public rclcpp::Node{
public:
  PerceptionNode() : Node("perception_node") {
     //subscribe to laserScan to get information from this topic
     //data that we can get from laserScan: 
     scan_subscriber = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));
     //subscribing to amcl
     //data that we can obtain from amcl: 
     pose_subscriber = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, std::bind(&PerceptionNode::amclCallback, this, std::placeholders::_1));
     //subscribing to a STATIC map to have reference of intial env
     //data we can obtain from map: 
     static_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PerceptionNode::staticMapCallback, this, std::placeholders::_1));
     //need to publish human information to other nodes like navigation 
     humans_publisher = create_publisher<geometry_msgs::msg::PoseArray>("/humans_moved", 10);
     //subscribing to the map that changes when we make changes 
     dynamic_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>( "/local_costmap/costmap", 10, std::bind(&PerceptionNode::dynamicMapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PerceptionNode is starting!!");
  }
  
  private: 
    //variables for the callback functions:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr humans_publisher;
    bool has_initial_map = false;
    bool has_latest_map = false;
    bool has_pose = false;
    int humanCount = 0;
    nav_msgs::msg::OccupancyGrid initial_map;
    nav_msgs::msg::OccupancyGrid latest_map;
    geometry_msgs::msg::Pose robot_pose;
    std::vector<geometry_msgs::msg::Pose> detected_humans;
    std::vector<HumanCluster> clusters; //build clusters while we scan and pusback here 


    //implementing all callback functions for the subscribers, callback functions serve as event handlers for each time we get new information from that topic 
    
    //callback for amcl subscription
    //info I can get from /amcl: timestamps with the new pose 
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        //everytime the topic is updated with the robots pose, we store it here
        robot_pose = msg->pose.pose;
        //do we currently have a pose? if we're in this callback, then yes 
        has_pose = true;
        RCLCPP_WARN(get_logger(),  "Have current pose");

    }
    //callback function for subscribing to /scan
    //info I can get from /scan: range, angle_increment, angle_min, angle_max 
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        //if we dont have the initial map or we dont have the current pose then we cant utilize scanner
        if(!has_latest_map || !has_pose){
           RCLCPP_WARN(get_logger(),  "dont have enough info for scanner have map: %d, have pose: %d", has_latest_map, has_pose); 

          return;
        }
        
        //clear our cluster data before we take a new scan 
        clusters.clear();

        //to localize scan we need to know coordinates of the robot 
        float rx = robot_pose.position.x; //robot x coordinate from amcl
        float ry = robot_pose.position.y; //robot y coordinate from amcl
        //used transform library to getYaw as well 
        float yaw = tf2::getYaw(robot_pose.orientation); //robot yaw from amcl (rotational placement)

        //scan for humans 
        //checking all the ranges we have but we only want to look at ranges less than min or greater than max 
        for(size_t i =0; i < msg->ranges.size(); i++){
            //ranges is the distance from sensor to the point it measured 
            float range = msg->ranges[i];
            if (range < msg->range_min || range >= msg->range_max) continue;
            //robots angle in robots frame -> in polar coord
            float r_angle_rframe = msg->angle_min + i * msg->angle_increment;
            
            //robots angle in global frame -> in polar coord
            float r_angle_wframe = r_angle_rframe + yaw + 1.5708; //minus 90 or some sort of translation to allign the frame 

            //convert polar coordinates to cartesian :)
            float wx = rx + range * cos(r_angle_wframe); // x coordinate of the global position (i think)
            float wy = ry + range * sin(r_angle_wframe); // y coordinate of the global position (i think)
            
            //if there is a change in the map its likely human was found 
            if(changeHappened(wx, wy)){
              RCLCPP_WARN(get_logger(),  "Change in map at (%.2f, %.2f)", wx, wy);  
              addClusters(wx, wy);
                //we want to record the pose of where human was found 
                // geometry_msgs::msg::Pose p;
                // //global (x,y) where human was found 
                // p.position.x = wx;
                // p.position.y = wy;
                // p.orientation.w = 1.0;
                // // record this pose!
                // detected_humans.push_back(p);

                //better to do clustering to avoid recording a false positive pose 



                RCLCPP_WARN(get_logger(),  "Detected possible human position at (%.2f, %.2f)", wx, wy);
                humanCount++;


                // if(humanCount == 2){
                //     //need finish function 
                // }
            }

        }

       if (!clusters.empty()) {
            publishDetectedHuman();
        }
    }

    //callback function for /map subscition 
    //info I can get from /map: 
    void staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(get_logger(), "static map callback called :)");
        if(!has_initial_map){ 
            initial_map = *msg;
            has_initial_map = true;
            RCLCPP_INFO(get_logger(), "Stored Intial Map!");
        }

        // latest_map = *msg;
        // has_latest_map = true;

    }

    void dynamicMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(get_logger(), "dynamic map callback called :)");
        latest_map = *msg;
        has_latest_map = true;
        RCLCPP_INFO(get_logger(), "Updated Dynamic Map!");
    }

    void addClusters(float px, float py){

        const float cluster_distance = 0.4;

        for (size_t i = 0; i < clusters.size(); i++) {
             HumanCluster &c = clusters[i];
            //distance between new point and centroid 
            float dx = px - c.cx();
            float dy = py - c.cy();
            //how far is this point from the middle 
            float dist = std::sqrt(dx*dx + dy*dy); //pythagorean theorum 

            //if dist is less than cluster disntace than it is part of our cluster 
            if (dist < cluster_distance) {
                //add point to cluster 
                c.x_sum += px;
                c.y_sum += py;
                //another point added means update point count (for accurate centroid calc)
                c.count += 1;
                return;
            }
         
        }
        
        //if our cluster is not reminiscint of any exisiting one, its prob a human 
        HumanCluster newC;
        newC.x_sum = px;
        newC.y_sum = py;
        newC.count =1;

        clusters.push_back(newC);

    }

    bool changeHappened(float wx, float wy){
        //lidar gives points in global coordinates, this need to be converted to grid valies
        int mx = static_cast<int>((wx - initial_map.info.origin.position.x) / initial_map.info.resolution);
        int my = static_cast<int>((wy - initial_map.info.origin.position.y) / initial_map.info.resolution);
        RCLCPP_INFO(get_logger(), "converting (%0.2f, %0.2f) into grid value: (%d, %d)", wx, wy, mx, my);

        //if my grid points are out of bounds 
        if (mx < 0 || my < 0 ||mx >= (int)initial_map.info.width ||my >= (int)initial_map.info.height) return false;

        //ros2 stores map 1-dimensionally within an array, need to adjust coordinates
        //calculating the index we're using to get info from that array 
        int index = my * initial_map.info.width + mx;

        //looking at original map, grid was free if its cost was zero
        bool free_before = (initial_map.data[index] == 0);
        //with current map, is this space still free? We are saying above 50 instead of 0 to avoid false positives
        bool now_occupied = (latest_map.data[index] > 50);
        // bool scanned = true; //if we made this far then we got a valud range from lidar 
        bool change = false;
        //need to compare prev grid and current grid to look for change 
        if(free_before != now_occupied) change = true;
        RCLCPP_INFO(get_logger(), "change in map: %d", change); //true prints 1 

        return change;
    }

    void publishDetectedHuman(){

        geometry_msgs::msg::PoseArray arr;
        arr.header.frame_id = "map";
        // arr.poses = detected_humans;

        for(size_t i = 0; i < clusters.size(); i++){
            RCLCPP_INFO(get_logger(), "Clustering!");
            HumanCluster &c = clusters[i];
            geometry_msgs::msg::Pose p;
            //record position of cluster of found humans and pusblish to nav
            p.position.x = c.cx();
            p.position.y = c.cy();
            p.orientation.w = 1.0;
            arr.poses.push_back(p);
        }

        //now can be accessed by nav node since we pusblished :)
        humans_publisher->publish(arr);

        // detected_humans.clear(); //need to look into how to stop scanning after this 
        RCLCPP_INFO(get_logger(), "Published moved human positions! Other nodes can access.");
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}

