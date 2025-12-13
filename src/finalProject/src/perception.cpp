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
#include <thread> 

// Implementing clustering can help with reducing noise (hopefully) 
struct HumanCluster {
    // x and y for calculating the center points of the cluster its the corrdinates of a point  
    float x_sum = 0.0;
    float y_sum = 0.0;
    int count = 0; //how many points are within a cluster 

    float cx() const { 
        //center point for x 
        return x_sum / count; 
    }
    float cy() const { 
        //center point for y 
        return y_sum / count;
     }
};
 
class PerceptionNode : public rclcpp::Node{
public:
  PerceptionNode() : Node("perception_node") {
    
    // had to do QoS becayse static map and pose were not subscirbing properly, using QoS fixed that 
    rclcpp::QoS map_qos(1); 
    map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL); // Static Map Fix

    rclcpp::QoS continuous_qos(10);
    continuous_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // Pose Fix

    
    
    //getting the unchanging map from /map topic, every time this topic updates we call staticMapCallback()
    static_map_subscriber = create_subscription<nav_msgs::msg::OccupancyGrid>( "/map", map_qos, std::bind(&PerceptionNode::staticMapCallback, this, std::placeholders::_1));

    //get the robots pose everytime it updates from /amcl subscription by calling amclCallback
    pose_subscriber = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", continuous_qos, std::bind(&PerceptionNode::amclCallback, this, std::placeholders::_1));

    //get the updated local cost map from /global_costmap/costmap and when the human moves or the topic is updated we get a updated map from callback function
    dynamic_map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>( "/global_costmap/costmap", 10, std::bind(&PerceptionNode::dynamicMapCallback, this, std::placeholders::_1));

    //every time we get more info from /scan we'll call scanCallback() which 
    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PerceptionNode::scanCallback, this, std::placeholders::_1));
   
    //publishes so nav node can access 
    humans_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/humans_moved", 10);

    RCLCPP_INFO(this->get_logger(), "PerceptionNode is starting!!");
    
    //wait until we have initial robot pose to start (so we know where we are)
    this->waitForInitialPose();
  }

private: 
    //variables that will store values revelanvt to each callback function 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr humans_publisher;
    bool has_initial_map = false;
    bool has_latest_map = false;
    bool has_pose = false;
    bool is_initialized_ = false; 
    int humanCount = 0;
    nav_msgs::msg::OccupancyGrid initial_map;
    nav_msgs::msg::OccupancyGrid latest_map;
    geometry_msgs::msg::Pose robot_pose;
    std::vector<geometry_msgs::msg::Pose> detected_humans; //human pose storage 
    std::vector<HumanCluster> clusters; // Cluster storage var

    //helper functions:

    //before the program start 
    void waitForInitialPose() {
        RCLCPP_INFO(this->get_logger(), "Waiting for single initial robot pose from AMCL...");
        using namespace std::chrono_literals;

        while (!has_pose) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(10ms); 
        }
        RCLCPP_INFO(this->get_logger(), "Initial pose received! Robot is localized.");
    }
    
    //callback functiosns:
    
    //every time the amcl topic updates we want to update what we know about the robots current pose 
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        robot_pose = msg->pose.pose;
        RCLCPP_FATAL(this->get_logger(), "current robot pose (%.2f,%.2f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
        has_pose = true;
        // RCLCPP_WARN(this->get_logger(),  "Have current pose");
    }
    
    // /map only updates once and we want to store the inital grid in a pointer so that we can reference it later 
    void staticMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "static map callback called :)");
        if(!has_initial_map){ 
            //if we dont have the initial map then store that laoded map here
            initial_map = *msg;
            has_initial_map = true;
            RCLCPP_INFO(this->get_logger(), "Stored Intial Map!");
        }
    }

    //every time /global_cost_map is updated we store it here
    //were going to use local_cost_map but were unable to understand how to do the calculations to compare to inital map 
    void dynamicMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "dynamic map callback called :)");
        //store map in pointer!
        latest_map = *msg;
        has_latest_map = true;
        RCLCPP_INFO(this->get_logger(), "Updated Dynamic Map!");
    }

    //every time we do a scan we 
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        // are we intialized, need costmap, intial map, and pose of robot 
        if(!has_latest_map || !has_initial_map || !has_pose){
            if (!is_initialized_) {
                RCLCPP_WARN(this->get_logger(),  "Not ready: static map: %d, pose: %d, changing map: %d", has_initial_map, has_pose, has_latest_map); 
            }
            return; 
        }
        
        //
        if (!is_initialized_) {
            RCLCPP_INFO(this->get_logger(), "Perception node is fully initialized and starting Lidar processing.");
            is_initialized_ = true; 
        }

        // at every scan we want to analyze new clusters -- we dont want old clusters to mess with new ones 
        clusters.clear();

        //robots local pose from amcl
        float rx = robot_pose.position.x; 
        float ry = robot_pose.position.y; 
        float yaw = tf2::getYaw(robot_pose.orientation); 
        //need to convert from polar coordinares to cartesian
        //these are the angles of the lidar rays from the robot 
        float r_angle_rframe;
        float r_angle_wframe;
        //to hold the world frame of the laser cartesian coordinates 
        float wx;
        float wy;

        //for every laser beam we've done 
        for(size_t i =0; i < msg->ranges.size(); i++){
            float range = msg->ranges[i]; //distance of laser beam
            //range is the distance produced by laser, if its lower than than min then its probably invalid and if its greater than five then it may be to far(?)
            if (range < msg->range_min || range >= 5) continue;
            
            // need angles of laser to be able to compute cartesian coordinate of laser
            r_angle_rframe = msg->angle_min + i * msg->angle_increment;
            r_angle_wframe = r_angle_rframe + yaw - 1.5708; // Yaw + fixed offset

            //standard polar -> cartesian conversion :)
            wx = rx + range * cos(r_angle_wframe); 
            wy = ry + range * sin(r_angle_wframe); 
            
            //if we see a change in the current cost map (should've been local cost map not sure if global updates properly) compared to inital grid 
            //basically within a few grids if we see an obstacle that we didnt see during the intial map traversal then change happened
            if(this->changeHappened(wx, wy)){
                RCLCPP_WARN(this->get_logger(),  "Change in map at (%.2f, %.2f)", wx, wy);  
                //if we see a change then we want to see if the clusters allign with what we feel would identify a human 
                //so we add these clusters to be analyzed
                this->addClusters(wx, wy);
            }
        }
        
        //if we recorded some clusters (!clusters.empty())
       if (!clusters.empty()) {
            // this->publishDetectedHuman();
            std::vector<HumanCluster> human_clusters;
            for(size_t i = 0; i < clusters.size(); i++){
                HumanCluster &c = clusters[i];
                if(c.count >= 10){ 
                    //add these clusters to our human cluster array 
                    human_clusters.push_back(c);
                }
            }
            //if we have info in our human clusters array
            //i think we need more logic here analyzing what makes these clusters a human as opposed to anything else 
            if(!human_clusters.empty()){
                //we may have deteced a human 
                this->publishDetectedHuman();
                RCLCPP_WARN(this->get_logger(),  "Detected possible human position at (%.2f, %.2f)", wx, wy);
                humanCount++;
            } 
        }
    }

    void addClusters(float px, float py){
        //maximum distance from out position to a point to be considered part of our cluster 
        const float cluster_distance = 0.6;
        for (size_t i = 0; i < clusters.size(); i++) {
             HumanCluster &c = clusters[i];
            //distance between new point to cluster's centroid 
            float dx = px - c.cx();
            float dy = py - c.cy();
            float dist = std::sqrt(dx*dx + dy*dy); 
            //if the we are a part of this cluster then our computed distance should be less than 0.6
            if (dist < cluster_distance) {
                c.x_sum += px; //update values for center calc 
                c.y_sum += py; 
                c.count += 1;
                return;
            }
        }
        
        // If point doesn't belong to any cluster, create a new one
        HumanCluster newC;
        newC.x_sum = px;
        newC.y_sum = py;
        newC.count =1;

        clusters.push_back(newC);
    }

    bool changeHappened(float wx, float wy){
        //mx and my are lidar coordinates that we want to convert to global map coordinates
        int mx = static_cast<int>((wx - initial_map.info.origin.position.x) / initial_map.info.resolution);
        int my = static_cast<int>((wy - initial_map.info.origin.position.y) / initial_map.info.resolution);
        RCLCPP_INFO(this->get_logger(), "converting (%0.2f, %0.2f) into grid value: (%d, %d)", wx, wy, mx, my);
        //if grid points are out of bounds, return false
        if (mx < 0 || my < 0 ||mx >= (int)initial_map.info.width ||my >= (int)initial_map.info.height) return false;

        int index = my * initial_map.info.width + mx; //this could be wrong 
        //we want to compare the cost values at this index in both maps
        int initial_cost = initial_map.data[index];
        int latest_cost = latest_map.data[index];

        //if latest cost is unknown, return false
        if (latest_cost < 0) return false;

        // new obstacle if the cost is between to and 50 but not -1 
        bool new_obstacle = (initial_cost >= 0 && initial_cost <= 10) && (latest_cost > 200);

        // Case 2: Obstacle Disappeared (Moved Away) - Spot was occupied (> 50), is now free (0)
        // This is needed if the human was originally mapped as a static obstacle.
        bool obstacle_gone = (initial_cost > 50) && (latest_cost == 0);

        //there is a change if we found a new obstacle or an obstacle is gone
        bool change = new_obstacle || obstacle_gone;
        
        RCLCPP_INFO(this->get_logger(), "Change check: Initial cost: %d, Latest cost: %d. Change: %d", 
                     initial_cost, latest_cost, change); 

        return change;
    }

    //should return true if we approach a wall 
    bool iswall(float wx, float wy){
        //paramters are global coordinates and we want to convert to local coordinates
        int mx = (wx - initial_map.info.origin.position.x) / initial_map.info.resolution;
        int my = (wy - initial_map.info.origin.position.y) / initial_map.info.resolution;
        

    }

    void publishDetectedHuman(){
        geometry_msgs::msg::PoseArray arr;
        arr.header.frame_id = "map";

        for(size_t i = 0; i < clusters.size(); i++){
            RCLCPP_INFO(this->get_logger(), "Clustering!");
            HumanCluster &c = clusters[i];
            geometry_msgs::msg::Pose p;
            p.position.x = c.cx(); //updating centroid average x 
            p.position.y = c.cy(); //updating centroid average y 
            p.orientation.w = 1.0;
            arr.poses.push_back(p);
        }

        humans_publisher->publish(arr);
        RCLCPP_INFO(this->get_logger(), "Published moved human positions! Other nodes can access.");
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}