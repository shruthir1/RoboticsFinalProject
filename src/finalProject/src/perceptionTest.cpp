#include <finalProject/navigation.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class HumanFinderNode : public rclcpp::Node {
public:
    HumanFinderNode() : Node("human_finder") {
        // Subscribe to perception results
        human_sub = create_subscription<geometry_msgs::msg::PoseArray>(
            "/humans_moved", 10, 
            std::bind(&HumanFinderNode::humanCallback, this, std::placeholders::_1));
        
        // Create navigator
        navigator = std::make_shared<Navigator>(true, false);
        
        // Set initial pose
        auto initial_pose = std::make_shared<geometry_msgs::msg::Pose>();
        initial_pose->position.x = 2.12;
        initial_pose->position.y = -21.3;
        initial_pose->orientation.z = sin(1.57/2);
        initial_pose->orientation.w = cos(1.57/2);
        navigator->SetInitialPose(initial_pose);
        navigator->WaitUntilNav2Active();
        
        // Start exploration
        exploreWarehouse();
    }
    
private:
    std::shared_ptr<Navigator> navigator;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr human_sub;
    std::vector<geometry_msgs::msg::Pose> detected_humans;
    
    void humanCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        for(const auto& pose : msg->poses) {
            // Check if this is a new detection
            bool is_new = true;
            for(const auto& existing : detected_humans) {
                float dx = pose.position.x - existing.position.x;
                float dy = pose.position.y - existing.position.y;
                if(sqrt(dx*dx + dy*dy) < 1.0) { // Within 1m
                    is_new = false;
                    break;
                }
            }
            if(is_new) {
                detected_humans.push_back(pose);
                RCLCPP_INFO(get_logger(), "New human found at (%.2f, %.2f)", 
                           pose.position.x, pose.position.y);
            }
        }
    }
    
    void exploreWarehouse() {
        // Define waypoints to cover the warehouse
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        
        // Add strategic positions to scan known human locations
        // Original human locations: (1, -1) and (-12, 15)
        addWaypoint(waypoints, 1.0, -1.0, 0.0);
        addWaypoint(waypoints, -12.0, 15.0, 0.0);
        // Add more waypoints to cover the warehouse...
        
        navigator->FollowWaypoints(waypoints);
        
        while(!navigator->IsTaskComplete()) {
            rclcpp::spin_some(this->get_node_base_interface());
        }
        
        reportResults();
    }
    
    void addWaypoint(std::vector<geometry_msgs::msg::PoseStamped>& waypoints, 
                     double x, double y, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.z = sin(yaw/2);
        pose.pose.orientation.w = cos(yaw/2);
        waypoints.push_back(pose);
    }
    
    void reportResults() {
        RCLCPP_INFO(get_logger(), "=== FINAL RESULTS ===");
        RCLCPP_INFO(get_logger(), "Found %zu humans:", detected_humans.size());
        for(size_t i = 0; i < detected_humans.size(); i++) {
            RCLCPP_INFO(get_logger(), "Human %zu: (%.2f, %.2f)", 
                       i+1, detected_humans[i].position.x, detected_humans[i].position.y);
        }
    }
};
