#include <rclcpp/rclcpp.hpp> 
#include <chrono>
//need to find the proper way to include his nav2 library -> need to look into this, makes stuff a bunch easier 
#include <finalProject/navigation.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

// manual test waypoints:
// ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {
//     header: { frame_id: 'map' },
//     pose: {
//         position: {x: 12.0, y: -12.0, z: -0.10},
//         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
//     }
// }}"

//need to make robot stop every 3 seconds along path to check for humans

// checking map bounds: 'ros2 run nav2_map_server map_saver_cli -t map'
// map meta data: 'cat map_1765412866.yaml'
// resolution: 0.03
// origin: [-15.100, -25.000, 0]
// size: 1006 × 1674 pixels

// X_min = -15.1
// X_max = -15.1 + 30.18 = 15.08
// Y_min = -25.0
// Y_max = -25.0 + 50.22 = 25.22


//need to add human detection functionality 
class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode(std::shared_ptr<Navigator> navigator) : Node("navigation_node"), navigator_(navigator){
        //publishes PoseStamped messages to Nav2 on the /goal_pose topic
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10); //queue size 10 means it stores up to 10 messages if system is slow
        // initialize Navigator inside constructor
        // initializePose();
        // loadWaypoints();

        //timer to call callback function every 5 seconds to send next waypoint
        timer_ = this->create_wall_timer(5s, std::bind(&NavigationNode::callback, this));

        current_waypoint_index_ = 0;
        waypoints_ = {
            {2.12, -23.3, 0.0},
            {14.0, -23.3, 0.0},
            {9.0, -21.0, 0.0},
            {14.0, -16.0, 0.0},
            {9.0, -16.0, 0.0}
            //havent added the rest for testing 

            // std::vector<std::pair<double,double>> coords = {
//             {2.12, -23.3},
//             {14, -23.3},
//             {9, -21},
//             {14, -16},
//             {9, -16}
//             // {12, -12},
//             // {12, -5},
//             // {12, 3},
//             // {6, 3},
//             // {6, 6},
//             // {13.5, 6},
//             // {13.5, 14},
//             // {0, 14},
//             // {0, 16.5},
//             // {10, 21},
//             // {-6, 24},
//             // {-9, 20},
//             // {-13, 24},
//             // {-13, 7},
//             // {-13, 6.3}
//             // unfinished
        };
        RCLCPP_INFO(this->get_logger(), "Navigation node started. Publishing to /goal_pose.");
    }

private:
    std::shared_ptr<Navigator> navigator_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_; // publisher for goal poses
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::array<double, 3>> waypoints_;
    size_t current_waypoint_index_;

//     void initializePose(){
//         // first: it is mandatory to initialize the pose of the robot
//         geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
//         // given in assignment
//         init->position.x = 2.12;
//         init->position.y = -21.3;
//         init->position.z = 0.00; 

//         double yaw = 1.57;   // 90 degrees, from assignment
//         init->orientation.x = 0.0;
//         init->orientation.y = 0.0;
//         init->orientation.z = sin(yaw / 2.0);  // ≈ 0.707
//         init->orientation.w = cos(yaw / 2.0);  // ≈ 0.707

//         navigator_->SetInitialPose(init);
//         RCLCPP_INFO(this->get_logger(), "Initial pose set to (%.2f, %.2f).", init->position.x, init->position.y);
//    }
   
//    void loadWaypoints(){
//     std::vector<std::pair<double,double>> coords = {
//             {2.12, -23.3},
//             {14, -23.3},
//             {9, -21},
//             {14, -16},
//             {9, -16}
//             // {12, -12},
//             // {12, -5},
//             // {12, 3},
//             // {6, 3},
//             // {6, 6},
//             // {13.5, 6},
//             // {13.5, 14},
//             // {0, 14},
//             // {0, 16.5},
//             // {10, 21},
//             // {-6, 24},
//             // {-9, 20},
//             // {-13, 24},
//             // {-13, 7},
//             // {-13, 6.3}
//             // unfinished
//         };

//         for(const auto &c : coords){
//             geometry_msgs::msg::Pose pose;
//             pose.position.x = c.first;
//             pose.position.y = c.second;
//             // pose.position.z = -0.10; //this is the default in gazebo??
//             // pose.orientation.w = 1.0; 
//             //store the created pose into waypoints vector
//             waypoints_.push_back(pose);
//             RCLCPP_INFO(this->get_logger(), "Loaded waypoint: (%.2f, %.2f, %.2f).", pose.position.x, pose.position.y, pose.position.z);
//         }

//         RCLCPP_INFO(this->get_logger(), "Loaded %d waypoints.", (int)waypoints_.size());
//     }

  //sends robot to next waypoint as PoseStamped, triggered every 5 seconds 
  void callback(){
        //no more waypoints
        if(current_waypoint_index_ >= waypoints_.size()){
            RCLCPP_INFO(this->get_logger(), "Finished all waypoints.");
            return;
        }

        //get current waypoint
        std::array<double, 3> &wp = waypoints_[current_waypoint_index_];

        geometry_msgs::msg::PoseStamped msg; //PoseStamped msg to publish
        msg.header.frame_id = "map"; //need to set map frame for Nav2 global navigation
        msg.header.stamp = this->now(); //timestamp to determine when pose was valid

        // position x, y, z
        msg.pose.position.x = wp[0];
        msg.pose.position.y = wp[1];
        msg.pose.position.z = 0.0; //as per gazebo default

        //convert yaw to quat
        tf2::Quaternion q;
        q.setRPY(0, 0, wp[2]); // roll=0, pitch=0, yaw=wp[2]
        q.normalize(); //normalize quaternion to avoid errors

        //assign quaternion to PoseStamped msg orientation
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        //publish goal pose
        goal_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published waypoint %ld: (%.2f, %.2f).", current_waypoint_index_, wp[0], wp[1]);

        current_waypoint_index_++;
    }

};

int main(int argc,char **argv) {
    rclcpp::init(argc,argv); // initialize ROS 
    auto navigator = std::make_shared<Navigator>(true, false); // create node with debug info but not verbose
    navigator->WaitUntilNav2Active();
    
    auto nodeh = std::make_shared<NavigationNode>(navigator);
    rclcpp::spin(nodeh);
    rclcpp::shutdown(); // shutdown ROS
    return 0;
}