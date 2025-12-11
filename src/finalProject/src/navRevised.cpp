#include <rclcpp/rclcpp.hpp> 
#include <chrono>
//need to find the proper way to include his nav2 library -> need to look into this, makes stuff a bunch easier 
#include <finalProject/navigation.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <iostream>

using namespace std::chrono_literals;



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
        // initialize Navigator inside constructor
        initializePose();
        loadWaypoints();

        timer_ = this->create_wall_timer(200ms, std::bind(&NavigationNode::callback, this)
    );
    }

private:
    std::shared_ptr<Navigator> navigator_;
    rclcpp::TimerBase::SharedPtr timer_;

    // vector to hold waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints_;
    int current_waypoint_index_ = 0;
   
   void loadWaypoints(){
    std::vector<std::pair<double,double>> coords = {
            {2.12, 14},
            {14, -16},
            {12, -12},
            {12, -5},
            {12, 3},
            {6, 3},
            {6, 6},
            {13.5, 6},
            {13.5, 14},
            {0, 14},
            {0, 16.5},
            {10, 21},
            {-6, 24},
            {-9, 20},
            {-13, 24},
            {-13, 7},
            {-13, 6.3}
            // unfinished
        };

        for(std::pair<double,double> &c : coords){
            geometry_msgs::msg::Pose pose;
            pose.position.x = c.first;
            pose.position.y = c.second;
            pose.position.z = -0.10; //this is the default in gazebo??
            pose.orientation.w = 1.0; 
            //store the created pose into waypoints vector
            waypoints_.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %d waypoints.", (int)waypoints_.size());
    }

    void initializePose(){
        // first: it is mandatory to initialize the pose of the robot
        geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
        init->position.x = 2.12;
        init->position.y = -21.3;
        init->position.z = -0.10; //double check in gazebo
        init->orientation.w = 1.53;
        navigator_->SetInitialPose(init);
        RCLCPP_INFO(this->get_logger(), "Initial pose set to (%.2f, %.2f).", init->position.x, init->position.y);
   }

  //this function checks if the last action is complete, spins once at each waypoint, then sends robot to next waypoint 
  void callback(){
    // if(!humanFound){
        //no more waypoints
        if((int)current_waypoint_index_ >= (int)waypoints_.size()){
            RCLCPP_INFO(this->get_logger(), "Finished all waypoints.");
            return;
        }

        while ( ! navigator_->IsTaskComplete() ) {
            // busy waiting for task to be completed
        }

        if (current_waypoint_index_ > 0){
            RCLCPP_INFO(this->get_logger(), "Arrived at waypoint %d -> spinning...", current_waypoint_index_ - 1);
            //spin 90 degrees four times at each waypoint to do a full rotation
            navigator_->Spin();
            navigator_->Spin();
            navigator_->Spin();
            navigator_->Spin();
            return;
        }
        while ( ! navigator_->IsTaskComplete() ) {
            // busy waiting for task to be completed
        }

        //for first waypoint or after spinning:
        std::shared_ptr<geometry_msgs::msg::Pose> next = std::make_shared<geometry_msgs::msg::Pose>(waypoints_[current_waypoint_index_]);
        RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %d at (%.2f, %.2f)", current_waypoint_index_, next->position.x, next->position.y);
        navigator_->GoToPose(next);
        while ( ! navigator_->IsTaskComplete() ) {
            // busy waiting for task to be completed
        }
        current_waypoint_index_++;
    }
//   }


};

int main(int argc,char **argv) {
    rclcpp::init(argc,argv); // initialize ROS 
    auto navigator = std::make_shared<Navigator>(true, false); // create node with debug info but not verbose
    navigator->WaitUntilNav2Active();
    
    auto nodeh = std::make_shared<NavigationNode>(navigator);
    //if we dont spin what we to be reoccuring then we need another "patrol-timer" like function 
    rclcpp::spin(nodeh); // here the spin statement needs to wait for whether human was found info from perception, if not found keep calling relevant functions 
    rclcpp::shutdown(); // shutdown ROS
    return 0;
}