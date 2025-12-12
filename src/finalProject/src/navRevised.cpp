#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <finalProject/navigation.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <array>
#include <iostream>

using namespace std::chrono_literals;

// manual test waypoints:
// ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {
//     header: { frame_id: 'map' },
//     pose: {
//         position: {x: 12.0, y: -12.0, z: -0.10},
//         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
//     }
// }}"

// checking map bounds: 'ros2 run nav2_map_server map_saver_cli -t map'
// map meta data: 'cat map_1765412866.yaml'
// resolution: 0.03
// origin: [-15.100, -25.000, 0]
// size: 1006 × 1674 pixels

// X_min = -15.1
// X_max = -15.1 + 30.18 = 15.08
// Y_min = -25.0
// Y_max = -25.0 + 50.22 = 25.22


class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode(std::shared_ptr<Navigator> navigator) : Node("navigation_node"), navigator_(navigator){;

        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&NavigationNode::amclCallback, this, std::placeholders::_1));
        
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);


        //timer to call callback function every 500ms to send next waypoint
        timer_ = this->create_wall_timer(500ms, std::bind(&NavigationNode::callback, this));
        
        current_waypoint_phase_ = 0;
        current_waypoint_index_ = 0;
        waypoints_ = {
            {2.0, -1.0, 0.0},
            {14.0, -21.3, 0.0},
            {14.0, -16, 0.0},
            {12.0, -12.0, 0.0},
            {12.0, -5.0, 0.0},
            {12.0, 3.0, 0.0},
            {6.0, 3.0, 0.0},
            {6.0, 6.0, 0.0},
            {13.5, 6.0, 0.0},
            {13.5, 14.0, 0.0},
            {0, 14.0, 0.0},
            {0, 16.5, 0.0},
            {10.0, 21.0, 0.0},
            {-6, 24.0, 0.0},
            {-9.0, 20, 0.0},
            {13.0, 24.0, 0.0},
            {-13, 7.0, 0.0},
            {-13.0, 6.3, 0.0}

            //adding more
        };

        setInitialPose(2.12, -21.3, 1.57); //synchronize gazebo and rviz start positions

        RCLCPP_INFO(this->get_logger(), "Navigation node started.");
    }

private:
    std::shared_ptr<Navigator> navigator_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::array<double, 3>> waypoints_;
    size_t current_waypoint_index_;

    bool sent_goal_ = false;
    int current_waypoint_phase_ = 0; // 0 is Navigate, 1 is Spin
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;

    void setInitialPose(double x, double y, double yaw) {
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = this->now();
        initial_pose.header.frame_id = "map";

        initial_pose.pose.pose.position.x = x;
        initial_pose.pose.pose.position.y = y;
        initial_pose.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();

        initial_pose.pose.pose.orientation.x = q.x();
        initial_pose.pose.pose.orientation.y = q.y();
        initial_pose.pose.pose.orientation.z = q.z();
        initial_pose.pose.pose.orientation.w = q.w();

        //covariance just in case
        for (size_t i = 0; i < 36; i++) {
            initial_pose.pose.covariance[i] = 0.0;
        }

        initial_pose_pub_->publish(initial_pose);
        RCLCPP_INFO(this->get_logger(), "Published initial pose to AMCL: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    }

    
    //sends robot to next waypoint, triggered every 500 ms 
    void callback(){
        //no more waypoints
        if(current_waypoint_index_ >= waypoints_.size()){
            RCLCPP_INFO(this->get_logger(), "Finished all waypoints.");
            return;
        }

        if(current_waypoint_phase_ ==0){
            if(!sent_goal_){
                //get current waypoint
                std::array<double, 3> &wp = waypoints_[current_waypoint_index_];
                auto pose = std::make_shared<geometry_msgs::msg::Pose>();

                // position x, y, z
                pose->position.x = wp[0];
                pose->position.y = wp[1];
                pose->position.z = 0.0; //as per gazebo default

                //convert yaw to quat
                tf2::Quaternion q;
                q.setRPY(0, 0, wp[2]); // roll=0, pitch=0, yaw=wp[2]
                q.normalize(); //normalize quaternion to avoid errors

                //assign quaternion
                pose->orientation.x = q.x();
                pose->orientation.y = q.y();
                pose->orientation.z = q.z();
                pose->orientation.w = q.w();

                RCLCPP_INFO(this->get_logger(), "Sending waypoint %ld: (%.2f, %.2f).", current_waypoint_index_ + 1, wp[0], wp[1]);
                navigator_->GoToPose(pose);

                sent_goal_ = true;
            }
            else if (navigator_->IsTaskComplete()){
                RCLCPP_INFO(this->get_logger(), "PHASE 0: Navigation to waypoint %ld complete.", 
                            current_waypoint_index_ + 1);
                //switch to the spin phase
                sent_goal_ = false;
                current_waypoint_phase_ = 1;
            }
            return; 
        }

        if(current_waypoint_phase_ == 1){
            if(!sent_goal_){
                double full_rotation_radians = 6.28; //360 degrees
                RCLCPP_INFO(this->get_logger(), "PHASE 1: Starting rotation at waypoint %ld (%.2f radians).", 
                            current_waypoint_index_ + 1, full_rotation_radians);
                if(navigator_->Spin(full_rotation_radians)){
                    sent_goal_ = true;
                }
                else{
                    //debug: move to next waypoint even if spin fails
                    RCLCPP_ERROR(this->get_logger(), "PHASE 1: Spin request rejected.");
                }
            }
            else if (navigator_->IsTaskComplete()){
                RCLCPP_INFO(this->get_logger(), "PHASE 1: Rotation at waypoint %ld complete.", 
                            current_waypoint_index_ + 1);

                //print final amcl pose after movement and rotation
                if (last_amcl_pose_) {
                    auto p = last_amcl_pose_->pose.pose;
                    RCLCPP_INFO(this->get_logger(), "AMCL pose after rotation: x=%.2f, y=%.2f, z=%.2f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                                p.position.x, p.position.y, p.position.z,
                                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                }
                //go to the next waypoint
                current_waypoint_index_++; 
                sent_goal_ = false;
                current_waypoint_phase_ = 0; // reset phase for next waypoint
            }
        }
        return;
    }

    //store latest amcl pose
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        last_amcl_pose_ = msg;
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
