// #include <chrono>
// #include <vector>

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>

// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <nav2_msgs/action/follow_path.hpp>
// #include <nav_msgs/msg/OccupancyGrid.hpp>

// // #include </home/shruthir/CSE180FinalProject/src/finalProject/src/nav.hpp>

// //this node is supposed to subscibe to perception node and publish to locomotion node 
// //this is the node that decides where to go 


// using namespace std::chrono_literals;

// class NavigationNode : public rclcpp::Node{
// public:
//   //using nav2 libary 
//   using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>; //gotopose
//   using LoadingMap = rclcpp_action::nav2_msgs::::OccupancyGrid;
//   // using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>; //followPath


//   NavigationNode() : Node("navigation_node"), current_wp_(0), goal_in_progress_(false){
//     //communicates with nav2 server and GoToPose is what we want to use 
//     nav_client_ = rclcpp_action::create_client<GoToPose>(this, "go_to_pose");

//     //subscribing to the array of destinations from perception node (because we used /scan)
//     humans_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
//       "/humans_moved", 10, std::bind(&NavigationNode::humansCallback, this, std::placeholders::_1));

//     //sets route thorugh map 
//     buildPatrolWaypoints();                                           

//     //publishing to /pose
//     pose_publisher = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",10);

//     //calls patrolStep every three seconds when the timer expires, "&NavigationNode::patrolStep" binds it to this class
//     //every three seconds we are deciding whether or not to move to the next waypoint 
//     patrol_timer_ = this->create_wall_timer(3s, std::bind(&NavigationNode::patrolStep, this));

//     //print statement, we made it this far in running - yay!
//     RCLCPP_INFO(get_logger(), "NavigationNode started. Waiting for Nav2 and Perception.");
//   }

// private:
//   //building target points we want to visit 
//   void buildPatrolWaypoints(){
//     //pointer of type msg 
//     geometry_msgs::msg::PoseStamped p1;
//     //specifies our frame 
//     p1.header.frame_id = "map";
//     //starting robot pose 
//     p1.pose.position.x = 2.12;   
//     p1.pose.position.y = -21.3;
//     p1.pose.orientation.w = 1.0;

//     //next movement
//     geometry_msgs::msg::PoseStamped p2 = p1;
//     p2.pose.position.x = 7.0;
//     p2.pose.position.y = -18.0;

//     //another movement
//     geometry_msgs::msg::PoseStamped p3 = p1;
//     p3.pose.position.x = -4.0;
//     p3.pose.position.y = -18.0;

//     //adding to a list ??
//     waypoints_.push_back(p1);
//     waypoints_.push_back(p2);
//     waypoints_.push_back(p3);
//   }
  
//   //function called with bind in patrolTimer 
//   void patrolStep(){
//     //need to change wait_for_action_server 
//     //waiting for nav2 to start 
//     if (!nav_client_->wait_for_action_server(500ms)) {
      
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
//                            "Waiting for Nav2 action server...");
//       return;
//     }
    
//     //if we're in the middle of a movement decision, we dont do more goals 
//     if (goal_in_progress_) {
//       return;
//     }
    
//     //sending to nav2
//     const auto &goal_pose = waypoints_[current_wp_];

//     //builds a nav2 goal message 
//     GoToPose::Goal goal;
//     goal.pose = goal_pose;

//     //dbg statement 
//     RCLCPP_INFO(get_logger(),
//                 "Patrol: sending goal %zu at (%.2f, %.2f)",
//                 current_wp_,
//                 goal_pose.pose.position.x,
//                 goal_pose.pose.position.y);

//     //
//     auto options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
//     options.result_callback = std::bind(&NavigationNode::navResultCallback, this, std::placeholders::_1);


//     goal_in_progress_ = true;
//     //non-blocking, other tasks can run simultanously 
//     nav_client_->async_send_goal(goal, options);

//     //incrementing the index of the goal pose we are looking at 
//     current_wp_ = (current_wp_ + 1) % waypoints_.size();
//   }

//   //debug statements for whether we were successful or not
//   void navResultCallback(const GoalHandleNav::WrappedResult &result){
//     goal_in_progress_ = false;
//     if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//       RCLCPP_INFO(get_logger(), "NavigationNode: goal succeeded.");
//     } else {
//       RCLCPP_WARN(get_logger(), "NavigationNode: goal failed or aborted.");
//     }
//   }

//   void humansCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
//     if (msg->poses.empty()) {
//       return;
//     }

//     if (!nav_client_->wait_for_action_server(500ms)) {
//       RCLCPP_WARN(get_logger(),
//                   "Nav2 action server not available; cannot go to human.");
//       return;
//     }

//     geometry_msgs::msg::PoseStamped target;
//     target.header.frame_id = "map";
//     target.header.stamp = this->now();
//     target.pose = msg->poses[0];

//     NavigateToPose::Goal goal;
//     goal.pose = target;

//     RCLCPP_WARN(get_logger(),
//                 "NavigationNode: received moved human pose -> navigating to (%.2f, %.2f)",
//                 target.pose.position.x,
//                 target.pose.position.y);

//     auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     options.result_callback = [this](const GoalHandleNav::WrappedResult &res){
//         if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
//           RCLCPP_INFO(get_logger(), "Reached moved human location.");
//         } else {
//           RCLCPP_WARN(get_logger(), "Failed to reach moved human location.");
//         }
//       };

//     goal_in_progress_ = true;
//     nav_client_->async_send_goal(goal, options);
//   }

//   rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr humans_sub_;
//   rclcpp::TimerBase::SharedPtr patrol_timer_;

//   std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
//   std::size_t current_wp_;
//   bool goal_in_progress_;

//   void callback(const nav_msgs::msg::OccupancyGrid sharedPtr){
//     //
//   }

// };

// int main(int argc, char **argv){
//   rclcpp::init(argc, argv);
//   //subscribing to map topic 
//   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
//   auto nodeh = rclcpp::Node::make_shared("loadingMap"); 
//   sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
//                                             ("/map",10, &callback);
//   rclcpp::spin(nodeh);
//   rclcpp::shutdown();
//   return 0;

// }

#include <chrono>
#include <vector>
//perception: find where the human is and point to that direction 
//navigation: 
/*

navigate to the initial human pos 
perception takes scan 
if theres no humans we move incrementally set waypoint 
how can we extract pose of all things on the map 
where does recording of final new human -> perception

*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;

class NavigationNode : public rclcpp::Node{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  double humanX[2] = {1.0, -12.0};
  double humanY[2] = {-1.0, 15.0};
  int humanCount = 0;
  bool humanFound;
  NavigationNode() : Node("navigation_node"), current_wp_(0), goal_in_progress_(false){

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    humans_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/humans_moved", 10,
      std::bind(&NavigationNode::humansCallback, this, std::placeholders::_1));

    buildPatrolWaypoints();

    patrol_timer_ = this->create_wall_timer(3s, std::bind(&NavigationNode::patrolStep, this));

    RCLCPP_INFO(get_logger(), "NavigationNode started. Waiting for Nav2 and Perception.");
  }

private:
  void buildPatrolWaypoints(){
    geometry_msgs::msg::PoseStamped p1;
    p1.header.frame_id = "map";
    p1.pose.position.x = 2.12;   // near start pose
    p1.pose.position.y = -21.3;
    p1.pose.orientation.w = 1.0;
    
    geometry_msgs::msg::PoseStamped p2 = p1;
    p2.pose.position.x = 7.0;
    p2.pose.position.y = -18.0;

    geometry_msgs::msg::PoseStamped p3 = p1;
    p3.pose.position.x = -4.0;
    p3.pose.position.y = -18.0;

    waypoints_.push_back(p1);
    waypoints_.push_back(p2);
    waypoints_.push_back(p3);
  }

  void patrolStep(){
    if(humanFound){
      humanCount++;
    }

    if(humanCount == 2){

      return;
    }

    if (!nav_client_->wait_for_action_server(500ms)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for Nav2 action server...");
      return;
    }

    if (goal_in_progress_) {
      return;
    }

    const auto &goal_pose = waypoints_[current_wp_];

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;

    RCLCPP_INFO(get_logger(),
                "Patrol: sending goal %zu at (%.2f, %.2f)",
                current_wp_,
                goal_pose.pose.position.x,
                goal_pose.pose.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback =
      std::bind(&NavigationNode::navResultCallback, this, std::placeholders::_1);

    goal_in_progress_ = true;
    nav_client_->async_send_goal(goal, options);

    current_wp_ = (current_wp_ + 1) % waypoints_.size();
  }

  void navResultCallback(const GoalHandleNav::WrappedResult &result){
    goal_in_progress_ = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "NavigationNode: goal succeeded.");
    } else {
      RCLCPP_WARN(get_logger(), "NavigationNode: goal failed or aborted.");
    }
  }

  void humansCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
    if (msg->poses.empty()) {
      return;
    }

    if (!nav_client_->wait_for_action_server(500ms)) {
      RCLCPP_WARN(get_logger(),
                  "Nav2 action server not available; cannot go to human.");
      return;
    }

    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "map";
    target.header.stamp = this->now();
    target.pose = msg->poses[0];

    NavigateToPose::Goal goal;
    goal.pose = target;

    RCLCPP_WARN(get_logger(),
                "NavigationNode: received moved human pose -> navigating to (%.2f, %.2f)",
                target.pose.position.x,
                target.pose.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback = [this](const GoalHandleNav::WrappedResult &res){
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Reached moved human location.");
        } else {
          RCLCPP_WARN(get_logger(), "Failed to reach moved human location.");
        }
      };

    goal_in_progress_ = true;
    nav_client_->async_send_goal(goal, options);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr humans_sub_;
  rclcpp::TimerBase::SharedPtr patrol_timer_;

  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::size_t current_wp_;
  bool goal_in_progress_;
};

void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  //
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  // auto nodeh = std::make_shared<NavigationNode>();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
  auto nodeh = rclcpp::Node::make_shared("loadingMap");
  sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",10, &callback);

  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}