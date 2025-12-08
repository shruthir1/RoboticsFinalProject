#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node"){
        // initialize Navigator inside constructor
    }

private:
    // waypoints, timers, callback functions go here
    Navigator navigator_;
};

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose
  
  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = 2.12;
  init->position.y = -21.3;
  init->orientation.w = 1.53;
  navigator.SetInitialPose(init);
  // spin in place of 90 degrees (default parameter)
  //if we go to intentded position we want to spin 360 and then continue moving
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }

  void newWaypoints(int x, int y, int w){
    //sping 360 at the start of every waypoint 
      navigator.Spin();
      navigator.Spin();
      navigator.Spin();
      navigator.Spin();
    //creating target waypoint
    //max Y coorindates are -38 to 24
    //max X coordinates are 

    //spin at original human positions and around these coordinates: 
    
    geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
    if(/*if reached a maximum/minimum X or Y*/){
         goal_pos->position.x = x;
         goal_pos->position.y = y;
         goal_pos->orientation.w = w;
    }else{
         goal_pos->position.x = x;
         goal_pos->position.y = y;
         goal_pos->orientation.w = w;
    }

    //moving to a new pose
    navigator.GoToPose(goal_pos);
  }

  while (!navigator.IsTaskComplete()) {
    return; 
  }

  //go to new pose again 
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose

  while ( ! navigator.IsTaskComplete() ) {
    //busy doing another task
    return;
  }

  // complete here....
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}