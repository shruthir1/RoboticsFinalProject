#include <rclcpp/rclcpp.hpp> 
#include <nav.hpp>
#include <iostream>
//make an array of all x and y coordinates, use for loop for exery x all y's and for every y all x's in this for loop we call the waypoints function. 
//Use if statements if above or below max/min values, handle accordingly 
class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node"){
        // initialize Navigator inside constructor
    }

    int allX[30] = {-15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    int allY[63] = {-38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    int humanCount = 0;
    bool humanFound = false;
    int finalHuman1x = 0;
    int finalHuman1y = 0;
    int finalHuman2x = 0;
    int finalHuman2y = 0;

private:
   void initializePose(){
     // first: it is mandatory to initialize the pose of the robot
        geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
        init->position.x = 2.12;
        init->position.y = -21.3;
        init->orientation.w = 1.53;
        navigator.SetInitialPose(init);
    
   }

   void newWaypoints(int x, int y, int w){
    //use occupancy grid, if there is an obstacle here then we want to avoid -> obstacle here would mean grid cost is nonzero (i think) 
    //^ need to figure out how to do this
    //if(grid cost is nonzero) break; else: continue navigating towards
    //sping 360 at the start of every waypoint 
      navigator.Spin();
      navigator.Spin();
      navigator.Spin();
      navigator.Spin();
    //creating target waypoint
    if(humanFound){ 
        if(humanCount !=2){
            finalHuman1x = x;
            finalHuman1y = y;
            humanFound = false;
            RCLCPP_INFO(this->get_logger(), " Found Human 1 Around: (%d, %d)\n", finalHuman1x, finalHuman1y);
        }

        finalHuman2x = x;
        finalHuman2y = y;
        RCLCPP_INFO(this->get_logger(), "Found Human 2 Around: (%d, %d)\n", finalHuman2x, finalHuman2y);
    }
     
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

  //this function needs to go in spin 
  void callback(){
    if(!humanFound){
      for (int i = 0; i < 30, i++){
        for(int j = 0, j < 63, j++){
         newWaypoints(allX[i], allY[j]);
         if(humanFound) break;
        }
     }
    }
  }


};

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose
  
  //if we go to intentded position we want to spin 360 and then continue moving
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }


  while (!navigator.IsTaskComplete()) {
    return; 
  }
  //if we dont spin what we to be reoccuring then we need another "patrol-timer" like function 
  rclcpp::spin(); // here the spin statement needs to wait for whether human was found info from perception, if not found keep calling relevant functions 
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
