#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class LocomotionNode : public rclcpp::Node{
public:
  LocomotionNode(): Node("locomotion_node"){
    // If you configure Nav2 to output on /cmd_vel, we can just tap that here.
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&LocomotionNode::cmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LocomotionNode started (monitoring /cmd_vel).");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    // For now, just log. In a custom robot, you'd send these to wheel controllers.
    RCLCPP_INFO(get_logger(),
                "LocomotionNode: linear=%.2f, angular=%.2f",
                msg->linear.x, msg->angular.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocomotionNode>());
  rclcpp::shutdown();
  return 0;
}
