#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner_node") {}

  void init()
  {
  
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "panda_link0";
    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.w = 1.0;

    move_group_->setPoseTarget(target_pose);

    bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(this->get_logger(), "Move result: %s", success ? "SUCCESS" : "FAILURE");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerNode>();
  node->init();  // safe to use shared_from_this() now
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
