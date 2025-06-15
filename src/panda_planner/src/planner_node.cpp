#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <mutex>


class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner_node"), is_moving_(false)
    {
      rclcpp::on_shutdown([this]() {
        if (move_group_) {
          move_group_->stop();
          move_group_->clearPoseTargets();
          RCLCPP_INFO(this->get_logger(), "Shutdown: stopped and cleared goals.");
        }
      });
    }

  void init()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");

    // Subscribe to /target_pose
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10,
      std::bind(&PlannerNode::targetPoseCallback, this, std::placeholders::_1));
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  std::atomic_bool is_moving_;  // Thread-safe flag

  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (is_moving_)
    {
      RCLCPP_WARN(this->get_logger(), "Robot is busy, ignoring new target.");
      return;
    }

    is_moving_ = true;

    RCLCPP_INFO(this->get_logger(), "Received target pose: [x: %.2f, y: %.2f, z: %.2f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    move_group_->setPoseTarget(*msg);

    bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(this->get_logger(), "Move result: %s", success ? "SUCCESS" : "FAILURE");

    is_moving_ = false;
  }
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
