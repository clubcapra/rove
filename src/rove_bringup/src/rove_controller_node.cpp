#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;
using Trigger = std_srvs::srv::Trigger;

class RoveController : public rclcpp::Node
{
public:
  RoveController() : Node("rove_controller_node")
  {
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
        this, "/robotiq_gripper_controller/gripper_cmd");

    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(get_logger(), "Gripper action server not available.");
    }

    open_srv_ = create_service<Trigger>(
        "open_gripper", std::bind(&RoveController::openGripper, this, std::placeholders::_1, std::placeholders::_2));

    close_srv_ = create_service<Trigger>(
        "close_gripper", std::bind(&RoveController::closeGripper, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  rclcpp::Service<Trigger>::SharedPtr open_srv_;
  rclcpp::Service<Trigger>::SharedPtr close_srv_;

  void openGripper(const std::shared_ptr<Trigger::Request> request,
                   std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    RCLCPP_INFO(get_logger(), "Service call: Open gripper");
    sendGripperGoal(0.0, 10.0);
    response->success = true;
    response->message = "Gripper opened";
  }

  void closeGripper(const std::shared_ptr<Trigger::Request> request,
                    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;
    RCLCPP_INFO(get_logger(), "Service call: Close gripper");
    sendGripperGoal(1.0, 10.0);
    response->success = true;
    response->message = "Gripper closed";
  }

  void sendGripperGoal(double position, double max_effort)
  {
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = max_effort;
    gripper_action_client_->async_send_goal(goal_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoveController>());
  rclcpp::shutdown();
  return 0;
}
