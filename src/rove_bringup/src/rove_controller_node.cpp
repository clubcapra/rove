// rove_controller.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"

using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

enum ControllerMode
{
  TRACKS_MODE = 0,
  ARM_MODE = 1,
  FLIPPER_MODE = 2,
};

class RoveController : public rclcpp::Node
{
public:
  RoveController() : Node("rove_controller_node")
  {
    // Buttons
    A = declare_parameter("A", 0);
    B = declare_parameter("B", 1);
    X = declare_parameter("X", 2);
    Y = declare_parameter("Y", 3);
    LB = declare_parameter("LB", 4);
    RB = declare_parameter("RB", 5);
    VIEW = declare_parameter("VIEW", 6);
    MENU = declare_parameter("MENU", 7);
    XBOX = declare_parameter("XBOX", 8);
    LS = declare_parameter("LS", 9);
    RS = declare_parameter("RS", 10);
    SHARE = declare_parameter("SHARE", 11);

    // Axes
    LS_X = declare_parameter("LS_X", 0);
    LS_Y = declare_parameter("LS_Y", 1);
    LT = declare_parameter("LT", 2);
    RS_X = declare_parameter("RS_X", 3);
    RS_Y = declare_parameter("RS_Y", 4);
    RT = declare_parameter("RT", 5);
    D_PAD_X = declare_parameter("D_PAD_X", 6);
    D_PAD_Y = declare_parameter("D_PAD_Y", 7);

    // Subscribe to the joy topic
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&RoveController::joyCallback, this, std::placeholders::_1));

    joy_pub_rove_ = create_publisher<sensor_msgs::msg::Joy>("/rove/joy", 1);
    joy_pub_ovis_ = create_publisher<sensor_msgs::msg::Joy>("/ovis/joy", 1);

    previous_msg_ = sensor_msgs::msg::Joy();
    previous_msg_.axes.resize(6, 0.0);
    previous_msg_.buttons.resize(10, 0);

    teleop_msg_ = sensor_msgs::msg::Joy();
    teleop_msg_.axes.resize(2, 0);
    teleop_msg_.buttons.resize(1, 0);

    gripper_action_client_ =
        rclcpp_action::create_client<GripperCommand>(this, "/robotiq_gripper_controller/gripper_cmd");

    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
    }
  }

private:
  int A, B, X, Y, LB, RB, VIEW, MENU, XBOX, LS, RS, SHARE;
  int LS_X, LS_Y, LT, RS_X, RS_Y, RT, D_PAD_X, D_PAD_Y;

  int selected_mode = TRACKS_MODE;
  bool publish_to_rove = true;  // Toggle between rove and ovis

  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  bool gripper_opened_ = true;

  void nextMode()
  {
    selected_mode = (selected_mode + 1) % 3;  // Now cycles through 3 modes

    // Reset previous message state when switching modes
    previous_msg_ = sensor_msgs::msg::Joy();
    previous_msg_.axes.resize(6, 0.0);
    previous_msg_.buttons.resize(10, 0);

    std::string mode_name;
    switch (selected_mode)
    {
      case TRACKS_MODE:
        mode_name = "TRACKS";
        break;
      case ARM_MODE:
        mode_name = "ARM";
        break;
      case FLIPPER_MODE:
        mode_name = "FLIPPER";
        break;
    }
    RCLCPP_INFO(get_logger(), "Current mode: %s", mode_name.c_str());
  }

  void common_controls(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    // Mode switching with MENU button
    if (button_down(joy_msg, MENU))
    {
      nextMode();
    }

    // Gripper control with SHARE button
    if (button_down(joy_msg, SHARE))
    {
      gripper_opened_ = !gripper_opened_;
      sendGripperGoal(gripper_opened_ ? 0.0 : 1.0, 10.0);
    }
  }

  void tracks_mode(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    teleop_msg_.axes[0] = joy_msg->axes[LS_X];
    teleop_msg_.axes[1] = joy_msg->axes[LS_Y];
    teleop_msg_.buttons[0] = joy_msg->buttons[A];
    joy_pub_rove_->publish(teleop_msg_);
  }

  void arm_mode(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    // Create a modified joy message with inverted axes
    sensor_msgs::msg::Joy modified_joy = *joy_msg;
    
    // Invert the relevant axes (Y axes are typically the ones that need inversion)
    modified_joy.axes[LS_Y] = -joy_msg->axes[LS_Y];
    modified_joy.axes[RS_Y] = -joy_msg->axes[RS_Y];
    
    joy_pub_ovis_->publish(modified_joy);
  }

  void flipper_mode(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Flipper mode not implemented yet");
  }

  bool button_down(const sensor_msgs::msg::Joy::SharedPtr curr_msg, int index)
  {
    return previous_msg_.buttons[index] != curr_msg->buttons[index] && curr_msg->buttons[index] == 1;
  }

  bool button_up(const sensor_msgs::msg::Joy::SharedPtr curr_msg, int index)
  {
    return previous_msg_.buttons[index] != curr_msg->buttons[index] && curr_msg->buttons[index] == 0;
  }

  bool button_pressed(const sensor_msgs::msg::Joy::SharedPtr curr_msg, int index)
  {
    return curr_msg->buttons[index] == 1;
  }

  void sendGripperGoal(double position, double max_effort)
  {
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = max_effort;
    RCLCPP_INFO(get_logger(), "Sending gripper command: pos=%.2f, effort=%.2f", position, max_effort);
    gripper_action_client_->async_send_goal(goal_msg);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    common_controls(joy_msg);

    switch (selected_mode)
    {
      case TRACKS_MODE:
        tracks_mode(joy_msg);
        break;
      case ARM_MODE:
        arm_mode(joy_msg);
        break;
      case FLIPPER_MODE:
        flipper_mode(joy_msg);
        break;
    }

    previous_msg_ = *joy_msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_rove_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_ovis_;
  sensor_msgs::msg::Joy previous_msg_;
  sensor_msgs::msg::Joy teleop_msg_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoveController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
