// rove_controller.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

enum JoyBtn{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  VIEW = 4,
  XBOX = 5,
  MENU = 6,
  LS = 7,
  RS = 8,
  LB = 9,
  RB = 10,
  D_UP = 11,
  D_DOWN = 12,
  D_LEFT = 13,
  D_RIGHT = 14,
//   // Paddles
//   P_ = 15,
//   P_ = 16,
//   P_ = 17,
//   P_ = 18,
//   P_ = 19,
//   P_ = 20,
//   P_ = 21,
};

enum JoyStick{
    LS_X = 0,
    LS_Y = 1,
    RS_X = 2,
    RS_Y = 3,
    LT = 4,
    RT = 5,
};

class RoveController : public rclcpp::Node {
public:
  RoveController() : Node("rove_controller") {
    // Subscribe to the joy topic
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&RoveController::joyCallback, this, std::placeholders::_1)
    );
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    // Handle joystick input here
    // For example, print the values of the axes
    for (size_t i = 0; i < joy_msg->axes.size(); ++i) {
        if(i == LS_X){
            RCLCPP_INFO(get_logger(), "Axis %zu: %f", i, joy_msg->axes[i]);
        }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoveController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
