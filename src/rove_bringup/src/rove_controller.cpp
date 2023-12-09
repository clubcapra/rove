// rove_controller.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

enum {
  JOY_BTN_A = 0,
  JOY_BTN_B = 1,
  JOY_BTN_X = 2,
  JOY_BTN_Y = 3,
  JOY_BTN_LB = 4,
  JOY_BTN_RB = 5,
};

class JoyListener : public rclcpp::Node {
public:
  JoyListener() : Node("joy_listener") {
    // Subscribe to the joy topic
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyListener::joyCallback, this, std::placeholders::_1)
    );
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    // Handle joystick input here
    // For example, print the values of the axes
    for (size_t i = 0; i < joy_msg->axes.size(); ++i) {
      RCLCPP_INFO(get_logger(), "Axis %zu: %f", i, joy_msg->axes[i]);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
