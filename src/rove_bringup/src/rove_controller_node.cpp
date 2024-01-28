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

enum ControllerMode{
    FLIPPER_MODE = 0,
    ARM_MODE = 1,
};


class RoveController : public rclcpp::Node {
public:
    RoveController() : Node("rove_controller_node") {
        // Subscribe to the joy topic
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&RoveController::joyCallback, this, std::placeholders::_1)
        );

        joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("/rove/joy", 10);

        previous_msg_ = sensor_msgs::msg::Joy();
        previous_msg_.axes.resize(6, 0.0);
        previous_msg_.buttons.resize(10, 0);
    }

private:
    std::unordered_map<int, std::unordered_map<size_t, std::function<void()>>> button_handlers_;
    int selected_mode = FLIPPER_MODE;

    void nextMode(){
        selected_mode = (selected_mode + 1) % 2; // Number of modes
        RCLCPP_INFO(get_logger(), "profil %i", selected_mode);
    }

    void common_action(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {

        if(buttton_up(joy_msg, B)){
            log("Common B up");
        }

        if(buttton_down(joy_msg, B)){
            log("Common B down");
        }

        if (buttton_down(joy_msg, Y)){
            nextMode();
        }
    }

    void arm_action(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        if(button_pressed(joy_msg, B)){
            log("Arm B pressed");
        }

        if(buttton_up(joy_msg, X)){
            log("Arm X up");
        }

        if(buttton_down(joy_msg, X)){
            log("Arm X down");
        }
    }

    void flipper_action(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        log("Flipper...");
        joy_pub_->publish(*joy_msg);
    }

    bool buttton_down(sensor_msgs::msg::Joy::SharedPtr curr_msg, int index){
        return previous_msg_.buttons[index] != curr_msg->buttons[index] && curr_msg->buttons[index] == 1;
    }
    bool buttton_up(sensor_msgs::msg::Joy::SharedPtr curr_msg, int index){
        return previous_msg_.buttons[index] != curr_msg->buttons[index] && curr_msg->buttons[index] == 0;
    }
    bool button_pressed(sensor_msgs::msg::Joy::SharedPtr curr_msg, int index){
        return previous_msg_.buttons[index] == curr_msg->buttons[index] && curr_msg->buttons[index] == 1;
    }

    void log(const char *text){
        RCLCPP_INFO(get_logger(), text);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        common_action(joy_msg);
        switch (selected_mode){
            case ARM_MODE:
                arm_action(joy_msg);
                break;
            case FLIPPER_MODE:
                flipper_action(joy_msg);
                break;
        }

        previous_msg_ = *joy_msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    sensor_msgs::msg::Joy previous_msg_;
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
