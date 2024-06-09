// rove_controller.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum ControllerMode{
    FLIPPER_MODE = 0,
    ARM_MODE = 1,
};


class RoveController : public rclcpp::Node {
public:
    RoveController() : Node("rove_controller_node") {
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
            "/joy", 10, std::bind(&RoveController::joyCallback, this, std::placeholders::_1)
        );
        // cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&RoveController::cmdvelCallback, this, std::placeholders::_1));

        joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("/rove/joy", 1);

        // auto x = create_wall_timer(
        //     std::chrono::milliseconds(1000/20), std::bind(&RoveController::cmdvelTimerCallback, this, std::placeholders::_1)
        // );

        previous_msg_ = sensor_msgs::msg::Joy();
        previous_msg_.axes.resize(6, 0.0);
        previous_msg_.buttons.resize(10, 0);

        teleop_msg_ = sensor_msgs::msg::Joy();
        teleop_msg_.axes.resize(2,0);
        teleop_msg_.buttons.resize(1,0);

        // cmd_vel_msg_ = geometry_msgs::msg::Twist();
        // cmd_vel_stamped_msg_ = geometry_msgs::msg::TwistStamped();
    }

private:
    int A, B, X, Y, LB, RB, VIEW, MENU, XBOX, LS, RS, SHARE, LS_X, LS_Y, LT, RS_X, RS_Y, RT, D_PAD_X, D_PAD_Y;

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
        log("Arm");
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
        teleop_msg_.axes[0] = joy_msg->axes[LS_X];
        teleop_msg_.axes[1] = joy_msg->axes[LS_Y];
        teleop_msg_.buttons[0] = joy_msg->buttons[A];
        joy_pub_->publish(teleop_msg_);
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
        static rclcpp::Clock clk{};
        static const char* last = nullptr;
        if (last == text) return;
        last = text;
        // RCLCPP_INFO_THROTTLE(get_logger(), clk, 2000, text);
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
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    sensor_msgs::msg::Joy previous_msg_; 
    sensor_msgs::msg::Joy teleop_msg_;
    // geometry_msgs::msg::Twist cmd_vel_msg_;
    // geometry_msgs::msg::TwistStamped cmd_vel_stamped_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
