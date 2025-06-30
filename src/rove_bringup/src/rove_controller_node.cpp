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

class RoveController : public rclcpp::Node {
public:
    RoveController() : Node("rove_controller_node") {
        // Buttons
        BTN_1 = declare_parameter("BTN_1", 0);
        BTN_2 = declare_parameter("BTN_2", 1);

        // Subscribe to the joy topic
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/spacemouse_joy", 10, std::bind(&RoveController::joyCallback, this, std::placeholders::_1)
        );
        // cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&RoveController::cmdvelCallback, this, std::placeholders::_1));

        joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("/rove/joy", 1);

        // auto x = create_wall_timer(
        //     std::chrono::milliseconds(1000/20), std::bind(&RoveController::cmdvelTimerCallback, this, std::placeholders::_1)
        // );

        previous_msg_ = sensor_msgs::msg::Joy();
        // previous_msg_.axes.resize(6, 0.0);
        // previous_msg_.buttons.resize(10, 0);

        // teleop_msg_ = sensor_msgs::msg::Joy();
        // teleop_msg_.axes.resize(2,0);
        // teleop_msg_.buttons.resize(1,0);

        gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(this, "/robotiq_gripper_controller/gripper_cmd");
        
        // Check if action server is available
        if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
        }

        // cmd_vel_msg_ = geometry_msgs::msg::Twist();
        // cmd_vel_stamped_msg_ = geometry_msgs::msg::TwistStamped();
    }

private:
    int BTN_1, BTN_2;

    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;

    // void nextMode(){
    //     selected_mode = (selected_mode + 1) % 2; // Number of modes
    //     RCLCPP_INFO(get_logger(), "profil %i", selected_mode);
    // }

    void common_action(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        // if (buttton_down(joy_msg, Y)){
        //     nextMode();
        // }

        if (buttton_down(joy_msg, BTN_1)){
            gripper_opened_ = !gripper_opened_;
            // position, effort
            if(gripper_opened_){
                sendGripperGoal(0.0, 10.0);
            }
            else{
                sendGripperGoal(1.0, 10.0);
            }
        }

        // if(buttton_up(joy_msg, B)){
        //     log("Common B up");
        // }
    }

    // void arm_action(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    //     log("Arm");
    //     if(button_pressed(joy_msg, B)){
    //         log("Arm B pressed");
    //     }

    //     if(buttton_up(joy_msg, X)){
    //         log("Arm X up");
    //     }

    //     if(buttton_down(joy_msg, X)){
    //         log("Arm X down");
    //     }

    // }

    // void flipper_action(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    //     log("Flipper...");
    //     teleop_msg_.axes[0] = joy_msg->axes[LS_X];
    //     teleop_msg_.axes[1] = joy_msg->axes[LS_Y];
    //     teleop_msg_.buttons[0] = joy_msg->buttons[A];
    //     joy_pub_->publish(teleop_msg_);
    // }

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
        // switch (selected_mode){
        //     case ARM_MODE:
        //         arm_action(joy_msg);
        //         break;
        //     case FLIPPER_MODE:
        //         flipper_action(joy_msg);
        //         break;
        // }

        previous_msg_ = *joy_msg;
    }

    void sendGripperGoal(double position, double max_effort) {
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = max_effort;

        log("Sending goal");

        gripper_action_client_->async_send_goal(goal_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    sensor_msgs::msg::Joy previous_msg_; 
    sensor_msgs::msg::Joy teleop_msg_;
    // geometry_msgs::msg::Twist cmd_vel_msg_;
    // geometry_msgs::msg::TwistStamped cmd_vel_stamped_msg_;
    bool gripper_opened_ = true;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
