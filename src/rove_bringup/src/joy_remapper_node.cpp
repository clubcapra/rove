// rove_controller.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <map>


class JoyRemapper : public rclcpp::Node {
public:
    JoyRemapper() : Node("joy_remapper_node") {

        // From
        // Buttons
        from.A = declare_parameter("A", 0);
        from.B = declare_parameter("B", 1);
        from.X = declare_parameter("X", 2);
        from.Y = declare_parameter("Y", 3);
        from.LB = declare_parameter("LB", 4);
        from.RB = declare_parameter("RB", 5);
        from.LT_BTN = declare_parameter("LT_BTN", 6);
        from.RT_BTN = declare_parameter("RT_BTN", 7);
        from.VIEW = declare_parameter("VIEW", 8);
        from.MENU = declare_parameter("MENU", 9);
        from.LS = declare_parameter("LS", 10);
        from.RS = declare_parameter("RS", 11);
        from.D_PAD_UP = declare_parameter("D_PAD_UP", 12);
        from.D_PAD_DOWN = declare_parameter("D_PAD_DOWN", 13);
        from.D_PAD_LEFT = declare_parameter("D_PAD_LEFT", 14);
        from.D_PAD_RIGHT = declare_parameter("D_PAD_RIGHT", 15);
        from.XBOX = declare_parameter("XBOX", 16);
        from.SHARE = declare_parameter("SHARE", 17);

        // Axes
        from.LS_X = declare_parameter("LS_X", 0);
        from.LS_Y = declare_parameter("LS_Y", 1);
        from.RS_X = declare_parameter("RS_X", 2);
        from.RS_Y = declare_parameter("RS_Y", 3);

        // To
        // Buttons
        to.A = declare_parameter("to.A", 0);
        to.B = declare_parameter("to.B", 1);
        to.X = declare_parameter("to.X", 2);
        to.Y = declare_parameter("to.Y", 3);
        to.LB = declare_parameter("to.LB", 4);
        to.RB = declare_parameter("to.RB", 5);
        to.VIEW = declare_parameter("to.VIEW", 6);
        to.MENU = declare_parameter("to.MENU", 7);
        to.XBOX = declare_parameter("to.XBOX", 8);
        to.LS = declare_parameter("to.LS", 9);
        to.RS = declare_parameter("to.RS", 10);
        to.SHARE = declare_parameter("to.SHARE", 11);

        // Axes
        to.LS_X = declare_parameter("to.LS_X", 0);
        to.LS_Y = declare_parameter("to.LS_Y", 1);
        to.LT = declare_parameter("to.LT", 2);
        to.RS_X = declare_parameter("to.RS_X", 3);
        to.RS_Y = declare_parameter("to.RS_Y", 4);
        to.RT = declare_parameter("to.RT", 5);
        to.D_PAD_X = declare_parameter("to.D_PAD_X", 6);
        to.D_PAD_Y = declare_parameter("to.D_PAD_Y", 7);

        // Subscribe to the joy topic
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyRemapper::joyCallback, this, std::placeholders::_1)
        );

        joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("/joy_out", 1);

        buttonsMap = {
            {from.A, to.A},
            {from.B, to.B},
            {from.X, to.X},
            {from.Y, to.Y},
            {from.LB, to.LB},
            {from.RB, to.RB},
            {from.VIEW, to.VIEW},
            {from.MENU, to.MENU},
            {from.XBOX, to.XBOX},
            {from.LS, to.LS},
            {from.RS, to.RS},
            {from.SHARE, to.SHARE},
        };

        axesMap = {
            {from.LS_X, to.LS_X},
            {from.LS_Y, to.LS_Y},
            {from.RS_X, to.RS_X},
            {from.RS_Y, to.RS_Y},
        };
    }

private:
    struct FROM {
        int A, B, X, Y, LB, RB, LT_BTN, RT_BTN, VIEW, MENU, LS, RS, D_PAD_UP, D_PAD_DOWN, D_PAD_LEFT, D_PAD_RIGHT, XBOX, SHARE;
        int LS_X, LS_Y, RS_X, RS_Y;
    };
    struct TO {
        int A, B, X, Y, LB, RB, VIEW, MENU, XBOX, LS, RS, SHARE;
        int LS_X, LS_Y, LT, RS_X, RS_Y, RT, D_PAD_X, D_PAD_Y;
    };
    FROM from;
    TO to;
    std::map<int, int> buttonsMap{};
    std::map<int, int> axesMap{};

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        auto res = sensor_msgs::msg::Joy();
        res.buttons.resize(12);
        for (auto kv : buttonsMap) {
            if (kv.first < joy_msg->buttons.size())
                res.buttons[kv.second] = joy_msg->buttons[kv.first];
        }

        res.axes.resize(8);
        for (auto kv : axesMap) {
            if (kv.first < joy_msg->axes.size())
                res.axes[kv.second] = joy_msg->axes[kv.first];
        }

        res.axes[to.D_PAD_X] = joy_msg->buttons[from.D_PAD_RIGHT] - joy_msg->buttons[from.D_PAD_LEFT];
        res.axes[to.D_PAD_Y] = joy_msg->buttons[from.D_PAD_UP] - joy_msg->buttons[from.D_PAD_DOWN];
        res.axes[to.LT] = 1 - (2 * joy_msg->buttons[from.LT_BTN]);
        res.axes[to.RT] = 1 - (2 * joy_msg->buttons[from.RT_BTN]);

        res.header = joy_msg->header;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyRemapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
