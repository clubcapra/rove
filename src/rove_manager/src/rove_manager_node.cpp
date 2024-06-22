#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <unordered_map>
#include <string>
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;

class RoveManagerNode : public rclcpp::Node
{
public:
    RoveManagerNode() : Node("rove_manager_node")
    {
        launch_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/manager/launch", 10, std::bind(&RoveManagerNode::launch_callback, this, std::placeholders::_1));

        kill_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/manager/kill", 10, std::bind(&RoveManagerNode::kill_callback, this, std::placeholders::_1));

        status_publisher_ = this->create_publisher<std_msgs::msg::String>("/manager/status", 10);
    }

private:
    void launch_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string package, file;
        if (!parse_launch_message(msg->data, package, file))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse launch message: %s", msg->data.c_str());
            return;
        }

        pid_t pid = fork();
        if (pid == 0) // Child process
        {
            // Create a new process group for the child process
            setpgid(0, 0);

            std::vector<char *> args;
            args.push_back(strdup("ros2"));
            args.push_back(strdup("launch"));
            args.push_back(strdup(package.c_str()));
            args.push_back(strdup(file.c_str()));
            args.push_back(nullptr);

            execvp("ros2", args.data());
            perror("execvp failed"); // Only reached if execvp fails
            _exit(1);
        }
        else if (pid > 0) // Parent process
        {
            processes_[package + "_" + file].push_back(pid);
            RCLCPP_INFO(this->get_logger(), "Launched: %s %s", package.c_str(), file.c_str());

            // Publish status
            std_msgs::msg::String status_msg;
            status_msg.data = "Running: " + package + " " + file;
            status_publisher_->publish(status_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to fork");
        }
    }

    void kill_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string package, file;
        if (!parse_kill_message(msg->data, package, file))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse kill message: %s", msg->data.c_str());
            return;
        }

        std::string key = package + "_" + file;
        auto it = processes_.find(key);
        if (it != processes_.end())
        {
            for (pid_t pid : it->second)
            {
                kill(pid, SIGKILL);
                waitpid(pid, nullptr, 0);
            }
            processes_.erase(it);
            RCLCPP_INFO(this->get_logger(), "Killed processes for: %s %s", package.c_str(), file.c_str());

            // Publish status
            std_msgs::msg::String status_msg;
            status_msg.data = "Not running: " + package + " " + file;
            status_publisher_->publish(status_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No process found for: %s %s", package.c_str(), file.c_str());
        }
    }

    bool parse_launch_message(const std::string &data, std::string &package, std::string &file)
    {
        std::istringstream stream(data);
        std::getline(stream, package, ' ');
        std::getline(stream, file, ' ');

        return !package.empty() && !file.empty();
    }

    bool parse_kill_message(const std::string &data, std::string &package, std::string &file)
    {
        std::istringstream stream(data);
        std::getline(stream, package, ' ');
        std::getline(stream, file, ' ');

        return !package.empty() && !file.empty();
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr launch_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kill_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

    std::unordered_map<std::string, std::vector<pid_t>> processes_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoveManagerNode>());
    rclcpp::shutdown();
    return 0;
}
