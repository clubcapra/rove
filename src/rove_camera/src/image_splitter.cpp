#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSplitter : public rclcpp::Node {
public:
    ImageSplitter() : Node("image_splitter") {
        this->declare_parameter<int>("requested_width", 1440);
        requested_width_ = this->get_parameter("requested_width").as_int();
        rclcpp::QoS qos = rclcpp::SensorDataQoS();

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/input_image/compressed", qos, std::bind(&ImageSplitter::image_callback, this, std::placeholders::_1));
        angle_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/requested_angle", qos, std::bind(&ImageSplitter::angle_callback, this, std::placeholders::_1));
        top_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/requested_top", qos, std::bind(&ImageSplitter::top_callback, this, std::placeholders::_1));
        zoom_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/requested_zoom", qos, std::bind(&ImageSplitter::zoom_callback, this, std::placeholders::_1));

        publisher_requested_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/requested_image/compressed", qos);
    }

private:
    int requested_angle_ = 0;
    int requested_width_;
    int requested_top_ = 50;
    float requested_zoom_ = 1.0;  // Default zoom level (1.0 means no zoom)

    void angle_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int r_angle = msg->data % 360;
        if (r_angle < 0){
            r_angle += 360;
        }
        requested_angle_ = r_angle;
    }

    void top_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        requested_top_ = std::min(std::max(0, msg->data), 100);
    }

    void zoom_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        requested_zoom_ = std::max(1.0f, msg->data);  // Ensure zoom is at least 1.0
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat img;
        try {
            img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV decoding error: %s", e.what());
            return;
        }
        
        cv::Mat requested_image = treat_image(img, requested_angle_, requested_zoom_);
        publisher_requested_->publish(create_msg(msg, requested_image));
    }

    cv::Mat treat_image(cv::Mat img, int angle, float zoom){
        int height = img.rows;
        int width = img.cols;

        int crop_width = static_cast<int>(requested_width_ / zoom);
        int crop_height = static_cast<int>(height / zoom);

        int start_y = static_cast<int>((height - crop_height) * requested_top_ / 100);
        int start_x = (width - crop_width) / 2 + (angle * width / 360);
        start_x = start_x % width;

        cv::Mat requested_part;
        if (start_x + crop_width <= width){
            requested_part = cv::Mat(img, cv::Rect(start_x, start_y, crop_width, crop_height));
        }
        else{
            cv::Mat left_part(img, cv::Rect(0, start_y, start_x + crop_width - width, crop_height));
            cv::Mat right_part(img, cv::Rect(start_x, start_y, width - start_x, crop_height));
            cv::hconcat(right_part, left_part, requested_part);
        }
        
        // Resize using cv::Size
        cv::Mat resized;
        cv::resize(requested_part, resized, cv::Size(requested_width_, height));
        
        return resized;
    }

    sensor_msgs::msg::CompressedImage create_msg (const sensor_msgs::msg::CompressedImage::SharedPtr source_msg, cv::Mat img){
        sensor_msgs::msg::CompressedImage msg;
        msg.header = source_msg->header;
        msg.format = "jpeg";
        cv::imencode(".jpg", img, msg.data);
        return msg; 
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr angle_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr top_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr zoom_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_requested_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSplitter>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
