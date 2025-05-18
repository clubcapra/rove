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
        zoom_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/requested_zoom", qos, std::bind(&ImageSplitter::zoom_callback, this, std::placeholders::_1));

        publisher_requested_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/requested_image/compressed", qos);
    }

private:
    int requested_angle_ = 0;
    int requested_width_;
    float requested_zoom_ = 1.0;  // Default zoom level (1.0 means no zoom)

    void angle_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int r_angle = msg->data % 360;
        if (r_angle < 0){
            r_angle += 360;
        }
        requested_angle_ = r_angle;
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

        // Apply zoom: crop the center of the image
        int new_width = static_cast<int>(width / zoom);
        int new_height = static_cast<int>(height / zoom);
        int start_x = (width - new_width) / 2;
        int start_y = (height - new_height) / 2;

        cv::Mat zoomed_img = img(cv::Rect(start_x, start_y, new_width, new_height));
        cv::resize(zoomed_img, zoomed_img, cv::Size(width, height));

        // Compute cropping for requested_angle_
        int center_x = width / 2 + (angle * width / 360);
        int requested_start = (center_x - requested_width_ / 2) % width;

        cv::Mat requested_part;
        if (requested_start + requested_width_ <= width){
            requested_part = cv::Mat(zoomed_img, cv::Rect(requested_start, 0, requested_width_, height));
        }
        else{
            cv::Mat left_part(zoomed_img, cv::Rect(0, 0, requested_start + requested_width_ - width, height));
            cv::Mat right_part(zoomed_img, cv::Rect(requested_start, 0, width - requested_start, height));
            cv::hconcat(right_part, left_part, requested_part);
        }

        return requested_part;
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
