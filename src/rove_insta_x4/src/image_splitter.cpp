#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32.hpp>
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

        publisher_back_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/back_image/compressed", qos);
        publisher_front_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/front_image/compressed", qos);
        publisher_requested_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/requested_image/compressed", qos);
    }

private:
    int requested_angle_ = 0;
    int requested_width_;

    void angle_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int r_angle = msg->data % 360;
        if (r_angle < 0){
            r_angle += 360;
        }
        requested_angle_ = r_angle;
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat img;
        try {
            img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV decoding error: %s", e.what());
            return;
        }
        
        cv::Mat requested_image = treat_image(img, requested_angle_);
        cv::Mat front_image = treat_image(img, 0);
        cv::Mat back_image = treat_image(img, 180);

        publisher_back_->publish(create_msg(msg, back_image));
        publisher_front_->publish(create_msg(msg, front_image));
        publisher_requested_->publish(create_msg(msg, requested_image));
    }

    cv::Mat treat_image(cv::Mat img, int angle){
        int height = img.rows;
        int width = img.cols;
        
        int center_x = width / 2 + (angle * width / 360);
        int requested_start = (center_x - requested_width_ / 2) % width;

        cv::Mat requested_part;
        if (requested_start + requested_width_ <= width){
            requested_part = cv::Mat(img, cv::Rect(requested_start, 0, requested_width_, height));
        }
        else{
            cv::Mat left_part(img, cv::Rect(0, 0, requested_start + requested_width_ - width, height));
            cv::Mat right_part(img, cv::Rect(requested_start, 0, width - requested_start, height));

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
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_back_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_front_;
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
