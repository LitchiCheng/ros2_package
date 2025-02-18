#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>

#include "orbbec.h"

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher")
    {
        rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);
        depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&ImagePublisher::timer_callback, this));
    }
    void camInit()
    {
        cam_.wait4Device();
        cam_.init();
        std::thread t = std::thread([&]() {
            cam_.run();
        });
        t.detach();
    }
private:
    void timer_callback()
    {
        //CV_8UC3 8位无符号3通道图像
        cv::Mat rgb_image = cam_.getImg();
        sensor_msgs::msg::Image::SharedPtr rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_image).toImageMsg();
        rgb_publisher_->publish(*rgb_msg);
        //CV_16UC1 16位无符号单通道图像
        cv::Mat depth_image = cam_.getDepth();
        sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();
        depth_publisher_->publish(*depth_msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing image");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    OrbbecCam cam_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    node->camInit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}