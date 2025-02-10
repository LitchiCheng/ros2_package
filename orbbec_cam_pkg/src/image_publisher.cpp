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
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);
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
        // 创建一个简单的RGB图像（这里创建一个红色方块图像）
        // cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
        cv::Mat image = cam_.getImg();

        // 将OpenCV图像转换为ROS 2图像消息
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

        // 发布图像消息
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Publishing an RGB image");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
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