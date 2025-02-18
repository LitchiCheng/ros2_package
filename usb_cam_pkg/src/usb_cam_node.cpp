#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class UsbCamNode : public rclcpp::Node
{
public:
    UsbCamNode() : Node("usb_cam_node")
    {
        // 初始化图像发布者
        pub_ = image_transport::create_publisher(this, "/rgb_image");

        // 打开USB相机
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            return;
        }

        // 创建定时器，定期发布图像
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), std::bind(&UsbCamNode::publishImage, this));
    }

private:
    void publishImage()
    {
        cv::Mat frame;
        if (cap_.read(frame)) {
            // 将OpenCV图像转换为ROS 2图像消息
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            // 发布图像消息
            pub_.publish(msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read frame from camera");
        }
    }

    image_transport::Publisher pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UsbCamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}