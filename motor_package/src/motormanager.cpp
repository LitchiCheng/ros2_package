#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor_package_msg/msg/cmd.hpp"
#include "motor_package_msg/msg/info.hpp"

class Motors : public rclcpp::Node
{
public:
    Motors(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s is .", name.c_str());

        motorinfo_publisher_ = this->create_publisher<motor_package_msg::msg::Info>("motorinfo", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Motors::timer_callback, this));
        motorcmd_subscriber_ = this->create_subscription<motor_package_msg::msg::Cmd>("motorcmd", 10, std::bind(&Motors::motorcmd_callback, this, std::placeholders::_1));
    }

private:
    void timer_callback()
    {
        motor_package_msg::msg::Info message;
        message.name = {"m1","m2"};
        message.state = {0x37, 0x37};
        message.speedcnt = {0,0};
        motorinfo_publisher_->publish(message);
    }
    rclcpp::Subscription<motor_package_msg::msg::Cmd>::SharedPtr motorcmd_subscriber_;
    void motorcmd_callback(const motor_package_msg::msg::Cmd::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "recv motorcmd %s ", msg->name[0].c_str());
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<motor_package_msg::msg::Info>::SharedPtr motorinfo_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Motors>("Motors");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
