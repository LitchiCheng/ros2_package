#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor_package_msg/msg/cmd.hpp"

class MotorPuber : public rclcpp::Node
{
public:
    MotorPuber(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s is .", name.c_str());

        command_publisher_ = this->create_publisher<motor_package_msg::msg::Cmd>("command", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MotorPuber::timer_callback, this));
    }

private:
    void timer_callback()
    {
        motor_package_msg::msg::Cmd message;
        message.name = "motor1";
        message.value = 0.1;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.name.c_str());
        command_publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<motor_package_msg::msg::Cmd>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MotorPuber>("MotorPuber");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
