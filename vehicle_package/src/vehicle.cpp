#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor_package_msg/msg/cmd.hpp"
#include "motor_package_msg/msg/info.hpp"

class Vehicle : public rclcpp::Node
{
public:
    Vehicle(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s is .", name.c_str());

        motorcmd_publisher_ = this->create_publisher<motor_package_msg::msg::Cmd>("motorcmd", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Vehicle::timer_callback, this));
        motorinfo_subscriber_ = this->create_subscription<motor_package_msg::msg::Info>("motorinfo", 10, std::bind(&Vehicle::motorinfo_callback, this, std::placeholders::_1));
        this->declare_parameter("log_flag", false);     /*声明参数*/
        this->get_parameter("log_flag", log_flag); /*获取参数*/   
    }

private:
    bool log_flag;
    void timer_callback()
    {
        motor_package_msg::msg::Cmd message;
        message.name = {"m1","m2"};
        message.mode = {"pv", "pv"};
        message.speedcnt = {0,0};
        this->get_parameter("log_flag", log_flag); /*获取参数*/
        if(log_flag)
            RCLCPP_INFO(this->get_logger(), "Publishing");
        motorcmd_publisher_->publish(message);
    }
    rclcpp::Subscription<motor_package_msg::msg::Info>::SharedPtr motorinfo_subscriber_;
    void motorinfo_callback(const motor_package_msg::msg::Info::SharedPtr msg)
    {
        this->get_parameter("log_flag", log_flag); /*获取参数*/
        if(log_flag){
            RCLCPP_INFO(this->get_logger(), "recv motorinfo %s ", msg->name[0].c_str());
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<motor_package_msg::msg::Cmd>::SharedPtr motorcmd_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Vehicle>("Vehicle");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
