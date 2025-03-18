#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo_panda.panda_servo_node");

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
double linear_x = 0.0;
double linear_y = 0.0;
double linear_z = 0.0;
double angular_x = 0.0;
double angular_y = 0.0;
double angular_z = 0.0;

void publishCommands()
{
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "panda_link0";
    msg->twist.linear.x = linear_x;
    msg->twist.linear.y = linear_y;
    msg->twist.linear.z = linear_z;
    msg->twist.angular.x = angular_x;
    msg->twist.angular.y = angular_y;
    msg->twist.angular.z = angular_z;

    twist_cmd_pub_->publish(std::move(msg));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("panda_servo_node", node_options);

  node_->declare_parameter("linear_x", linear_x);
  node_->declare_parameter("linear_y", linear_y);
  node_->declare_parameter("linear_z", linear_z);
  node_->declare_parameter("angular_x", angular_x);
  node_->declare_parameter("angular_y", angular_y);
  node_->declare_parameter("angular_z", angular_z);
  node_->get_parameter("linear_x", linear_x);
  node_->get_parameter("linear_y", linear_y);
  node_->get_parameter("linear_z", linear_z);
  node_->get_parameter("angular_x", angular_x);
  node_->get_parameter("angular_y", angular_y);
  node_->get_parameter("angular_z", angular_z);


  // 监控机器人的关节状态和场景中的碰撞对象信息，并将更新后的规划场景信息发布出去，为后续的运动规划和控制提供基础
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("panda_servo_node/delta_twist_cmds", 10);
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);
  servo->start();

  rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, publishCommands);

  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
