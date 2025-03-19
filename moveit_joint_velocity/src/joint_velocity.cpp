#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_joint_velocity.joint_velocity_node");

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
std::string joint_name = "panda_joint1";
double velocity = 0.0;

void publishCommands()
{
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->joint_names.push_back(joint_name);
    msg->velocities.push_back(velocity);
    joint_cmd_pub_->publish(std::move(msg));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("joint_velocity_node", node_options);

  node_->declare_parameter("velocity", velocity);
  node_->declare_parameter("joint_name", joint_name);
  node_->get_parameter("velocity", velocity);
  node_->get_parameter("joint_name", joint_name);

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

  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("joint_velocity_node/delta_joint_cmds", 10);
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