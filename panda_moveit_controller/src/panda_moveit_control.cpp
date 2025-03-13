#include <rclcpp/rclcpp.hpp>
// 运动规划和执行功能进行交互的头文件
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include <cmath>

// 定义一个将度数转换为弧度的函数
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("panda_moveit_control");

    // 创建MoveGroupInterface对象，指定规划组名称
    moveit::planning_interface::MoveGroupInterface move_group(node, "so-arm100-groups");

    // 获取规划组的参考坐标系
    std::string planning_frame = move_group.getPlanningFrame();
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", planning_frame.c_str());

    // 获取规划组的末端执行器名称
    std::string end_effector_link = move_group.getEndEffectorLink();
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", end_effector_link.c_str());

    // 设置目标位置
    geometry_msgs::msg::Pose target_pose;

    // 定义欧拉角（单位：弧度）
    double roll = degreesToRadians(0.0);
    double pitch = degreesToRadians(180.0);
    double yaw = degreesToRadians(0.0);

    // 创建一个四元数对象
    tf2::Quaternion quaternion;
    // 根据欧拉角设置四元数
    quaternion.setRPY(roll, pitch, yaw);

    // 将tf2的四元数转换为geometry_msgs的四元数
    target_pose.orientation = tf2::toMsg(quaternion);

    target_pose.position.x = 0.2;
    target_pose.position.y = 0.1;
    target_pose.position.z = 0.1;
    // 定义转换矩阵
    Eigen::Matrix3d transform_matrix;
    transform_matrix << 1, 0, 0,
                        0, 1, 0,
                        0, 0, -1;

    // 将位置向量转换为Eigen向量
    Eigen::Vector3d original_position(target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // 进行矩阵乘法
    Eigen::Vector3d new_position = transform_matrix * original_position;

    // 更新目标位置
    target_pose.position.x = new_position(0);
    target_pose.position.y = new_position(1);
    target_pose.position.z = new_position(2);
    RCLCPP_INFO(node->get_logger(), "new position %f %f %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group.setPoseTarget(target_pose);

    // 进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Planning %s", success ? "SUCCEEDED" : "FAILED");

    // 执行运动规划
    if (success) {
        move_group.execute(my_plan);
    }

    rclcpp::shutdown();
    return 0;
}