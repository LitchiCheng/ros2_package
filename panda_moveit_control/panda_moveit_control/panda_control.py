import rclpy
from rclpy.node import Node
from moveit_ros2_planning_interface_py import MoveGroupInterface
from geometry_msgs.msg import PoseStamped

class PandaControlNode(Node):
    def __init__(self):
        super().__init__('panda_control_node')
        rclpy.init()
        # 初始化 MoveGroupInterface，指定机械臂的规划组名称
        self.move_group = MoveGroupInterface(self, 'panda_arm')

    def move_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # 创建目标位姿消息
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.move_group.get_planning_frame()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        # 设置目标位姿
        self.move_group.set_pose_target(target_pose)

        # 进行运动规划
        plan = self.move_group.plan()

        # 执行运动规划
        if plan.joint_trajectory.points:
            self.get_logger().info("Executing plan...")
            self.move_group.execute(plan)
        else:
            self.get_logger().warn("No valid plan found.")

    def run(self):
        # 设置目标位姿，这里只是示例值，你可以根据需要修改
        x = 0.5
        y = 0.0
        z = 0.3
        self.move_to_pose(x, y, z)


def main(args=None):
    node = PandaControlNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()