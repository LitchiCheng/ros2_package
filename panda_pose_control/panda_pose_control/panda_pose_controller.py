import rclpy
from rclpy.node import Node
import sys
sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped

class PandaPoseController(Node):
    def __init__(self):
        super().__init__('panda_pose_controller')
        rclpy.init()
        # 初始化MoveIt Commander
        self.robot = MoveGroupCommander("panda_arm", node=self)

    def set_end_effector_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # 创建一个PoseStamped消息
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.robot.get_planning_frame()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        # 设置目标位姿
        self.robot.set_pose_target(target_pose)

        # 进行运动规划
        plan = self.robot.plan()

        # 执行运动规划
        if plan.joint_trajectory.points:
            self.get_logger().info("Executing plan...")
            self.robot.execute(plan)
        else:
            self.get_logger().warn("No valid plan found.")

    def run(self):
        # 设置目标位姿，这里只是示例值，你可以根据需要修改
        x = 0.5
        y = 0.0
        z = 0.3
        self.set_end_effector_pose(x, y, z)


def main(args=None):
    node = PandaPoseController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()