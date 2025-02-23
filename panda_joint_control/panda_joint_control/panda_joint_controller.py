import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import tty
import termios

class PandaJointController(Node):
    def __init__(self):
        super().__init__('panda_joint_controller')
        # 创建发布者，发布到 "/panda_arm_controller/joint_trajectory" 话题
        self.publisher_ = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        # 定义关节名称列表
        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        # 初始化关节位置
        self.joint_positions = [0.0] * 7
        # 定义每个关节位置的递增步长
        self.step = 0.1

    def get_key(self):
        # 获取终端输入的按键
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def send_trajectory(self):
        # 创建 JointTrajectory 消息
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # 创建 JointTrajectoryPoint 并设置目标位置等信息
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        # 设置运动时间
        point.time_from_start = rclpy.duration.Duration(seconds=1).to_msg()

        # 将点添加到轨迹消息中
        trajectory_msg.points.append(point)

        # 发布轨迹消息
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info('Sent joint trajectory command')

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == '-':
                self.minus_pressed = True
            elif key in ['1', '2', '3', '4', '5', '6', '7']:
                index = int(key) - 1
                if self.minus_pressed:
                    self.joint_positions[index] -= self.step
                    self.minus_pressed = False
                else:
                    self.joint_positions[index] += self.step
                self.send_trajectory()
            elif key == '\x03':  # Ctrl+C 退出
                break

def main(args=None):
    rclpy.init(args=args)
    panda_joint_controller = PandaJointController()
    panda_joint_controller.run()
    panda_joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()