import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from your_custom_msgs.msg import RobotInfo  # 请替换为实际的消息类型
import socket

class TcpRos2Bridge(Node):
    def __init__(self):
        super().__init__('tcp_ros2_bridge')
        # 创建Twist消息发布者
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # 创建RobotInfo消息订阅者
        self.subscription = self.create_subscription(
            RobotInfo,
            'robot_info',
            self.robot_info_callback,
            10)
        self.subscription  # 防止未使用的变量警告

        # 初始化TCP连接
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('127.0.0.1', 12345)  # 请根据实际情况修改
        try:
            self.tcp_socket.connect(server_address)
            self.get_logger().info(f"Connected to TCP server at {server_address}")
            # 启动TCP接收线程
            self.get_logger().info("Starting TCP receive loop")
            self.tcp_receive_loop()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to TCP server: {e}")

    def tcp_receive_loop(self):
        while rclpy.ok():
            try:
                data = self.tcp_socket.recv(1024).decode()
                if not data:
                    break
                vx, vy, vw = map(float, data.split(','))
                twist_msg = Twist()
                twist_msg.linear.x = vx
                twist_msg.linear.y = vy
                twist_msg.angular.z = vw
                self.publisher_.publish(twist_msg)
                self.get_logger().info(f"Received vx={vx}, vy={vy}, vw={vw} and published Twist message")
            except Exception as e:
                self.get_logger().error(f"Error receiving data from TCP server: {e}")
                break
        self.tcp_socket.close()

    def robot_info_callback(self, msg):
        try:
            # 假设RobotInfo消息有linear_x, linear_y, angular_z字段
            data_to_send = f"{msg.linear_x},{msg.linear_y},{msg.angular_z}"
            self.tcp_socket.sendall(data_to_send.encode())
            self.get_logger().info(f"Sent robot info to TCP server: {data_to_send}")
        except Exception as e:
            self.get_logger().error(f"Error sending robot info to TCP server: {e}")

def main(args=None):
    rclpy.init(args=args)
    tcp_ros2_bridge = TcpRos2Bridge()
    rclpy.spin(tcp_ros2_bridge)
    tcp_ros2_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()