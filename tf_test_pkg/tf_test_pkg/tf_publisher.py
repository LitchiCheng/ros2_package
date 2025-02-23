import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)
        self.pitch = 0.0

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = '/base_link'
        t.child_frame_id = '/test_link'

        # 设置平移
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 增加pitch角度
        self.pitch += 0.2
        if self.pitch > 2 * math.pi:
            self.pitch -= 2 * math.pi

        # 设置旋转（仅pitch变化）
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, self.pitch, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 发布TF变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()