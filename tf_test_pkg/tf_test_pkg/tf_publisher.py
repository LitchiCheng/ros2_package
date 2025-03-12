import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)
        # self.pitch = 0.0
        self.move_x = 0.0
        self.move_y = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _broadcast_tf(self, parent_frame, child_frame, x, y, z, yaw, pitch, roll):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # 设置平移
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(math.radians(yaw), math.radians(pitch), math.radians(roll))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 发布TF变换
        self.tf_broadcaster.sendTransform(t)

    def broadcast_tf(self):
        self.move_x += 0.001
        self.move_y += 0.0001
        self._broadcast_tf('/map_link', '/base_link', self.move_x, self.move_y, 0.0, 0.0, 0.0, 0.0)
        self._broadcast_tf('/base_link', '/camera_link', 0.0, 0.0, 2.0, 0.0, 0.0, 0.0)
        self._broadcast_tf('/base_link', '/right_arm_base_link', 0.0, -1.0, 1.5, 90.0, 90.0, 0.0)
        self._broadcast_tf('/map_link', '/object_link', 1.0, 0.15, 1.7, 0.0, 0.0, 0.0)
        try:
            trans = self.tf_buffer.lookup_transform(
                'right_arm_base_link', 'object_link', rclpy.time.Time())
            self.get_logger().info(f"Translation: {trans.transform.translation}")
            self.get_logger().info(f"Rotation: {trans.transform.rotation}")
            print(type(trans.transform.translation))
            print(type(trans.transform.rotation))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Could not find transform between /right_arm_base_link and /object_link")

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
colcon build --packages-select tf_test_pkg
source install/setup.bash
ros2 run tf_test_pkg tf_publisher

colcon build --packages-select tf_test_pkg ; source install/setup.bash ; ros2 run tf_test_pkg tf_publisher

'''
