import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image as ROSImage


class YoloTargetDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_target_detection_node')
        
        # Initialize the YOLOv8 model
        self.model = YOLO("yolov8n.pt")  # 选择你训练的模型
        self.bridge = CvBridge()

        # Create a subscriber for RGB image
        self.image_sub = self.create_subscription(
            Image,
            '/rgb_image',  # 修改为你订阅的topic
            self.image_callback,
            10
        )

        # Create a publisher for output image with bounding boxes
        self.obb_pub = self.create_publisher(
            ROSImage,
            '/camera/rgb/obb_image',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Perform object detection using YOLOv8
        results = self.model(cv_image)
        
        # YOLOv8 returns a list of results, each result is a Results object
        result = results[0]  # Get the first result (assuming single image inference)
        
        # Get bounding boxes, class IDs, and confidences
        boxes = result.boxes.xywh.cpu().numpy()  # Bounding boxes (x_center, y_center, width, height)
        confidences = result.boxes.conf.cpu().numpy()  # Confidence scores
        class_ids = result.boxes.cls.cpu().numpy()  # Class IDs
        labels = result.names  # Class names

        # Draw bounding boxes on the image
        for i, box in enumerate(boxes):
            x_center, y_center, width, height = box[:4]
            confidence = confidences[i]
            class_id = int(class_ids[i])  # Get the class ID
            label = labels[class_id]  # Get the class label
            
            # Convert center to top-left coordinates for cv2
            x1 = int((x_center - width / 2))
            y1 = int((y_center - height / 2))
            x2 = int((x_center + width / 2))
            y2 = int((y_center + height / 2))

            # Draw bounding box and label on the image
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, f"{label} {confidence:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Convert the image with bounding boxes back to ROS message
        try:
            obb_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            obb_image_msg.header = Header()
            obb_image_msg.header.stamp = self.get_clock().now().to_msg()
            obb_image_msg.header.frame_id = "camera_frame"  # 根据你的相机frame进行调整
            
            # Publish the image with bounding boxes
            self.obb_pub.publish(obb_image_msg)
            self.get_logger().info("Published object-bound box image.")
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = YoloTargetDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
