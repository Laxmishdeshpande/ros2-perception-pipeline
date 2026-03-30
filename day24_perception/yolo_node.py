import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class Yolo_Node(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.img_pub = self.create_publisher(Image, '/yolo/detections_image', 10)

        self.bridge = CvBridge()
        self.model  = YOLO('yolov8n.pt')   # nano — fastest, smallest

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge failed: {e}')
            return
        results = self.model(frame)
        annotated = results[0].plot()

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header   # keep same timestamp
        except Exception as e:
            self.get_logger().error(f'CvBridge encode failed: {e}')
            return
        self.img_pub.publish(out_msg)
        # After results = self.model(frame), before annotated:
        for box in results[0].boxes:
            class_name = self.model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            if conf > 0.5:    # only log confident detections
                self.get_logger().info(f'Detected: {class_name} ({conf:.2f})')

def main():
    rclpy.init()
    node = Yolo_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

