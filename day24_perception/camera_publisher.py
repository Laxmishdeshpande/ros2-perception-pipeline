import rclpy                              # WHY: rclpy is the ROS2 Python library — like importing ros2 itself
from rclpy.node import Node              # WHY: Node is the base class — like MonoBehaviour in Unity
from sensor_msgs.msg import Image       # WHY: String is a standard ROS2 message type — typed, not a plain dict
from cv_bridge import CvBridge
import cv2
import os
import glob         

class CameraPublisher(Node):             # WHY: inherit from Node to get all ROS2 powers
    def __init__(self, image_dir):
        super().__init__('camera_publisher')  # WHY: registers this node with ROS2 under the name 'camera_publisher'
                                              # two nodes with same name = ROS2 kills the older one

        # WHY: create_publisher tells ROS2 three things:
        # 1. String — what message type we're sending
        # 2. '/camera/image_raw' — which topic to publish on
        # 3. 10 — queue size (max 10 messages backlog before dropping old ones)
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)        # WHY: create_timer replaces while True + time.sleep()
        # ROS2 calls self.publish_frame automatically every 0.5 seconds
        # cleaner than threading — ROS2 manages the loop for you
        self.timer = self.create_timer(0.5, self.publish_frame)

        self.frame_id = 0  
        self.bridge = CvBridge()                                          # translator instance — reuse for every frame
        self.image_files = sorted(glob.glob(os.path.join(image_dir, '*.png')))  # find all PNGs, sorted by name
        self.current_index = 0                                            # which frame we're on
        self.total_frames = len(self.image_files)                         # total count              # WHY: track which frame we're on

    def publish_frame(self):
        if self.current_index >= self.total_frames:    # no more images? stop the timer
            self.timer.cancel()
            return

    # STEP 1: Load image from disk
        image_path = self.image_files[self.current_index]
        frame = cv2.imread(image_path)                 # returns (480, 640, 3) BGR NumPy array

        if frame is None:                              # corrupted file? skip it
            self.current_index += 1
            return

    # STEP 2: Convert BGR → grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # (480, 640, 3) → (480, 640) uint8

    # STEP 3: NumPy → sensor_msgs/Image  ← THIS IS THE DAY 28 FIX
        img_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')

    # STEP 4: Add timestamp and frame ID
        img_msg.header.stamp = self.get_clock().get_rostime()
        img_msg.header.frame_id = 'camera_frame'

    # STEP 5: Publish
        self.publisher.publish(img_msg)
        self.current_index += 1

def main():
    rclpy.init()
    image_dir = '/path/to/tum_dataset/rgb'   # ← change this to your actual path
    node = CameraPublisher(image_dir)
    rclpy.spin(node)
    rclpy.shutdown()                  # WHY: clean up when done

if __name__ == '__main__':
    main()