import rclpy                              # WHY: rclpy is the ROS2 Python library — like importing ros2 itself
from rclpy.node import Node              # WHY: Node is the base class — like MonoBehaviour in Unity
from std_msgs.msg import String          # WHY: String is a standard ROS2 message type — typed, not a plain dict

class CameraPublisher(Node):             # WHY: inherit from Node to get all ROS2 powers
    def __init__(self):
        super().__init__('camera_publisher')  # WHY: registers this node with ROS2 under the name 'camera_publisher'
                                              # two nodes with same name = ROS2 kills the older one

        # WHY: create_publisher tells ROS2 three things:
        # 1. String — what message type we're sending
        # 2. '/camera/image_raw' — which topic to publish on
        # 3. 10 — queue size (max 10 messages backlog before dropping old ones)
        self.publisher = self.create_publisher(String, '/camera/image_raw', 10)

        # WHY: create_timer replaces while True + time.sleep()
        # ROS2 calls self.publish_frame automatically every 0.5 seconds
        # cleaner than threading — ROS2 manages the loop for you
        self.timer = self.create_timer(0.5, self.publish_frame)

        self.frame_id = 0                # WHY: track which frame we're on

    def publish_frame(self):             # WHY: this is the callback — ROS2 calls this every 0.5 seconds
        msg = String()                   # WHY: create a typed ROS2 String message — not a plain dict
        msg.data = f'Frame {self.frame_id} — 248250 points | Z range: 1.46m → 9.33m'
                                         # WHY: simulating what our real point cloud publisher would send
        self.publisher.publish(msg)      # WHY: send message to the topic — bus delivers to all subscribers
        self.get_logger().info(f'Published: {msg.data}')  # WHY: ROS2 logger — better than print() for robots
        self.frame_id += 1

def main():
    rclpy.init()                         # WHY: initialize ROS2 — must be called before anything else
    node = CameraPublisher()             # WHY: create our node instance
    rclpy.spin(node)                     # WHY: spin keeps the node alive — like Unity's game loop
                                         # it processes callbacks, timers, incoming messages forever
    rclpy.shutdown()                     # WHY: clean up when done

if __name__ == '__main__':
    main()