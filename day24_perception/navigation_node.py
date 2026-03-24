import rclpy                              # WHY: rclpy is the ROS2 Python library — like importing ros2 itself
from rclpy.node import Node              # WHY: Node is the base class — like MonoBehaviour in Unity
from std_msgs.msg import String          # WHY: String is a standard ROS2 message type — typed, not a plain dict
from  geometry_msgs.msg import Twist       

class navigation_node(Node):             # WHY: inherit from Node to get all ROS2 powers
    def __init__(self):
        super().__init__('navigation_node') 
        self.navigator = self.create_subscription(String, '/camera/image_raw', self.callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def callback(self,msg):
        data = msg.data
        z_part = data.split('Z range: ')[1]  # gives '1.46m → 9.33m'
        z_min = float(z_part.split('m')[0])   # '1.46' → 1.46
        twist = Twist()
        if z_min < 0.5:
            twist.linear.x = 0.0
            self.get_logger().warn(f"STOP — obstacle at {z_min}m")
            
        else:
            twist.linear.x = 0.5
            self.get_logger().info("Path clear — moving forward")

        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()                         # WHY: initialize ROS2 — must be called before anything else
    node = navigation_node()             # WHY: create our node instance
    rclpy.spin(node)                     # WHY: spin keeps the node alive — like Unity's game loop
                                         # it processes callbacks, timers, incoming messages forever
    rclpy.shutdown()                     # WHY: clean up when done

if __name__ == '__main__':
    main()

