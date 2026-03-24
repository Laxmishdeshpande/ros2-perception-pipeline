import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class CameraTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tf_broadcaster')
        
        # WHY: StaticTransformBroadcaster publishes transforms that never change
        # camera is bolted to robot — its position relative to base never moves
        self.broadcaster = StaticTransformBroadcaster(self)
        
        # WHY: TransformStamped is the message type for a single transform
        # it contains: parent frame, child frame, translation, rotation
        transform = TransformStamped()
        
        # WHY: timestamp tells other nodes when this transform was measured
        transform.header.stamp = self.get_clock().now().to_msg()
        
        # WHY: parent frame — robot body center, everything is relative to this
        transform.header.frame_id = 'base_link'
        
        # WHY: child frame — where the camera lives
        transform.child_frame_id = 'camera_frame'
        
        # WHY: camera is mounted 10cm forward, 30cm above robot center
        # measured with a ruler in real life, hardcoded here
        transform.transform.translation.x = 0.1   # 10cm forward
        transform.transform.translation.y = 0.0   # centered left/right
        transform.transform.translation.z = 0.3   # 30cm above base
        
        # WHY: no rotation — camera faces same direction as robot
        # quaternion for zero rotation is always (0, 0, 0, 1)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0      # WHY: w=1 means no rotation
        
        # WHY: sendTransform broadcasts this to the entire ROS2 system
        # any node can now ask "where is camera_frame relative to base_link?"
        self.broadcaster.sendTransform(transform)
        self.get_logger().info('Camera transform published: 0.1m forward, 0.3m above base_link')

def main():
    rclpy.init()
    node = CameraTransformBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()