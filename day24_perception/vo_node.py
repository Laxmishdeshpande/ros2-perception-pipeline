import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class vo_node(Node):
    def __init__(self):
        super().__init__('vo_node')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)   # FIX 6: consistent name
        self.frame_prev = None
        self.bridge = CvBridge()
        self.R_global = np.eye(3)
        self.t_global = np.zeros((3, 1))
        self.frame_count = 0
        self.path_pub  = self.create_publisher(Path, '/vo/path', 10)
        self.path_msg  = Path()

        self.K = np.array([[565.6, 0.0,   320.2],
                   [0.0,   565.6, 180.2],
                   [0.0,   0.0,   1.0  ]])

    def image_callback(self, msg):                         # FIX 1: indented inside class
        try:
            frame_curr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            frame_curr = cv2.resize(frame_curr, (640, 360))
        except Exception as e:
            self.get_logger().error(f'CvBridge failed: {e}')
            return

        if self.frame_prev is None:
            self.frame_prev = frame_curr
            self.get_logger().info('First frame stored. Waiting for frame 2.')
            return

        pts_prev, pts_curr = self.track_features(self.frame_prev, frame_curr)

        if pts_prev is not None and len(pts_prev) >= 5:
            R, t = self.estimate_pose(pts_prev, pts_curr)

            if R is not None:
                self.t_global = self.t_global + self.R_global @ t
                self.R_global = self.R_global @ R
                self.broadcast_pose(msg.header.stamp)

                self.frame_count += 1
                if self.frame_count % 20 == 0:
                    cam_pos = -self.R_global.T @ self.t_global
                    self.get_logger().info(
                        f'Frame {self.frame_count} | '
                        f'Features: {len(pts_prev)} | '
                        f'Pos: ({cam_pos[0,0]:.3f}, {cam_pos[1,0]:.3f}, {cam_pos[2,0]:.3f})'
                    )

        self.frame_prev = frame_curr

    def track_features(self, frame_prev, frame_curr):      # FIX 2: same indent as image_callback
        """
        Track features using Lucas-Kanade optical flow.
        Frames arrive already grayscale from camera_publisher.
        """                                                 # FIX 3: proper docstring
        # FIX 4: REMOVED cvtColor — frames are already mono8 grayscale
        # calling COLOR_BGR2GRAY on a 1-channel image would crash OpenCV

        pts_prev = cv2.goodFeaturesToTrack(
            frame_prev,
            maxCorners=500,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=7
        )

        if pts_prev is None or len(pts_prev) == 0:
            return None, None

        pts_curr, status, error = cv2.calcOpticalFlowPyrLK(
            frame_prev, frame_curr,
            pts_prev, None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                      30, 0.01)
        )

        status = status.ravel()
        pts_prev = pts_prev[status == 1]
        pts_curr = pts_curr[status == 1]

        return pts_prev, pts_curr

    def estimate_pose(self, pts_prev, pts_curr):            # FIX 1: indented inside class
        """
        Estimate camera motion using Essential Matrix decomposition.
        """
        E, mask = cv2.findEssentialMat(
            pts_prev, pts_curr, self.K,                     # FIX 5: self.K not K
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        _, R, t, mask = cv2.recoverPose(
            E, pts_prev, pts_curr, self.K                   # FIX 5: self.K not K
        )

        return R, t

    def broadcast_pose(self, stamp):                        # FIX 1: indented inside class
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'camera_frame'

        cam_pos = -self.R_global.T @ self.t_global
        tf.transform.translation.x = float(cam_pos[0])
        tf.transform.translation.y = float(cam_pos[1])
        tf.transform.translation.z = float(cam_pos[2])

        quat = Rotation.from_matrix(self.R_global.T).as_quat()
        tf.transform.rotation.x = float(quat[0])
        tf.transform.rotation.y = float(quat[1])
        tf.transform.rotation.z = float(quat[2])
        tf.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(tf)               # FIX 6: matches __init__ name

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = float(cam_pos[0])
        pose.pose.position.y = float(cam_pos[1])
        pose.pose.position.z = float(cam_pos[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        
        self.path_msg.header.stamp = stamp
        self.path_msg.header.frame_id = 'odom'
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = vo_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()