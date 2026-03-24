import rclpy                              # WHY: rclpy is the ROS2 Python library — like importing ros2 itself
from rclpy.node import Node              # WHY: Node is the base class — like MonoBehaviour in Unity
from std_msgs.msg import String          # WHY: String is a standard ROS2 message type — typed, not a plain dict
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
import numpy as np
def track_features(frame_prev, frame_curr):
    """
    Track features from previous frame to current frame
    using Lucas-Kanade optical flow.
    Returns matched point pairs.
    """
    # ── Convert to grayscale ──────────────────────────────────────────
    gray_prev = cv2.cvtColor(frame_prev, cv2.COLOR_BGR2GRAY)
    gray_curr = cv2.cvtColor(frame_curr, cv2.COLOR_BGR2GRAY)

    # ── Detect good features to track in previous frame ───────────────
    pts_prev = cv2.goodFeaturesToTrack(
        gray_prev,
        maxCorners=500,      # max features to find
        qualityLevel=0.01,   # minimum quality threshold
        minDistance=10,      # minimum pixels between features
        blockSize=7          # neighborhood size for detection
    )

    # ── Track those features into current frame ───────────────────────
    pts_curr, status, error = cv2.calcOpticalFlowPyrLK(
        gray_prev, gray_curr,
        pts_prev, None,
        winSize=(21, 21),    # search window size at each pyramid level
        maxLevel=3,          # number of pyramid levels
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                  30, 0.01)
    )

    # ── Keep only successfully tracked points ─────────────────────────
    status = status.ravel()
    pts_prev = pts_prev[status == 1]
    pts_curr = pts_curr[status == 1]

    return pts_prev, pts_curr

def estimate_pose(pts_prev, pts_curr, K):
    """
    Estimate camera motion between two frames
    from tracked point correspondences.
    """
    E, mask = cv2.findEssentialMat(
        pts_prev, pts_curr, K,
        method=cv2.RANSAC,
        prob=0.999,
        threshold=1.0
    )

    _, R, t, mask = cv2.recoverPose(E, pts_prev, pts_curr, K)

    return R, t


class vo_node(Node):             # WHY: inherit from Node to get all ROS2 powers
    def __init__(self):
        super().__init__('vo_node')  # WHY: registers this node with ROS2 under the name 'camera_publisher'
        self.voCode = self.create_subscription(String, '/camera/image_raw', self.callback, 10)
        self.tf2broadcaster = TransformBroadcaster(self)
        self.frame_prev = None 

        # camera intrinsics — TUM Kinect calibrated values
        self.K = np.array([[525.0,   0.0, 319.5],
                       [  0.0, 525.0, 239.5],
                       [  0.0,   0.0,   1.0]])

    def callback(self,msg):
        frame_cur = msg.data

        if self.frame_prev is None:
            self.frame_prev = frame_cur
            return
    
        pts_prev, pts_curr = track_features(self.frame_prev , frame_cur)

            # estimate how much the camera moved
        R, t = estimate_pose(pts_prev, pts_curr,self.K)

        # TODO: in real system msg.data would be actual image bytes
        self.frame_prev = frame_cur
    
def main():
    rclpy.init()                         # WHY: initialize ROS2 — must be called before anything else
    node = vo_node()             # WHY: create our node instance
    rclpy.spin(node)                     # WHY: spin keeps the node alive — like Unity's game loop
                                         # it processes callbacks, timers, incoming messages forever
    rclpy.shutdown() 

if __name__ == '__main__':
    main()
