#!/usr/bin/env python3
"""
seg_node.py — Day 34
=====================
Semantic segmentation node using DeepLabV3.
Subscribes to camera images, runs pixel-level classification,
publishes segmented image for visualization and costmap building.

Pipeline position:
  camera → /camera/image_raw → seg_node → /seg/colored (visualization)
                                        → /seg/map (raw class IDs for costmap)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision
from torchvision import transforms


class SegmentationNode(Node):
    def __init__(self):
        super().__init__('seg_node')

        # ─── Subscribe to camera images ───────────────────
        # Same topic as vo_node and yolo_node
        # Three nodes, one camera feed — pub/sub power
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # ─── Two publishers ───────────────────────────────
        # 1. Colored segmentation overlay — for human viewing in rqt
        self.color_pub = self.create_publisher(
            Image, '/seg/colored', 10
        )
        # WHY separate topic: rqt_image_view can display this directly
        # Shows the scene with colored class overlays

        # 2. Raw segmentation map — for other nodes (costmap builder)
        self.map_pub = self.create_publisher(
            Image, '/seg/map', 10
        )
        # WHY raw map: contains class ID per pixel (0-20)
        # A costmap node reads this and marks:
        #   person pixels → lethal obstacle (cost 254)
        #   road pixels → free space (cost 0)
        #   building pixels → obstacle (cost 254)

        self.bridge = CvBridge()

        # ─── Load DeepLabV3 model ─────────────────────────
        self.get_logger().info('Loading DeepLabV3 model...')
        self.model = torchvision.models.segmentation.deeplabv3_resnet101(
            weights=torchvision.models.segmentation.DeepLabV3_ResNet101_Weights.DEFAULT
        )
        self.model.eval()
        # WHY eval: disables dropout and batch norm training behavior
        # Same as MiDaS and YOLO — always call eval() for inference

        # ─── Preprocessing transform ─────────────────────
        # DeepLabV3 expects ImageNet-normalized input
        # WHY these exact values: model was TRAINED with this normalization
        # Wrong normalization = garbage output
        self.preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])

        # ─── Color map for visualization ──────────────────
        # 21 PASCAL VOC classes, each gets a distinct color
        self.CLASSES = [
            'background', 'aeroplane', 'bicycle', 'bird', 'boat',
            'bottle', 'bus', 'car', 'cat', 'chair',
            'cow', 'dining table', 'dog', 'horse', 'motorbike',
            'person', 'potted plant', 'sheep', 'sofa', 'train', 'tv/monitor'
        ]

        self.COLORS = np.array([
            [0,0,0],       [128,0,0],     [0,128,0],     [128,128,0],   [0,0,128],
            [128,0,128],   [0,128,128],   [128,128,128], [64,0,0],      [192,0,0],
            [64,128,0],    [192,128,0],   [64,0,128],    [192,0,128],   [64,128,128],
            [192,128,128], [0,64,0],      [128,64,0],    [0,192,0],     [128,192,0],
            [0,64,128]
        ], dtype=np.uint8)

        # ─── Frame counter for logging ────────────────────
        self.frame_count = 0

        self.get_logger().info(
            f'Segmentation node ready — {len(self.CLASSES)} classes, '
            f'subscribing to /camera/image_raw'
        )

    def image_callback(self, msg):
        """
        Called every time a new camera frame arrives.

        Flow:
          1. CvBridge: Image msg → NumPy (BGR)
          2. Convert BGR → RGB (model expects RGB)
          3. Preprocess: normalize for ImageNet
          4. Run DeepLabV3 → class ID per pixel
          5. Publish colored overlay (for visualization)
          6. Publish raw seg map (for costmap building)
        """
        # ─── Step 1: Convert ROS2 Image to NumPy ─────────
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # WHY bgr8: OpenCV default, DeepLabV3 needs color
        except Exception as e:
            self.get_logger().error(f'CvBridge failed: {e}')
            return

        # ─── Step 2: BGR → RGB ────────────────────────────
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # WHY: DeepLabV3 trained on RGB data, OpenCV loads BGR
        # The Day 7 trap — still matters on Day 34

        # ─── Step 3: Preprocess ───────────────────────────
        input_tensor = self.preprocess(frame_rgb).unsqueeze(0)
        # WHY unsqueeze(0): adds batch dimension
        # Model expects (1, 3, H, W) not (3, H, W)

        # ─── Step 4: Run segmentation ─────────────────────
        with torch.no_grad():
            # WHY no_grad: inference only, skip gradient computation
            output = self.model(input_tensor)

        # Get class ID for each pixel
        seg_map = output['out'][0].argmax(dim=0).numpy().astype(np.uint8)
        # WHY argmax(dim=0): for each pixel, which of 21 classes scored highest
        # WHY uint8: class IDs are 0-20, fits in one byte
        # seg_map shape: (H, W) — same size as input image

        # ─── Step 5: Publish colored overlay ──────────────
        # Convert class IDs to colors for visualization
        seg_colored = self.COLORS[seg_map]
        # WHY fancy indexing: seg_map value 15 → COLORS[15] = person color
        # Instant, no loop needed

        # Blend with original image for context
        overlay = frame_rgb.copy()
        mask = seg_map > 0  # non-background pixels
        overlay[mask] = (0.4 * frame_rgb[mask] + 0.6 * seg_colored[mask]).astype(np.uint8)
        # WHY blend: see original image through the color tint
        # Pure segmentation colors lose visual context

        # Convert back to BGR for ROS2 Image message
        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        color_msg = self.bridge.cv2_to_imgmsg(overlay_bgr, encoding='bgr8')
        color_msg.header = msg.header  # keep original timestamp
        self.color_pub.publish(color_msg)

        # ─── Step 6: Publish raw segmentation map ─────────
        # Raw class IDs as a single-channel image
        # Other nodes read this to know what each pixel IS
        map_msg = self.bridge.cv2_to_imgmsg(seg_map, encoding='mono8')
        # WHY mono8: single channel uint8, values 0-20 (class IDs)
        # A costmap node reads pixel value:
        #   0 = background → depends on context
        #   6 = bus → obstacle (cost 254)
        #   15 = person → lethal obstacle (cost 254)
        map_msg.header = msg.header
        self.map_pub.publish(map_msg)

        # ─── Log detections ───────────────────────────────
        self.frame_count += 1
        if self.frame_count % 10 == 0:
            unique_classes = np.unique(seg_map)
            class_names = [self.CLASSES[c] for c in unique_classes if c > 0]
            self.get_logger().info(
                f'Frame {self.frame_count} | '
                f'Classes: {", ".join(class_names) if class_names else "background only"}'
            )


def main():
    rclpy.init()
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()