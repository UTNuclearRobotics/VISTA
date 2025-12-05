#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import matplotlib.pyplot as plt

class DepthSubscriberTest(Node):
    def __init__(self):
        super().__init__('depth_subscriber_test')
        
        self.depth_msg = None
        self.info_msg = None
        
        self.sub_depth = self.create_subscription(
            Image,
            '/sonar/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/sonar/depth/camera_info',
            self.info_callback,
            10
        )

    def depth_callback(self, msg):
        if self.depth_msg is None:
            self.depth_msg = msg
            self.get_logger().info(f"Depth: {msg.height}x{msg.width}, encoding: {msg.encoding}")
            self.check_and_visualize()

    def info_callback(self, msg):
        if self.info_msg is None:
            self.info_msg = msg
            self.get_logger().info(f"CameraInfo: {msg.height}x{msg.width}")
            self.get_logger().info(f"  fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}")
            self.get_logger().info(f"  cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}")
            self.check_and_visualize()

    def check_and_visualize(self):
        if self.depth_msg is None or self.info_msg is None:
            return
        
        # Convert depth to numpy
        depth = np.frombuffer(self.depth_msg.data, dtype=np.float32).reshape(
            self.depth_msg.height, self.depth_msg.width
        )
        
        # Check dimensions match
        if self.depth_msg.height != self.info_msg.height or self.depth_msg.width != self.info_msg.width:
            self.get_logger().error("Dimension mismatch between Image and CameraInfo!")
        else:
            self.get_logger().info("✓ Dimensions match")
        
        # Depth stats
        valid_depth = depth[np.isfinite(depth)]
        self.get_logger().info(f"Depth range: {np.min(valid_depth):.3f} - {np.max(valid_depth):.3f}")
        
        # Visualize
        plt.imshow(depth)
        plt.colorbar(label='Depth')
        plt.title(f'Depth ({self.info_msg.width}x{self.info_msg.height}, fx={self.info_msg.k[0]:.1f})')
        plt.show()
        
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriberTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()