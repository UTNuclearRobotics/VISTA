#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

from sensor_model.ray_casting_core import RayCastingCore   # full package path
from sensor_model.compute_intrinsics import FLS_CONFIG


def transform_to_extrinsic(t):
    """Convert ROS TransformStamped to camera extrinsic matrix.

    Args:
        t: TransformStamped message (map to sonar, i.e., world-to-camera)

    Returns:
        4x4 numpy array (world to camera transform)
    """
    q = t.transform.rotation
    extrinsic = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    extrinsic[:3, 3] = [t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z]
    return extrinsic.astype(np.float64)


class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize ray caster
        self.fls_sensor = RayCastingCore.from_config(FLS_CONFIG)
        self.fls_sensor.create_monkey_scene()                     # test geometry
        # Rays will be set dynamically with transform in timer callback

        # Create publishers
        self.depth_pub = self.create_publisher(Image,
                                         '/sonar/depth/image_raw',
                                         10)

        self.pub_info = self.create_publisher(CameraInfo, '/sonar/depth/camera_info', 10)

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info("DepthPublisher started.")

    def timer_cb(self):
        stamp = self.get_clock().now().to_msg()

        # Lookup world-to-camera transform (sonar_optical relative to map) for Open3D extrinsic
        try:
            transform = self.tf_buffer.lookup_transform(
                'sonar_optical',
                'map',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')
            return

        # Convert to extrinsic matrix and set rays
        extrinsic = transform_to_extrinsic(transform)
        self.fls_sensor.set_rays(extrinsic)

        # Get depth array (H, W)
        depth = self.fls_sensor.ping().astype(np.float32)

        # Package into ROS Image 
        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "sonar_optical"
        img_msg.height = depth.shape[0]
        img_msg.width = depth.shape[1]
        img_msg.encoding = "32FC1"
        img_msg.is_bigendian = 0
        img_msg.step = depth.shape[1] * 4
        img_msg.data = depth.tobytes()

        # CameraInfo
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = "sonar_optical"
        info_msg.height = self.fls_sensor.height_px
        info_msg.width = self.fls_sensor.width_px
        
        # Intrinsic matrix K (3x3)
        info_msg.k = [
            self.fls_sensor.fx, 0.0,          self.fls_sensor.cx,
            0.0,          self.fls_sensor.fy, self.fls_sensor.cy,
            0.0,          0.0,          1.0
        ]
        
        # Projection matrix P (3x4) - required for DepthCloud
        info_msg.p = [
            self.fls_sensor.fx, 0.0,          self.fls_sensor.cx, 0.0,
            0.0,          self.fls_sensor.fy, self.fls_sensor.cy, 0.0,
            0.0,          0.0,          1.0,                   0.0
        ]
        
        # Rotation matrix R (3x3) - identity for single camera
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Distortion model and coefficients - plumb_bob with no distortion
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # [k1, k2, t1, t2, k3]

        # Publish both
        self.depth_pub.publish(img_msg)
        self.pub_info.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
