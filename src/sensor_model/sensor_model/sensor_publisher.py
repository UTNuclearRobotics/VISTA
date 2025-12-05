#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

from sensor_model.ray_casting_core import RayCastingCore   # full package path
from sensor_model.compute_intrinsics import FLS_CONFIG


class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')

        #initialize ray caster 
        self.fls_sensor = RayCastingCore.from_config(FLS_CONFIG)
        self.fls_sensor.create_box_scene()                       # test geometry
        self.fls_sensor.set_rays(camera_pos=[2.0, 0.0, 1.0])     # static camera for now

        #create publisher 
        self.depth_pub = self.create_publisher(Image,
                                         '/sonar/depth/image_raw',
                                         10)
        
        self.pub_info = self.create_publisher(CameraInfo, '/sonar/depth/camera_info', 10)

        #publish at 10 Hz 
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info("DepthPublisher started.")

    def timer_cb(self):
        stamp = self.get_clock().now().to_msg()

        # Get depth array (H, W) 
        depth = self.fls_sensor.ping().astype(np.float32)

        # Package into ROS Image 
        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "sonar"
        img_msg.height = depth.shape[0]
        img_msg.width = depth.shape[1]
        img_msg.encoding = "32FC1"
        img_msg.is_bigendian = 0
        img_msg.step = depth.shape[1] * 4
        img_msg.data = depth.tobytes()

        # CameraInfo (minimal for NBV)
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = "sonar"
        info_msg.height = self.fls_sensor.height_px
        info_msg.width = self.fls_sensor.width_px
        info_msg.k = [
            self.fls_sensor.fx, 0.0,          self.fls_sensor.cx,
            0.0,          self.fls_sensor.fy, self.fls_sensor.cy,
            0.0,          0.0,          1.0
        ]

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
