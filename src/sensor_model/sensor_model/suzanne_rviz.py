"""Publish Suzanne as a Marker in the map frame so you can inspect her
default orientation in RViz.

Usage:
    ros2 run sensor_model suzanne_rviz
    # or just: python3 suzanne_rviz.py

Then in RViz:
    - Set Fixed Frame to "map"
    - Add a Marker display (topic: /suzanne)
    - Add a TF or Axes display to see frame orientation
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import open3d as o3d
import numpy as np


class SuzannePublisher(Node):
    def __init__(self):
        super().__init__('suzanne_rviz')

        self.pub = self.create_publisher(Marker, '/suzanne', 1)

        # Load mesh
        monkey_data = o3d.data.MonkeyModel()
        mesh = o3d.io.read_triangle_mesh(monkey_data.path)
        mesh.compute_vertex_normals()

        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)

        self.get_logger().info(
            f"Suzanne: {len(vertices)} verts, {len(triangles)} tris, "
            f"center={mesh.get_center()}, "
            f"min={mesh.get_min_bound()}, max={mesh.get_max_bound()}"
        )

        # Build marker once
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'suzanne'
        self.marker.id = 0
        self.marker.type = Marker.TRIANGLE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)

        # Identity pose — no transform applied, raw mesh coordinates
        self.marker.pose.orientation.w = 1.0

        for tri in triangles:
            for vi in tri:
                v = vertices[vi]
                self.marker.points.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))

        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info('Publishing Suzanne on /suzanne at 1 Hz — open RViz and add a Marker display')

    def publish(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = SuzannePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()