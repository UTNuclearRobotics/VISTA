import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import open3d as o3d
import numpy as np
from ament_index_python.packages import get_package_share_directory


class MeshesPublisher(Node):
    def __init__(self):
        super().__init__('meshes_rviz')

        self.pub = self.create_publisher(MarkerArray, '/environment_meshes', 1)

        self.markers = MarkerArray()

        # Load box count first so we can dynamically declare the per-box position params
        self.declare_parameter('box_count', rclpy.Parameter.Type.INTEGER)
        box_count = self.get_parameter('box_count').value

        # Declare box_size, per-box positions, and terrain path
        box_params = [
            ('box_size', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ('terrain.file_path', rclpy.Parameter.Type.STRING),
        ] + [(f'box_positions.box_{i}', rclpy.Parameter.Type.DOUBLE_ARRAY) for i in range(box_count)]
        self.declare_parameters(namespace='', parameters=box_params)

        box_size = self.get_parameter('box_size').value

        # Terrain TRIANGLE_LIST marker loaded from mesh file
        terrain_rel = self.get_parameter('terrain.file_path').value
        terrain_path = os.path.join(get_package_share_directory('sensor_model'), terrain_rel)

        mesh = o3d.io.read_triangle_mesh(terrain_path)
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)

        terrain = Marker()
        terrain.header.frame_id = 'map'
        terrain.ns = 'terrain'
        terrain.id = 0
        terrain.type = Marker.TRIANGLE_LIST
        terrain.action = Marker.ADD
        terrain.scale.x = 1.0
        terrain.scale.y = 1.0
        terrain.scale.z = 1.0
        terrain.color = ColorRGBA(r=0.4, g=0.4, b=0.4, a=1.0)
        terrain.pose.orientation.w = 1.0

        for tri in triangles:
            for vi in tri:
                v = vertices[vi]
                terrain.points.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))

        self.markers.markers.append(terrain)

        # Box CUBE markers — offset centroid by half size to match Open3D corner convention
        for i in range(box_count):
            pos = self.get_parameter(f'box_positions.box_{i}').value
            cube = Marker()
            cube.header.frame_id = 'map'
            cube.ns = 'boxes'
            cube.id = i + 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(pos[0]) + float(box_size[0]) / 2.0
            cube.pose.position.y = float(pos[1]) + float(box_size[1]) / 2.0
            cube.pose.position.z = float(pos[2]) + float(box_size[2]) / 2.0
            cube.pose.orientation.w = 1.0
            cube.scale.x = float(box_size[0])
            cube.scale.y = float(box_size[1])
            cube.scale.z = float(box_size[2])
            cube.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.8)
            self.markers.markers.append(cube)

        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f'Publishing terrain + {box_count} boxes on /environment_meshes at 1 Hz')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for m in self.markers.markers:
            m.header.stamp = stamp
        self.pub.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    node = MeshesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
