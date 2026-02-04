#!/usr/bin/env python3
"""Interactive marker to control sonar pose for testing transforms.

Broadcasts map -> sonar TF. Combined with the static sonar -> sonar_optical
transform, this completes the chain for sensor_publisher.

Usage:
    ros2 run sensor_model pose_marker_test

In RViz:
    - Add InteractiveMarkers display (topic: /sonar_marker/update)
    - Drag the marker to move the sonar frame
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from interactive_markers import InteractiveMarkerServer
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class SonarInteractiveMarker(Node):
    def __init__(self):
        super().__init__('sonar_interactive_marker')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.server = InteractiveMarkerServer(self, 'sonar_marker')

        self._create_marker()
        self.server.applyChanges()

        # Publish TF at 50Hz even when not dragging
        self.current_pose = None
        self.timer = self.create_timer(0.02, self._broadcast_tf)
        self.get_logger().info('Sonar interactive marker ready — drag in RViz to move sonar frame')

    def _create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.name = 'sonar_control'
        int_marker.description = 'Sonar Pose'
        int_marker.scale = 0.3

        # Initial pose — position to view Suzanne (centered at origin)
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = -2.0
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.w = 1.0

        # Visual: box body + cone pointing forward (+X in sonar frame)
        visual_ctrl = InteractiveMarkerControl()
        visual_ctrl.always_visible = True

        # Box body
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = box.scale.z = 0.15
        box.color.r, box.color.g, box.color.b, box.color.a = 0.0, 0.8, 0.8, 1.0
        visual_ctrl.markers.append(box)

        # Small sphere at front to indicate +X direction
        nose = Marker()
        nose.type = Marker.SPHERE
        nose.scale.x = nose.scale.y = nose.scale.z = 0.06
        nose.color.r, nose.color.g, nose.color.b, nose.color.a = 1.0, 0.4, 0.7, 1.0
        nose.pose.position.x = 0.12
        nose.pose.orientation.w = 1.0
        visual_ctrl.markers.append(nose)

        int_marker.controls.append(visual_ctrl)

        # 6-DOF controls (rotation rings + translation arrows)
        orientations = [
            ('x', 1.0, 1.0, 0.0, 0.0),
            ('y', 1.0, 0.0, 0.0, 1.0),
            ('z', 1.0, 0.0, 1.0, 0.0),
        ]
        for axis, w, x, y, z in orientations:
            # Rotation ring
            ctrl = InteractiveMarkerControl()
            ctrl.orientation.w, ctrl.orientation.x, ctrl.orientation.y, ctrl.orientation.z = w, x, y, z
            ctrl.name = f'rotate_{axis}'
            ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(ctrl)

            # Translation arrow
            ctrl = InteractiveMarkerControl()
            ctrl.orientation.w, ctrl.orientation.x, ctrl.orientation.y, ctrl.orientation.z = w, x, y, z
            ctrl.name = f'move_{axis}'
            ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(ctrl)

        self.server.insert(int_marker, feedback_callback=self._on_feedback)

        # Set initial pose for TF broadcast
        self.current_pose = int_marker.pose

    def _on_feedback(self, feedback):
        self.current_pose = feedback.pose

    def _broadcast_tf(self):
        if self.current_pose is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'sonar'
        t.transform.translation.x = self.current_pose.position.x
        t.transform.translation.y = self.current_pose.position.y
        t.transform.translation.z = self.current_pose.position.z
        t.transform.rotation = self.current_pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SonarInteractiveMarker()
    rclpy.spin(node)
    node.server.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()