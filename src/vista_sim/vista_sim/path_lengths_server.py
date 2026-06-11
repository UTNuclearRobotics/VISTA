import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, TransformStamped
import tf2_ros
import tf_transformations
from uuv_interfaces.srv import DubinsLengths

from vista_sim.simple_vehicle_sim_v3 import (
    Eta,
    DubinsAirplanePath,
)


class PathLengthsService(Node):

    def __init__(self):
        super().__init__('path_length_service')
        self.srv = self.create_service(DubinsLengths, 'compute_path_lengths', self.length_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _matrix_from_translation_rotation(self, translation, rotation) -> np.ndarray:
        """4x4 homogeneous transform from Vector3 translation + Quaternion rotation."""
        matrix = tf_transformations.quaternion_matrix(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return matrix

    def pose_to_matrix(self, pose: Pose) -> np.ndarray:
        """4x4 homogeneous transform from a Pose."""
        return self._matrix_from_translation_rotation(pose.position, pose.orientation)

    def transform_to_matrix(self, tf: TransformStamped) -> np.ndarray:
        """4x4 homogeneous transform from a TransformStamped."""
        return self._matrix_from_translation_rotation(tf.transform.translation, tf.transform.rotation)

    def matrix_to_eta(self, matrix: np.ndarray) -> Eta:
        """Eta from a 4x4 homogeneous transform expressed in NED."""
        roll, pitch, yaw = tf_transformations.euler_from_matrix(matrix)
        north, east, depth = matrix[:3, 3]
        return Eta(north=north, east=east, depth=depth, roll=roll, pitch=pitch, yaw=yaw)

    def length_callback(self, request, response):
        # All required transforms are static/global for the whole candidate
        # set (view_frame->planning_frame mount, roi_frame->ned, current
        # vehicle pose), so look them up once up front. If any is missing,
        # every candidate is equally unreachable.
        try:
            t_ned_robot = self.tf_buffer.lookup_transform('ned', request.robot_frame_id, Time())
            t_view_planning = self.tf_buffer.lookup_transform(
                request.view_frame_id, request.robot_frame_id, Time()
            )
            t_ned_roi = self.tf_buffer.lookup_transform(
                'ned', request.nbv_poses.header.frame_id, Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Required transform unavailable: {ex}")
            # inf naturally heavily penalizes every candidate without
            # special-casing on the client side.
            response.path_lengths = [math.inf] * len(request.nbv_poses.poses)
            return response

        start = self.matrix_to_eta(self.transform_to_matrix(t_ned_robot))
        T_view_planning = self.transform_to_matrix(t_view_planning)
        T_ned_roi = self.transform_to_matrix(t_ned_roi)

        path_planner = DubinsAirplanePath(request.turn_radius, request.max_pitch_deg)

        path_lengths = []
        for pose in request.nbv_poses.poses:
            # Candidate pose (view_frame relative to roi_frame), right-multiplied
            # by the static mount offset to get planning_frame relative to
            # roi_frame, then left-multiplied into ned.
            T_roi_view = self.pose_to_matrix(pose)
            T_ned_planning = T_ned_roi @ T_roi_view @ T_view_planning
            goal = self.matrix_to_eta(T_ned_planning)
            path_lengths.append(path_planner.path_length_3d(start, goal))

        response.path_lengths = path_lengths
        return response


def main():
    rclpy.init()

    path_lengths_service = PathLengthsService()

    rclpy.spin(path_lengths_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()