"""
Author: Talal Ayman
ROS 2 Action Server that drives a simple Dubins‑based vehicle simulator
from its current pose to a goal pose.

Key features
------------
*   No changes to the Dubins planner or vehicle dynamics classes.
*   TF broadcast of `base_link` for RViz visualisation.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from uuv_interfaces.action import PoseToPose



from vista_sim.simple_vehicle_sim_v2 import (
    Eta,
    Nu,
    DubinsAirplanePath,
    SimpleVehicleModel,
)


class VehiclePoseActionServer(Node):
    """Synchronous Pose-to-Pose action server using Dubins planning."""

    def __init__(self):
        super().__init__("vehicle_pose_action_server")

        # Parameters
        self.declare_parameter("use_ned_frame", True)
        self.declare_parameter("frame_id", "ned")
        self.declare_parameter("time_step", 0.1)
        self.declare_parameter("constant_velocity", 0.5)
        self.declare_parameter("max_runtime", 300.0)

        self.use_ned_frame = self.get_parameter("use_ned_frame").value
        self.frame_id = self.get_parameter("frame_id").value
        self.dt = self.get_parameter("time_step").value
        self.constant_velocity = self.get_parameter("constant_velocity").value
        self.max_runtime = self.get_parameter("max_runtime").value

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Publisher for planned path visualization

        # Action Server
        self._action_server = ActionServer(
            self,
            PoseToPose,
            "pose_to_pose",
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            execute_callback=self.execute_cb,
        )

        self.get_logger().info("Vehicle Pose Action Server ready")

    # Note: goal_cb and cancel_cb are required by ActionServer, but currently contain no custom logic.
    # They always accept every goal and cancel request.
    # You can add logic here to reject concurrent goals, preempt, or restrict cancellation if needed.

    def goal_cb(self, goal_request):
        """Handle incoming goal requests.

        Currently, this callback always accepts incoming goals.
        If you want to support goal preemption or reject concurrent goals,
        add logic here.
        """
        self.get_logger().info("Goal accepted.")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        """Handle goal cancellation requests.

        Currently, this callback always accepts cancellation requests.
        Add logic here if you want to restrict when goals can be canceled.
        """
        self.get_logger().info("Cancel request received – accepted.")
        return CancelResponse.ACCEPT

    def pose_stamped_to_eta(self, pose: PoseStamped) -> Eta:
        """Convert PoseStamped to Eta."""
        qx, qy, qz, qw = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return Eta(
            north=pose.pose.position.x,
            east=pose.pose.position.y,
            depth=pose.pose.position.z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )

    def eta_to_pose_stamped(self, eta: Eta) -> PoseStamped:
        """Convert Eta to PoseStamped."""
        x, y, z, w = quaternion_from_euler(eta.roll, eta.pitch, eta.yaw)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = eta.north
        pose.pose.position.y = eta.east
        pose.pose.position.z = eta.depth
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def broadcast_tf(self, pose: PoseStamped):
        """Broadcast TF for visualization."""
        t = TransformStamped()
        t.header.stamp = pose.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = "base_link"
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
    

    def execute_cb(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info("Starting execution loop…")

        # Extract goal
        goal_eta = self.pose_stamped_to_eta(goal_handle.request.goal_pose)
        start_eta = Eta(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        start_nu = Nu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Plan path
        planner = DubinsAirplanePath(turn_radius=2.0, max_pitch_deg=15.0)
        waypoints = planner.get_poses(start_eta, goal_eta, waypoint_spacing=1.0)
        self.get_logger().info(f"Generated Dubins path with {len(waypoints)} waypoints")


        # Vehicle model
        vehicle = SimpleVehicleModel(
            {
                "speed_time_constant": 2.0,
                "yaw_time_constant": 1.0,
                "pitch_time_constant": 1.5,
                "roll_ratio": 0.2,
                "turn_radius_m": 2.0,
                "max_acceleration_mps2": 1.0,
                "max_speed_mps": self.constant_velocity,
                "max_pitch_deg": 15.0,
                "pitch_proportional_gain": 0.5,
                "look_ahead_dist": 1.0,
                "nominal_speed": self.constant_velocity,
            },
            waypoints,
        )

        # Execution loop
        eta = start_eta
        nu = start_nu
        feedback = PoseToPose.Feedback()
        t_start = time.time()

        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled by client.")
                return PoseToPose.Result(result_message="Canceled", distance_to_goal=-1.0)

            # Simulation step
            control = vehicle.calc_control_input(eta)
            eta, nu = vehicle.step(eta, nu, control, self.dt)

            # Publish feedback
            pose_msg = self.eta_to_pose_stamped(eta)
            self.broadcast_tf(pose_msg)
            dist = math.sqrt(
                (eta.north - goal_eta.north) ** 2
                + (eta.east - goal_eta.east) ** 2
                + (eta.depth - goal_eta.depth) ** 2
            )
            feedback.current_pose = pose_msg
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            # Check if goal is reached
            if dist < 0.3:
                goal_handle.succeed()
                self.get_logger().info("Goal reached ")
                return PoseToPose.Result(result_message="Succeeded", distance_to_goal=dist)

            # Check for timeout
            if time.time() - t_start > self.max_runtime:
                goal_handle.abort()
                self.get_logger().warning("Max runtime exceeded – aborting")
                return PoseToPose.Result(result_message="Aborted – timeout", distance_to_goal=dist)

            time.sleep(self.dt)

    def destroy_node(self):
        """Clean up resources."""
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehiclePoseActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()