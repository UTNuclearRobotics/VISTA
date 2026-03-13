"""
Author: Talal Ayman
ROS 2 Action Server that drives a simple Dubins-based vehicle simulator
from its current pose to a goal pose.

Key features
------------
*   No changes to the Dubins planner or vehicle dynamics classes.
*   TF broadcast of `base_link` for RViz visualisation.
*   Blocking execute callback running in a MultiThreadedExecutor thread.
*   Client-side cancellation support with MutuallyExclusiveCallbackGroup.
*   Drift node integration planned (pause/resume service calls).
"""

import math
import time
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from uuv_interfaces.action import PoseToPose
from uuv_interfaces.srv import PauseDrift, ResumeDrift

from vista_sim.simple_vehicle_sim_v3 import (
    Eta,
    Nu,
    DubinsAirplanePath,
    SimpleVehicleModel,
)


class VehiclePoseActionServer(Node):
    """Pose-to-Pose action server using Dubins planning and a blocking execute callback."""

    def __init__(self):
        super().__init__("vehicle_pose_action_server")

        # Parameters
        self.declare_parameter("frame_id", "ned")
        self.declare_parameter("time_step", 0.1)
        self.declare_parameter("constant_velocity", 0.5)
        self.declare_parameter("max_runtime", 300.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.dt = self.get_parameter("time_step").value
        self.constant_velocity = self.get_parameter("constant_velocity").value
        self.max_runtime = self.get_parameter("max_runtime").value

        # Vehicle state — always valid, never None
        self._eta = Eta(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._nu = Nu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._cancel_requested = False

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self._broadcast_origin_tf()

        # Action Server with cancellation support
        motion_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self,
            PoseToPose,
            "pose_to_pose",
            goal_callback=self.goal_cb,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            callback_group=motion_cb_group,
        )

        # Drift service clients (ReentrantCallbackGroup so the executor can
        # process the service response while execute_cb blocks on Event.wait)
        service_cb_group = ReentrantCallbackGroup()
        self._pause_drift_cli = self.create_client(PauseDrift, 'pause_drift', callback_group=service_cb_group)
        self._resume_drift_cli = self.create_client(ResumeDrift, 'resume_drift', callback_group=service_cb_group)

        self.get_logger().info("Vehicle Pose Action Server ready")

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def goal_cb(self, goal_request):
        if not isinstance(goal_request.goal_pose, PoseStamped):
            self.get_logger().warn("Goal rejected: goal_pose is not a PoseStamped.")
            return GoalResponse.REJECT
        self.get_logger().info("Goal received – accepting.")
        return GoalResponse.ACCEPT

    def cancel_cb(self, cancel_request):
        self.get_logger().info("Cancel accepted – halting navigation.")
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Coordinate conversion helpers
    # ------------------------------------------------------------------

    def pose_stamped_to_eta(self, pose: PoseStamped) -> Eta:
        """Convert PoseStamped to Eta."""
        qx = pose.pose.orientation.x
        qy = pose.pose.orientation.y
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w

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
        """Broadcast base_link TF for RViz visualisation."""
        t = TransformStamped()
        t.header.stamp = pose.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = "base_link"
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _broadcast_origin_tf(self):
        """Broadcast an identity ned -> base_link transform at startup."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def pause_drift(self):
        event = Event()
        future = self._pause_drift_cli.call_async(PauseDrift.Request())
        future.add_done_callback(lambda _: event.set())
        event.wait()
        response = future.result()
        return Eta(*response.eta), Nu(*response.nu)

    def resume_drift(self, eta, nu):
        request = ResumeDrift.Request()
        request.eta = list(eta)
        request.nu = list(nu)
        event = Event()
        future = self._resume_drift_cli.call_async(request)
        future.add_done_callback(lambda _: event.set())
        event.wait()
    # ------------------------------------------------------------------
    # Execute callback – runs in its own thread (MultiThreadedExecutor)
    # ------------------------------------------------------------------

    def execute_cb(self, goal_handle):
        
        """Blocking execution loop: plan once, then step the vehicle model."""
        self.get_logger().info("Execution started.")

        goal_eta = self.pose_stamped_to_eta(goal_handle.request.goal_pose)

        eta, nu = self.pause_drift()


        # --- Plan path once ---
        planner = DubinsAirplanePath(turn_radius=0.5, max_pitch_deg=15.0)
        waypoints = planner.get_poses(eta, goal_eta, waypoint_spacing=1.0)
        self.get_logger().info(f"Planned Dubins path with {len(waypoints)} waypoints.")

        vehicle = SimpleVehicleModel(
            {
                "speed_time_constant": 2.0,
                "yaw_time_constant": 0.2,
                "pitch_time_constant": 1.5,
                "roll_ratio": 0.2,
                "turn_radius_m": 0.5,
                "max_acceleration_mps2": 1.0,
                "max_speed_mps": self.constant_velocity,
                "max_pitch_deg": 15.0,
                "pitch_proportional_gain": 0.5,
                "look_ahead_dist": 1.0,
                "nominal_speed": self.constant_velocity,
            },
            waypoints,
        )

        result = PoseToPose.Result()
        rate = self.create_rate(1.0 / self.dt)
        start_time = time.time()

        # --- Follow path ---
        while True:
            if self._cancel_requested:
                self._cancel_requested = False
                self._eta, self._nu = eta, nu
                goal_handle.canceled()
                self.get_logger().info("Goal canceled.")
                result.result_message = "Canceled"
                result.distance_to_goal = -1.0
                result.yaw_error_at_goal = -1.0
                self.resume_drift(eta, nu)
                return result

            control = vehicle.calc_control_input(eta)
            eta, nu = vehicle.step(eta, nu, control, self.dt)

            # Update shared state and broadcast TF
            self._eta, self._nu = eta, nu
            pose_msg = self.eta_to_pose_stamped(eta)
            self.broadcast_tf(pose_msg)

            dist = math.sqrt(
                (eta.north - goal_eta.north) ** 2
                + (eta.east - goal_eta.east) ** 2
                + (eta.depth - goal_eta.depth) ** 2
            )
            yaw_err = abs(math.atan2(
                math.sin(eta.yaw - goal_eta.yaw),
                math.cos(eta.yaw - goal_eta.yaw),
            ))

            feedback = PoseToPose.Feedback()
            feedback.current_pose = pose_msg
            feedback.distance_remaining = dist
            feedback.yaw_error_remaining = yaw_err
            goal_handle.publish_feedback(feedback)

            if dist < 1.0 and yaw_err < 0.1:
                self._eta, self._nu = eta, nu
                goal_handle.succeed()
                self.get_logger().info(
                    f"Goal reached. pos_err={dist:.3f}m  yaw_err={math.degrees(yaw_err):.2f}deg"
                )
                result.result_message = "Succeeded"
                result.distance_to_goal = dist
                result.yaw_error_at_goal = yaw_err
                self.resume_drift(eta, nu)
                return result

            if time.time() - start_time > self.max_runtime:
                self._eta, self._nu = eta, nu
                goal_handle.abort()
                self.get_logger().warning("Max runtime exceeded – aborting.")
                result.result_message = "Aborted – timeout"
                result.distance_to_goal = dist
                result.yaw_error_at_goal = yaw_err
                self.resume_drift(eta, nu)
                return result

            rate.sleep()

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehiclePoseActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
