
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from uuv_interfaces.srv import ResumeDrift, PauseDrift

from vista_sim.simple_vehicle_sim_v3 import Eta, Nu

class DriftService(Node):
    
    def __init__(self):
        super().__init__('drift_service')
        
        #Parameters 
        self.declare_parameter("frame_id", "ned")
        self.declare_parameter("time_step", 0.1)
        
        #idle constant velocity
        self.declare_parameter("constant_velocity", 0.1)

        self.frame_id = self.get_parameter("frame_id").value
        self.dt = self.get_parameter("time_step").value
        self.constant_velocity = self.get_parameter("constant_velocity").value
        
        
        self.pause_drift = self.create_service(PauseDrift, 'pause_drift', self.pause_drift_cb)
        self.resume_drift = self.create_service(ResumeDrift,'resume_drift', self.resume_drift_cb)
        
        
        self._eta = Eta(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._nu = Nu(self.constant_velocity, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._robot_actuated = False
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self._broadcast_origin_tf()
    
    
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
    
    def pause_drift_cb(self, request, response):
        self._robot_actuated = True
        
        response.eta = list(self._eta)
        response.nu = list(self._nu)
        response.success = True
        return response
    
    def resume_drift_cb(self, request, response):
        self._robot_actuated = False
        
        # * is positional arguments
        self._eta = Eta(*request.eta)
        self._nu = Nu(*request.nu)

        response.success = True
        return response

    def propagate_and_broadcast(self):
        eta = self._eta
        self._eta = Eta(
            north=eta.north + self._nu.surge_vel * math.cos(eta.yaw) * self.dt,
            east=eta.east + self._nu.surge_vel * math.sin(eta.yaw) * self.dt,
            depth=eta.depth,
            roll=0.0,
            pitch=0.0,
            yaw=eta.yaw,
        )
        pose_msg = self.eta_to_pose_stamped(self._eta)
        self.broadcast_tf(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriftService()
    node.get_logger().info("Drift service running.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            if not node._robot_actuated:
                node.propagate_and_broadcast()
            time.sleep(node.dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
