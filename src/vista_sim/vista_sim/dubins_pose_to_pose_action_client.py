"""
Author: Talal Ayman
ROS 2 Action client  that sends a goal pose to the Dubins-based vehicle simulator.
It uses the `PoseToPose` action interface to communicate with the action server.

Key features
------------
*   Simple interface for sending goal poses.
*   Feedback and result callbacks for monitoring progress.
*   Parameterized goal pose configuration.

"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from uuv_interfaces.action import PoseToPose
from tf_transformations import quaternion_from_euler


class DubinsPoseToPoseActionClient(Node):
    """
    ROS2 Action Client for sending PoseToPose goals to the action server.
    """

    def __init__(self):
        super().__init__('dubins_pose_to_pose_action_client')

        # Declare parameters for goal pose
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('frame_id', 'ned')

        # Retrieve parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_z = self.get_parameter('goal_z').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.frame_id = self.get_parameter('frame_id').value

        # Create an action client
        self._action_client = ActionClient(self, PoseToPose, 'pose_to_pose')

    def send_goal(self):
        """
        Sends a goal to the action server.
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = PoseToPose.Goal()
        goal_msg.goal_pose = self.create_goal_pose()

        # Send the goal and register callbacks
        self.get_logger().info(f'Sending goal: ({self.goal_x}, {self.goal_y}, {self.goal_z}, {self.goal_yaw})')
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def create_goal_pose(self):
        """
        Creates a PoseStamped message for the goal.
        """
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        pose.pose.position.z = self.goal_z

        # Directly use quaternion_from_euler
        q = quaternion_from_euler(0.0, 0.0, self.goal_yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def goal_response_callback(self, future):
        """
        Callback for the goal response.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for feedback from the action server.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Current pose: {feedback.current_pose}, Distance remaining: {feedback.distance_remaining:.2f}m')

    def result_callback(self, future):
        """
        Callback for the result from the action server.
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.result_message}, Final distance to goal: {result.distance_to_goal:.2f}m')
        rclpy.shutdown()


def main(args=None):
    """
    Main function to run the action client.
    """
    rclpy.init(args=args)
    action_client = DubinsPoseToPoseActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()