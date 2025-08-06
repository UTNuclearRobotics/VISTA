from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    """Launch the vehicle simulation with action server and client"""

    # Launch arguments
    start_rviz = LaunchConfiguration('start_rviz')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_z = LaunchConfiguration('goal_z')
    goal_yaw = LaunchConfiguration('goal_yaw')
    constant_velocity = LaunchConfiguration('constant_velocity')
    max_runtime = LaunchConfiguration('max_runtime')
    time_step = LaunchConfiguration('time_step')
    
    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('vista_sim'),
        'config',
        'vista_sim.rviz'
    ])
    
    # Process the URDF file using ParameterValue
    package_dir = FindPackageShare('eca_a9_description')
    urdf_path = PathJoinSubstitution([package_dir, 'urdf', 'eca_a9_visual.urdf.xacro'])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    return LaunchDescription([
        # Launch argument declarations
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 with the simulation'
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='50.0',
            description='X-coordinate (North) of the goal position'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='50.0',
            description='Y-coordinate (East) of the goal position'
        ),
        DeclareLaunchArgument(
            'goal_z',
            default_value='0.0',
            description='Z-coordinate (Depth) of the goal position'
        ),
        DeclareLaunchArgument(
            'goal_yaw',
            default_value='1.57',
            description='Yaw angle of the goal orientation in radians'
        ),
        DeclareLaunchArgument(
            'constant_velocity',
            default_value='0.5',
            description='Constant velocity of the vehicle in m/s'
        ),
        DeclareLaunchArgument(
            'max_runtime',
            default_value='300.0',
            description='Maximum runtime for the simulation in seconds'
        ),
        DeclareLaunchArgument(
            'time_step',
            default_value='0.1',
            description='Simulation time step in seconds'
        ),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_ned_static',
            arguments=['--x', '10', '--y', '10', '--z', '10', '--yaw', '1.57079632679', '--pitch', '0', '--roll', '3.14159265359', '--frame-id', 'map', '--child-frame-id', 'ned'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ned_to_base_link_static',
            arguments=['0', '0', '0', '0', '0', '0', 'ned', 'base_link'],
        ),

        # Robot State Publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content}
            ]
        ),

        # Vehicle simulation action server
        Node(
            package='vista_sim',
            executable='dubins_pose_to_pose_action_server',
            name='vehicle_sim_server',
            output='screen',
            parameters=[
                {
                    'use_ned_frame': True,
                    'frame_id': 'ned',
                    'time_step': time_step,
                    'constant_velocity': constant_velocity,
                    'max_runtime': max_runtime
                }
            ]
        ),

        # Client to send a goal pose (only start if requested)
        Node(
            package='vista_sim',
            executable='dubins_pose_to_pose_action_client',
            name='pose_client',
            output='screen',
            parameters=[
                {
                    'goal_x': goal_x,
                    'goal_y': goal_y,
                    'goal_z': goal_z,
                    'goal_roll': 0.0,
                    'goal_pitch': 0.0,
                    'goal_yaw': goal_yaw,
                    'frame_id': 'ned'
                }
            ]
        ),

        # RViz node with configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(start_rviz)
        )
    ])