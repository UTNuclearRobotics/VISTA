from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import Command


def generate_launch_description():
    """Launch the vehicle simulation with action server, client, and sensor model"""

    # Sensor model environment config
    sensor_param_config = PathJoinSubstitution([
        FindPackageShare('sensor_model'), 'config', 'environment_basic.yaml'
    ])

    # Launch arguments
    start_rviz = LaunchConfiguration('start_rviz')
    drift_velocity = LaunchConfiguration('drift_velocity')       # idle drift speed
    constant_velocity = LaunchConfiguration('constant_velocity')  # action server navigation speed
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
            'drift_velocity',
            default_value='0.25',
            description='Idle drift velocity when not navigating (m/s)'
        ),
        DeclareLaunchArgument(
            'constant_velocity',
            default_value='0.5',
            description='Navigation velocity for Dubins path following (m/s)'
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

        # Robot State Publisher 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content}
            ]
        ),

        # Drift service — propagates vehicle at idle drift velocity,
        # exposes pause_drift / resume_drift services for action server handoff
        Node(
            package='vista_sim',
            executable='drift_service',
            name='drift_service',
            output='screen',
            parameters=[
                {
                    'frame_id': 'ned',
                    'time_step': time_step,
                    'constant_velocity': drift_velocity,
                }
            ]
        ),

        # Action server — plans and follows Dubins paths at navigation velocity,
        # pauses drift on goal receipt, resumes drift on completion
        Node(
            package='vista_sim',
            executable='dubins_pose_to_pose_action_server',
            name='vehicle_sim_server',
            output='screen',
            parameters=[
                {
                    'frame_id': 'ned',
                    'time_step': time_step,
                    'constant_velocity': constant_velocity,
                    'max_runtime': max_runtime
                }
            ]
        ),

        # Sensor model: sonar → sonar_optical frame convention
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sonar_to_optical_static',
            arguments=['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                       '--frame-id', 'sonar', '--child-frame-id', 'sonar_optical'],
        ),

        # Sensor model: scene mesh visualisation
        Node(
            package='sensor_model',
            executable='meshes_rviz',
            name='meshes_rviz',
            parameters=[sensor_param_config],
        ),

        # Sensor model: FLS depth image publisher
        Node(
            package='sensor_model',
            executable='sensor_publisher',
            name='depth_publisher',
            parameters=[sensor_param_config],
        ),

        # RViz node with configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)
        )
    ])