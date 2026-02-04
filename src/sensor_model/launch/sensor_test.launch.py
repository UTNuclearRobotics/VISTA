from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('sensor_model'), 'rviz', 'sensor_test.rviz'
    ])

    return LaunchDescription([
        # Fixed rotation: sonar (X-fwd, Y-left, Z-up) → sonar_optical (X-right, Y-down, Z-fwd)
        # Matches eca_a9_visual.urdf.xacro sonar_to_optical joint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sonar_to_optical_static',
            arguments=['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                       '--frame-id', 'sonar', '--child-frame-id', 'sonar_optical'],
        ),
        Node(
            package='sensor_model',
            executable='suzanne_rviz',
            name='suzanne_rviz',
        ),
        Node(
            package='sensor_model',
            executable='sensor_publisher',
            name='depth_publisher',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
