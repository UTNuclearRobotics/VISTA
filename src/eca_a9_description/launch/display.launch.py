from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('urdf_launch'), 'launch', 'display.launch.py'
                ])
            ),
            launch_arguments={
                'urdf_package': 'eca_a9_description',
                'urdf_package_path': PathJoinSubstitution(['urdf', 'eca_a9_visual.urdf.xacro']),
                'jsp_gui': 'true'
            }.items()
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '1', '--y', '1', '--z', '1',
                '--yaw', '1.57079632679', '--pitch', '0', '--roll',
                '3.14159265359', '--frame-id', 'map', '--child-frame-id', 'ned']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ned_to_base_link_static',
            arguments=['0','0','0', '0','0','0', 'ned','base_link']
        ),
        
    ])
