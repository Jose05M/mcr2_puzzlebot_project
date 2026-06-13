import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('mcr2_puzzlebot')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([pkg, 'config', 'params.yaml']),
        description='Path to YAML parameter file',
    )

    config_file = LaunchConfiguration('config')

    detector_semaforo = Node(
        package='mcr2_puzzlebot',
        executable='traffic_light_detector',
        name='traffic_light_detector',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    seguidor_linea = Node(
        package='mcr2_puzzlebot',
        executable='line_follower',
        name='line_follower',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    img_compressor = Node(
        package='mcr2_puzzlebot',
        executable='image_compressor',
        name='image_compressor',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    controlador = Node(
        package='mcr2_puzzlebot',
        executable='line_follower_controller',
        name='line_follower_controller',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    odometria = Node(
        package='mcr2_puzzlebot',
        executable='puzzlebot_odometry',
        name='puzzlebot_odometry',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        controlador,
        odometria,
    ])
