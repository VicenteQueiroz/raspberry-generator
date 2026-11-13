"""
Launch a generated raspberry field in Gazebo Harmonic (ROS 2 Jazzy).

    ros2 launch raspberry_field field.launch.py world_dir:=/path/to/generated/raspberry_field

Arguments:
    world_dir      directory produced by generate_field.py (contains <world>.sdf, models, markers.json)
    world          world file name inside world_dir (default: raspberry_field.sdf)
    ground_truth   start the RViz ground-truth publisher (default: true)
    gz_args        extra arguments for `gz sim` (default: -r -v 3)
"""
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_dir = LaunchConfiguration('world_dir')
    world = LaunchConfiguration('world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [LaunchConfiguration('gz_args'), ' ', PathJoinSubstitution([world_dir, world])],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')

    ground_truth = Node(
        package='raspberry_field', executable='ground_truth.py', name='raspberry_ground_truth',
        parameters=[{'world_dir': world_dir, 'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('ground_truth')), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('world_dir', description='generated world directory'),
        DeclareLaunchArgument('world', default_value='raspberry_field.sdf'),
        DeclareLaunchArgument('ground_truth', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value='-r -v 3'),
        # models are referenced as model://raspberry_N, so the world dir must be a resource path
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', world_dir),
        gz_sim,
        clock_bridge,
        ground_truth,
    ])
