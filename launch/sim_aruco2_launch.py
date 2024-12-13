import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('paused', default_value='true', description='Whether the simulation should be paused'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('extra_gazebo_args', default_value='', description='Extra arguments for Gazebo'),
        DeclareLaunchArgument('gui', default_value='true', description='Whether to start GUI'),
        DeclareLaunchArgument('recording', default_value='false', description='Whether to record the simulation'),
        DeclareLaunchArgument('world', default_value='assignment2', description='World to load in Gazebo'),
        DeclareLaunchArgument('headless', default_value='false', description='Whether to run Gazebo headlessly'),
        DeclareLaunchArgument('debug', default_value='false', description='Whether to run in debug mode'),
        DeclareLaunchArgument('physics', default_value='ode', description='Physics engine to use'),
        DeclareLaunchArgument('verbose', default_value='false', description='Enable verbose logging'),
        DeclareLaunchArgument('world_name', default_value='$(find assignment2_exprob)/worlds/$(arg world).world', description='Path to the world file'),
        DeclareLaunchArgument('respawn_gazebo', default_value='false', description='Whether to respawn Gazebo server'),
        DeclareLaunchArgument('use_clock_frequency', default_value='false', description='Whether to use clock frequency'),
        DeclareLaunchArgument('pub_clock_frequency', default_value='100', description='Clock publishing frequency'),

        # Set the '/use_sim_time' parameter
        Node(
            package='rosparam',
            executable='param',
            name='use_sim_time_param',
            parameters=[{'/use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # Start gazebo server (gzserver)
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo',
            output='screen',
            respawn=LaunchConfiguration('respawn_gazebo'),
            arguments=[
                '-e', LaunchConfiguration('physics'),
                LaunchConfiguration('extra_gazebo_args'),
                '-r' if LaunchConfiguration('recording') == 'true' else '',
                '--verbose' if LaunchConfiguration('verbose') == 'true' else '',
                '-u' if LaunchConfiguration('paused') == 'true' else '',
                LaunchConfiguration('world_name')
            ]
        ),

        # Start gazebo client (gzclient)
        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_gui',
            output='screen',
            respawn=False,
            condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
        )
    ])
