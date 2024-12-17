"""
Spawn Robot Description
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    test_robot_description_share = FindPackageShare(package='assignment2_exprob').find('assignment2_exprob')
   
    # default_model_path = os.path.join(test_robot_description_share, 'urdf/robot4.xacro')
    # default_world_path = os.path.join(test_robot_description_share, 'worlds/aruco_test.world')
    
    default_world_path = os.path.join( test_robot_description_share, 'worlds/assignment2.world')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')

    # # Find the package where the URDF is located
    # robot_urdf_share = FindPackageShare(package='robot_urdf').find('robot_urdf')

    # # Now you can use the path to the URDF file in the 'robot_urdf' package
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot4.xacro')

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', LaunchConfiguration('model')])}]
    )

    '''

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    '''
    
    
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_test_robot',
                                   '-topic', '/robot_description',
                                   '-x', '1.0',  # Change X-coordinate
                                   '-y', '2.0',  # Change Y-coordinate
                                   '-z', '0.5',  # Change Z-coordinate
                                   '-R', '0.0',  # Roll
                                   '-P', '0.0',  # Pitch
                                   '-Y', '1.57'  # Yaw (e.g., 90 degrees in radians)
                                   ],
                        output='screen')


    '''
    camera01_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        output="screen",
        arguments=["joint_camera_controller"],
    )
    '''
    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        #joint_state_publisher_node,
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        # ExecuteProcess(
        #     cmd=['rviz2', '-d', rviz_config_path],
        #     output='screen'),
        spawn_entity
        #camera01_controller
    ])
