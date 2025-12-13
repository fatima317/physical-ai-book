#!/usr/bin/env python3

"""
Launch file for JetBot simulation in Gazebo
This launch file starts Gazebo with the custom world and spawns the JetBot robot
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Get paths
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare('jetbot_description').find('jetbot_description')  # Assuming this package exists

    # For our example, we'll use the path relative to this file
    examples_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to the world file
    world_path = os.path.join(examples_dir, 'jetbot_world.world')

    # Path to the robot URDF
    robot_description_path = os.path.join(examples_dir, 'jetbot.urdf.xacro')

    # Process the xacro file to get the robot description
    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)
    robot_description_config = doc.toprettyxml(indent='  ')

    # Launch Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # Launch Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=lambda context: True  # Always launch the client
    )

    # Publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'jetbot',
            '-x', '0', '-y', '0', '-z', '0.1'  # Spawn at slight height to avoid ground collision
        ],
        output='screen'
    )

    # Joint state publisher (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': False,
            'source_list': ['joint_states']
        }]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_entity)

    return ld