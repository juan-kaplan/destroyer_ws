#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('mappeate_y_ubicate')

    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(pkg_share, 'rviz', 'slam.rviz')
    )

    # Declare launch argument for num_particles
    num_particles_arg = DeclareLaunchArgument(
        "num_particles",
        default_value="50",
        description="Number of particles for the filter"
    )

    delta_odom_node = Node(
        package='mappeate_y_ubicate',
        executable='delta_odom',
        name='delta_odom_node',
        output='screen'
    )

    fast_slam_node = Node(
        package='mappeate_y_ubicate',
        executable='fastslam',
        name='fast_slam_node',
        output='screen',
        parameters=[{"num_particles": LaunchConfiguration("num_particles")}]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()

    # IMPORTANT: add the argument declaration
    ld.add_action(num_particles_arg)

    # Add nodes
    ld.add_action(delta_odom_node)
    ld.add_action(fast_slam_node)
    ld.add_action(rviz2_node)

    return ld
