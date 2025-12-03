import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = "grid_fastslam"
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Configuration for RViz
    # Points to: install/grid_fastslam/share/grid_fastslam/rviz/fastslam.rviz
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "fastslam.rviz")

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=default_rviz_config_path,
        description="Absolute path to rviz config file",
    )

    # 2. Configuration for SLAM
    num_particles_arg = DeclareLaunchArgument(
        "num_particles",
        default_value="50",
        description="Number of particles for the GridFastSlam filter",
    )

    # 3. Nodes
    grid_fastslam_node = Node(
        package=pkg_name,
        executable="grid_fastslam_node",
        name="grid_fastslam_node",
        output="screen",
        parameters=[{"num_particles": LaunchConfiguration("num_particles")}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [rviz_arg, num_particles_arg, grid_fastslam_node, rviz_node]
    )
