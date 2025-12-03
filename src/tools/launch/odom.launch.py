from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    delta_odom_node = Node(
        package="tools",
        executable="delta_odom",
        name="delta_odom_node",
        output="screen",
    )

    return LaunchDescription([delta_odom_node])
