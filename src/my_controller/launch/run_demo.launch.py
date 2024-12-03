from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    my_node = Node(
        package='my_controller',
        executable='robot_commander'
    )

    ld.add_action(my_node)

    return ld
