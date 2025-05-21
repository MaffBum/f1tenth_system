from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'config.yaml'
    )

    stanley_controller = Node(
        package='pure_pursuit',
        executable='stanley_controller',
        name='stanley_controller',
        parameters=[config]
    )

    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[config]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory(
            'pure_pursuit'), 'launch', 'pure_pursuit.rviz')]
    )

    # finalize
    ld.add_action(rviz_node)
    ld.add_action(stanley_controller)
    ld.add_action(waypoint_visualizer_node)

    return ld
