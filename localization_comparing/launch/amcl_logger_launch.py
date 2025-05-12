from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_csv',
            default_value='amcl_log.csv',
            description='CSV file to write AMCL data to'
        ),
        Node(
            package='localization_comparing',
            executable='amcl_logger',
            name='amcl_logger',
            output='screen',
            parameters=[{'output_csv': LaunchConfiguration('output_csv')}]
        )
    ])
