from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_csv',
            default_value='pf_log_1p0max.csv',
            description='CSV file to write Particle Filter data to'
        ),
        Node(
            package='localization_comparing',
            executable='pf_logger',
            name='pf_logger',
            output='screen',
            parameters=[{'output_csv': LaunchConfiguration('output_csv')}]
        )
    ])
