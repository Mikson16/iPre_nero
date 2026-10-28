from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'channel',
            default_value='can0',
            description='SocketCAN channel.',
        ),
        DeclareLaunchArgument(
            'bitrate',
            default_value='1000000',
            description='CAN bitrate.',
        ),
        DeclareLaunchArgument(
            'firmware',
            default_value='v111',
            description='Nero firmware driver: v111, v112, v120 or v121.',
        ),
        DeclareLaunchArgument(
            'joint_index',
            default_value='1',
            description='Joint 1-7, or 255 for all joints.',
        ),
        Node(
            package='nero_ctrl',
            executable='calibration_node',
            name='nero_calibration_node',
            output='screen',
            parameters=[{
                'channel': LaunchConfiguration('channel'),
                'bitrate': LaunchConfiguration('bitrate'),
                'firmware': LaunchConfiguration('firmware'),
                'joint_index': LaunchConfiguration('joint_index'),
            }],
        ),
    ])
