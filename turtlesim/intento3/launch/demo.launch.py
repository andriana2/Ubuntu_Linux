from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='intento2',
            executable='intento2',
            name='intento2',
            output='screen',
            emulate_tty=True,
        ),
    ])
