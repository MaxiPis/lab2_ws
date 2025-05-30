from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_parte_1',
            executable='pose_loader',
            name='pose_loader',
            output='screen'
        ),
        Node(
            package='pkg_parte_1',
            executable='robot',
            name='robot',
            output='screen'
        ),
        Node(
            package='pkg_parte_1',
            executable='publicador_vel',
            name='publicador_vel',
            output='screen'
        ),
        Node(
            package='pkg_parte_1',
            executable='controlador_P',
            name='controlador_P',
            output='screen'
        )
    ])
