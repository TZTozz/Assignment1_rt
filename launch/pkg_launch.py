from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),

        ExecuteProcess(
            cmd=['konsole', '-e', 'ros2', 'run', 'assignment1_rt', 'UI'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['konsole', '-e', 'ros2', 'run', 'assignment1_rt', 'distance'],
            output='screen'
        ),

        Node(
            package='assignment1_rt',
            executable='spawner',
            name='spawner'
        )
    ])
