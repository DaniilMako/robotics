import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='raph',
        parameters=[{
            "background_r": 255,  # красный
            "background_g": 0,
            "background_b": 0,
        }],
    output='screen',
    arguments=['--ros-args', '--log-level', 'info']  # Задание уровня логирования
    )
    
    turtlesim2 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='mickey',
        parameters=[{
            "background_r": 255,
            "background_g": 165,
            "background_b": 0,
        }],
    output='screen',
    arguments=['--ros-args', '--log-level', 'info']  # Задание уровня логирования
    )
    turtlesim3 =  Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='leo',
        parameters=[{
            "background_r": 0,
            "background_g": 0,
            "background_b": 255,
        }],
    output='screen',
    arguments=['--ros-args', '--log-level', 'info']  # Задание уровня логирования
    )
    # переназначение, чтобы все три черепашки слушали одну и ту же команду
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic1',
        remappings=[
            ('/input/pose', '/turtle1/pose'),
            ('/output/cmd_vel', '/turtle2/cmd_vel'),
        ]
    )
    forward_turtlesim_commands_to_third_turtlesim_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic2',
        remappings=[
            ('/input/pose', '/turtle2/pose'),
            ('/output/cmd_vel', '/turtle3/cmd_vel'),
        ]
    )


    return LaunchDescription([
        # background_r_launch_arg,
        # background_g_launch_arg,
        # background_b_launch_arg,
        # chatter_ns_launch_arg,
        turtlesim1,
        turtlesim2,
        turtlesim3,
        forward_turtlesim_commands_to_second_turtlesim_node,
        forward_turtlesim_commands_to_third_turtlesim_node,
])