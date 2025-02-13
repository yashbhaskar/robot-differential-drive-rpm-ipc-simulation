import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Play the ROS 2 bag file
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', os.path.join(
                '/home/yash/ros_ws/src/rse_assignment/bag_files/rse_assignment.db3')],
            output='screen'
        ),

        # Run script_a
        ExecuteProcess(
            cmd=['ros2', 'run', 'rse_assignment', 'script_a'],
            output='screen'
        ),

        # Run script_b
        ExecuteProcess(
            cmd=['ros2', 'run', 'rse_assignment', 'script_b'],
            output='screen'
        ),

        # Run script_c.py
        ExecuteProcess(
            cmd=['ros2', 'run', 'rse_assignment', 'script_c.py'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rse_assignment', 'seaborn.py'],
            output='screen'
        ),

    ])
