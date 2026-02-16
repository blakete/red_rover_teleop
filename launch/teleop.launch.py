#!/usr/bin/env python3
"""
ROS 2 Launch file for Xbox 360 teleop control.

Launches:
  - joy_node: Reads Xbox 360 controller input from /dev/input/js0
  - xbox_teleop_node: Converts joy messages to Twist commands

Usage:
  ros2 launch teleop.launch.py namespace:=/rover1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/rover1',
        description='Namespace for cmd_vel topic (e.g., /rover1)'
    )
    
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')
    
    # Joy node - reads controller input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # Xbox teleop node - converts joy to Twist
    # Full stick = full speed (no turbo mode)
    teleop_node = Node(
        package=None,  # Not a package, running script directly
        executable='/ros2_ws/src/xbox_teleop_node.py',
        name='xbox_teleop_node',
        parameters=[{
            'max_linear_vel': 1.0,   # m/s (matches Teensy VMAX)
            'max_angular_vel': 1.5,  # rad/s (matches Teensy OMEGAMAX)
            'deadzone': 0.1,
            'smoothing_tau': 0.08,   # Snappy response
            'publish_rate': 50.0,
        }],
        remappings=[
            ('cmd_vel', [namespace, '/cmd_vel']),
            ('arm', [namespace, '/arm']),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        joy_dev_arg,
        LogInfo(msg=['Starting Xbox Teleop with namespace: ', namespace]),
        joy_node,
        teleop_node,
    ])
