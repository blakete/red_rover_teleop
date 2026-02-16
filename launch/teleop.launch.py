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
    # Normal mode: 50% speed | Turbo (hold LT+RT): 100% speed
    teleop_node = Node(
        package=None,  # Not a package, running script directly
        executable='/ros2_ws/src/xbox_teleop_node.py',
        name='xbox_teleop_node',
        parameters=[{
            'max_linear_vel': 1.5,   # m/s (pushing beyond 3-DX rated 1.2)
            'max_angular_vel': 3.0,  # rad/s (3-DX hardware limit ~7.3 rad/s)
            'normal_scale': 1.0,     # 100% speed at full stick
            'turbo_scale': 1.0,      # same as normal (no separate turbo)
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
