"""
Fast Maze Exploration Launch File

Launches all components for high-speed maze mapping challenge:
- Fast SLAM node (fewer particles, coarser map)
- Fast exploration node (aggressive frontier selection)
- Fast motion controller (pure pursuit, higher speeds)
- RViz2 for real-time visualization (optional)

Usage:
    ros2 launch mbot_nav maze_challenge.launch.py
    ros2 launch mbot_nav maze_challenge.launch.py rviz:=true   # with RViz2

Optional arguments:
    max_linear_vel:=0.5     Maximum linear velocity (m/s)
    max_angular_vel:=2.5    Maximum angular velocity (rad/s)
    rviz:=false             Set true to launch RViz2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )
    
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='2.5',
        description='Maximum angular velocity (rad/s)'
    )
    
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.25',
        description='Pure pursuit lookahead distance (m)'
    )
    
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.15',
        description='Waypoint reached tolerance (m)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # RViz2 config path
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('mbot_nav'),
        'rviz',
        'maze_exploration.rviz'
    ])

    # Fast SLAM node
    fast_slam_node = Node(
        package='mbot_slam',
        executable='fast_slam_node',
        name='fast_slam_node',
        output='screen',
    )

    # Fast exploration node
    fast_exploration_node = Node(
        package='mbot_nav',
        executable='fast_exploration_node',
        name='fast_exploration_node',
        output='screen',
        parameters=[{
            'goal_reached_threshold': 0.25,
            'timer_period': 0.05,
            'no_frontier_threshold': 15,
            'min_frontier_distance': 0.15,
            'goal_clearance': 0.08,
        }]
    )

    # Fast motion controller
    fast_motion_controller = Node(
        package='mbot_setpoint',
        executable='motion_controller_fast',
        name='fast_motion_controller',
        output='screen',
        parameters=[{
            'use_localization': True,
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
        }]
    )

    # RViz2 (optional, launch with rviz:=true)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        max_linear_vel_arg,
        max_angular_vel_arg,
        lookahead_distance_arg,
        goal_tolerance_arg,
        rviz_arg,
        fast_slam_node,
        fast_exploration_node,
        fast_motion_controller,
        rviz_node,
    ])
