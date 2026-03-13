"""
双机器人全局定位演示 Launch 文件

场景：
- Robot #1: 定位后驾驶到指定位置打开Robot #2的牢房
- Robot #2: 定位找到自己的位置，然后逃脱回到起点

使用方式：
    ros2 launch mbot_localization dungeon_escape.launch.py map_name:=event3_map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # =============== 启动参数声明 ===============
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        description='Map file name (without .yaml extension)'
    )

    map_package_arg = DeclareLaunchArgument(
        'map_package',
        default_value='mbot_nav',
        description='Package that contains the maps/ directory'
    )

    variance_threshold_arg = DeclareLaunchArgument(
        'variance_threshold',
        default_value='0.05',
        description='Particle variance threshold for localization success'
    )

    # =============== 地图服务器 ===============
    map_file_path = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('map_package')),
        'maps',
        [LaunchConfiguration('map_name'), '.yaml']
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path, 'use_sim_time': False}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}]
    )

    # =============== Robot #1 - 全局定位节点 ===============
    robot1_global_localization = Node(
        package='mbot_localization',
        executable='global_localization_node',
        name='robot1_global_localization',
        output='screen',
        remappings=[
            # 如果使用了命名空间隔离，可以重映射话题
            # ('/scan', '/robot1/scan'),
            # ('/map', '/robot1/map'),
        ],
        parameters=[{
            'variance_threshold': LaunchConfiguration('variance_threshold'),
            'forward_speed': 0.1,
            'rotation_speed': 0.5,
            'obstacle_distance': 0.35,
            'robot_namespace': 'robot1',
            'publish_localized_pose': True,
        }],
        prefix=['export ROS_NAMESPACE=/robot1; '],
    )

    # =============== Robot #2 - 全局定位节点 ===============
    robot2_global_localization = Node(
        package='mbot_localization',
        executable='global_localization_node',
        name='robot2_global_localization',
        output='screen',
        remappings=[
            # 如果使用了命名空间隔离，可以重映射话题
            # ('/scan', '/robot2/scan'),
            # ('/map', '/robot2/map'),
        ],
        parameters=[{
            'variance_threshold': LaunchConfiguration('variance_threshold'),
            'forward_speed': 0.1,
            'rotation_speed': 0.5,
            'obstacle_distance': 0.35,
            'robot_namespace': 'robot2',
            'publish_localized_pose': True,
        }],
        prefix=['export ROS_NAMESPACE=/robot2; '],
    )

    # =============== 位置监听节点 ===============
    pose_listener = Node(
        package='mbot_localization',
        executable='localization_pose_listener',
        name='pose_listener',
        output='screen',
    )

    # =============== RViz 可视化 (可选) ===============
    rviz_config = PathJoinSubstitution([
        FindPackageShare('mbot_localization'),
        'rviz',
        'localization.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        # 参数声明
        map_name_arg,
        map_package_arg,
        variance_threshold_arg,
        
        # 地图服务
        map_server,
        lifecycle_manager,
        
        # 机器人定位节点
        robot1_global_localization,
        robot2_global_localization,
        
        # 位置监听节点
        pose_listener,
        
        # RViz 可视化
        # rviz_node,  # 注释掉如果不需要rviz
    ])
