from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    global_localization_node = Node(
        package='mbot_localization',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[{
            'variance_threshold': LaunchConfiguration('variance_threshold'),
            'forward_speed': 0.1,
            'rotation_speed': 0.5,
            'obstacle_distance': 0.35,
        }]
    )

    return LaunchDescription([
        map_name_arg,
        map_package_arg,
        variance_threshold_arg,
        map_server,
        lifecycle_manager,
        global_localization_node,
    ])
