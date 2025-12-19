import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('jarabot_sim_cartographer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = LaunchConfiguration('config_dir')
    config_basename = LaunchConfiguration('config_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    start_delay = LaunchConfiguration('start_delay')

    # --- Nodes (defined BEFORE TimerAction uses them) ---
    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename
        ],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    occ_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.path.join(pkg_share, 'config')
        ),
        DeclareLaunchArgument('config_basename', default_value='jarabot_2d.lua'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('start_delay', default_value='5.0'),

        # Delay start so TF(/odom->base_link) and /scan are ready
        TimerAction(
            period=start_delay,
            actions=[carto_node, occ_node]
        ),
    ])
