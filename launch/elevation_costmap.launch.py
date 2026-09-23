from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('elevation_costmap')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to elevation_costmap parameter YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    node = Node(
        package='elevation_costmap',
        executable='elevation_costmap_node',
        name='elevation_costmap_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # base_link -> livox_frame: z=0.365 m, pitch=7 deg
    static_tf_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.365',
            '--roll', '0',
            '--pitch', '0.12217304764',  # 7 deg in rad
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        node,
        static_tf_base_to_livox,
    ])
