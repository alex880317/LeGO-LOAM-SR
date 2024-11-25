import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser


def generate_launch_description():

    # Configure environment
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # Simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nodes Configurations
    config_file = os.path.join(get_package_share_directory('lego_loam_sr'), 'config', 'loam_config.yaml')
    rviz_config = os.path.join(get_package_share_directory('lego_loam_sr'), 'rviz', 'origin.rviz')

    # Declare launch arguments for parameters
    declare_pgo_cov_param = DeclareLaunchArgument(
        'PGO_cov_param',
        default_value='[1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6]',
        description='Covariance parameters for PGO'
    )

    declare_ground_plane_param = DeclareLaunchArgument(
        'Ground_Plane_param',
        default_value='[1e-4, 1e-4, 1e-8]',
        description='Parameters for ground plane'
    )

    # Tf transformations
    transform_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='camera_init_to_map',
        arguments=['0', '0', '0', '1.570795', '0', '1.570795', 'map', 'camera_init'],
    )

    transform_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='base_link_to_camera',
        arguments=['0', '0', '0', '-1.570795', '-1.570795', '0', 'camera', 'base_link'],
    )

    # LeGO-LOAM node
    lego_loam_node = Node(
        package='lego_loam_sr',
        executable='lego_loam_sr',
        output='screen',
        parameters=[
            config_file,
            {
                'mapping.PGO_cov_param': LaunchConfiguration('PGO_cov_param'),
                'mapping.Ground_Plane_param': LaunchConfiguration('Ground_Plane_param'),
            }
        ],
        remappings=[('/lidar_points', '/velodyne_points')],
    )

    # Rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_pgo_cov_param)
    ld.add_action(declare_ground_plane_param)

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)

    # Add nodes
    ld.add_action(lego_loam_node)
    ld.add_action(transform_map)
    ld.add_action(transform_camera)
    ld.add_action(rviz_node)

    return ld
