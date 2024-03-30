import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser

import yaml

## Alex
def load_config(context, *args, **kwargs):
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    high_dense_mapping = LaunchConfiguration('HighDenseMapping').perform(context)
    config_path = os.path.join(get_package_share_directory('lego_loam_sr'), 'config', 'loam_config.yaml')
    
    with open(config_path, 'r') as file:
        all_configs = yaml.safe_load(file)
    
    # 根据 lidar_type 参数选择配置
    selected_config = all_configs.get(lidar_type, {})
    
    # 设置 HighDenseMapping 参数
    if high_dense_mapping.lower() == 'true':
        selected_config['map_optimization.ros__parameters.HighDenseMapping'] = True
    else:
        selected_config['map_optimization.ros__parameters.HighDenseMapping'] = False
    
    lego_loam_node = Node(
        package='lego_loam_sr',
        executable='lego_loam_sr',
        name='lego_loam_sr',
        output='screen',
        parameters=[selected_config],
        # remappings=[('/lidar_points', 'velodyne_points'),('/imu_type', '/kitti/oxts/imu')],
        remappings=[('/lidar_points', '/velodyne_points'),('/imu_type', '/imu/data')],
        # remappings=[('/lidar_points', '/velodyne_pcl_gen/cloud')],
        # remappings=[('/lidar_points', '/kitti/velo/pointcloud')],
        # remappings=[('/lidar_points', '/points_raw')],
        # remappings=[('/lidar_points', '/sensing/lidar/top/pointcloud_raw')],
        # remappings=[('/imu_type', '/imu/data')],
        # remappings=[('/imu_type', '/imu_correct')],
        # remappings=[('/imu_type', '/lidar_xsens/imu/data')],
        # remappings=[('/imu_type', '/kitti/oxts/imu')],
    )
    
    return [lego_loam_node]
##



def generate_launch_description():

  # Configure environment
  stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
  stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

  # Simulated time
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  # Nodes Configurations
  # config_file = os.path.join(get_package_share_directory('lego_loam_sr'), 'config', 'loam_config.yaml')
  rviz_config = os.path.join(get_package_share_directory('lego_loam_sr'), 'rviz', 'test2.rviz')

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

  # LeGO-LOAM
  # Declare the 'lidar_type' launch argument
  declare_lidar_type_arg = DeclareLaunchArgument(
      'lidar_type',
      default_value='VLP-16',
      description='Type of LIDAR: VLP-16, VLP-32c, or HDL-64E'
  )
  # 声明HighDenseMapping参数
  declare_high_dense_mapping_arg = DeclareLaunchArgument(
      'HighDenseMapping',
      default_value='false',  # 设置默认值为false
      description='Enable or disable high dense mapping'
  )
  # lego_loam_node = Node(
  #   package='lego_loam_sr',
  #   executable='lego_loam_sr',
  #   output='screen',
  #   parameters=[{config_file,
  #                }],
  #   # remappings=[('/lidar_points', 'velodyne_points'),('/imu_type', '/kitti/oxts/imu')],
  #   remappings=[('/lidar_points', '/velodyne_points'),('/imu_type', '/imu/data')],
  #   # remappings=[('/lidar_points', '/velodyne_pcl_gen/cloud')],
  #   # remappings=[('/lidar_points', '/kitti/velo/pointcloud')],
  #   # remappings=[('/lidar_points', '/points_raw')],
  #   # remappings=[('/lidar_points', '/sensing/lidar/top/pointcloud_raw')],
  #   # remappings=[('/imu_type', '/imu/data')],
  #   # remappings=[('/imu_type', '/imu_correct')],
  #   # remappings=[('/imu_type', '/lidar_xsens/imu/data')],
  #   # remappings=[('/imu_type', '/kitti/oxts/imu')],
  # )

  # Rviz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    namespace='rviz2',
    arguments=['-d', rviz_config],
    output='screen'
  )

  ld = LaunchDescription()
  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(stdout_colorized_envvar)
  # Add nodes
  # ld.add_action(lego_loam_node)
  ld.add_action(transform_map)
  ld.add_action(transform_camera)
  ld.add_action(rviz_node)
  # 添加到LaunchDescription实例
  ld.add_action(declare_lidar_type_arg)
  ld.add_action(declare_high_dense_mapping_arg)
  ld.add_action(OpaqueFunction(function=load_config))

  return ld
