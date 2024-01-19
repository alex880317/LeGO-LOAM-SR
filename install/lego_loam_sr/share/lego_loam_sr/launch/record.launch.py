import os
import launch
from launch_ros.actions import Node

# def generate_launch_description():
#     # 獲取當前功能包的路徑
#     pkg_path = os.path.join(os.path.dirname(__file__), '..')
#     # 創建完整的 rosbag 路徑
#     rosbag_path = os.path.join(pkg_path, 'rosbag')
    
#     return launch.LaunchDescription([
#         Node(
#             package='rosbag2', 
#             executable='record', 
#             name='rosbag_record',
#             arguments=['-o', rosbag_path, '/odom', '/racebot/joint_states', '/imu/data'],
#             output='screen'
#         ),
#         # 其他需要啟動的節點
#     ])
    
def generate_launch_description():
    # 獲取當前功能包的路徑
    pkg_path = os.path.join(os.path.dirname(__file__), '..')
    # 創建完整的 rosbag 路徑
    rosbag_path = os.path.join(pkg_path, 'rosbag')
    
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            # cmd=['ros2', 'bag', 'record', '-o', rosbag_path, '/odom', '/imu/data', '/racebot/joint_states'],
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        )
    ])
