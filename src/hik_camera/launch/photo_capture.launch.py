from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包目录
    pkg_dir = get_package_share_directory('hik_camera')
    
    return LaunchDescription([
        # 启动现有的海康相机节点
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera_node',
            output='screen'
        ),
        
        # 启动拍照节点
        Node(
            package='hik_camera',
            executable='take_photos.py',
            name='photo_capture_node',
            output='screen',
            # 可以添加参数，例如：
            # parameters=[{'max_photos': 10}]
        ),
    ])