from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
        # 启动海康相机节点
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen',
            parameters=[{
                'camera_serial': '',
                'exposure_time': 10000,
                'gain': 1.0,
                'frame_rate': 30.0,
                'pixel_format': 'BGR8',
                'image_topic': 'image_raw'
            }]
        ),
        
        # 启动装甲板检测节点
        Node(
            package='armor_detector',
            executable='armor_detection_node',
            name='armor_detection',
            output='screen',
            parameters=[{
                'camera_topic': '/hik_camera/image_raw',
                'calibration_file': '/home/linnnnnn/ArmorDetector/data/calibration_params.yml',
                'model_path': '/home/linnnnnn/ArmorDetector/data/model/resnet_model.onnx',
                'use_calibration': True
            }]
        )
    ])