# 基于海康相机的装甲板识别

## 安装依赖
确保C++,ROS2,MVS SDK,等依赖已经安装

## 命令行编译
```bash
colcon build --packages-select hik_camera
source install/setup.bash
```

## 运行节点
```bash
ros2 launch hik_camera hik_camera.launch.py
```
## 配置参数

cd src/hik_camera/ArmorDetector/python_scripts
source venv/bin/activate
ros2 run hik_camera take_photos.py
ros2 topic pub /photo_trigger std_msgs/msg/Empty "{}" --once
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 ros2 run hik_camera armor_detection_node --ros-args -p camera_topic:=/image_raw