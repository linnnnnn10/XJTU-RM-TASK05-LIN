# 基于海康相机的装甲板识别及pnp结算
## 命令行编译
```bash
colcon build --packages-select hik_camera
source install/setup.bash
```
## 运行节点
```bash
ros2 launch hik_camera hik_camera.launch.py
```
## 拍照功能
```bash
//另开终端
ros2 run hik_camera take_photos.py
//另开终端
ros2 topic pub /photo_trigger std_msgs/msg/Empty "{}" --once
```
## 识别及pnp结算
```bash
//另开终端
ros2 run hik_camera armor_detection_node
//如果不行尝试下面命令
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 ros2 run hik_camera armor_detection_node --ros-args -p camera_topic:=/image_raw
```
