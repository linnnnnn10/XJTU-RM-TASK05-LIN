cmake_minimum_required(VERSION 3.8)
project(hik_camera)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# 查找MVS SDK（可选）
find_path(MVS_INCLUDE_DIRS 
  NAMES MvCameraControl.h 
  PATHS /opt/MVS/include /usr/local/include /usr/include
  NO_DEFAULT_PATH
)

find_library(MVS_LIBRARIES 
  NAMES MvCameraControl MvCameraControl.so
  PATHS /opt/MVS/lib/64 /usr/local/lib /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH
)

if(MVS_INCLUDE_DIRS AND MVS_LIBRARIES)
  message(STATUS "Found MVS SDK: ${MVS_INCLUDE_DIRS}, ${MVS_LIBRARIES}")
  add_definitions(-DUSE_HIK_CAMERA)
else()
  message(WARNING "MVS SDK not found, building in simulation mode")
  add_definitions(-DSIMULATION_MODE)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 包含目录
include_directories(
  include
  ArmorDetector/include
  ${MVS_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(hik_camera_node src/hik_camera_node.cpp)
add_executable(armor_detection_node ArmorDetector/src/main_armor_detection.cpp)
add_executable(camera_calibration_node ArmorDetector/src/camera_calibration.cpp)

# 添加库
add_library(armor_detector_lib
  ArmorDetector/src/ArmorDetector.cpp
  ArmorDetector/src/CameraCalibrator.cpp
  ArmorDetector/src/NeuralNetwork.cpp
  ArmorDetector/src/PnPSolver.cpp
)

# 使用ament_target_dependencies确保ROS2依赖正确传递
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

ament_target_dependencies(armor_detection_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 链接库
target_link_libraries(hik_camera_node 
  ${MVS_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(armor_detector_lib 
  ${OpenCV_LIBS}
)

target_link_libraries(armor_detection_node
  armor_detector_lib
  ${OpenCV_LIBS}
)

target_link_libraries(camera_calibration_node
  armor_detector_lib
  ${OpenCV_LIBS}
)

# 安装目标
install(TARGETS
  hik_camera_node
  armor_detection_node
  camera_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/take_photos.py
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rclcpp sensor_msgs cv_bridge image_transport)
ament_package()