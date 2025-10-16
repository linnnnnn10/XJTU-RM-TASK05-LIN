#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ImageChecker(Node):
    def __init__(self):
        super().__init__('image_checker')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.received = False
        
    def image_callback(self, msg):
        self.received = True
        print(f"Received image: {msg.width}x{msg.height}, encoding: {msg.encoding}")
        
        # 尝试转换图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print(f"CV image shape: {cv_image.shape}")
            print(f"Image data type: {cv_image.dtype}")
            print("Image topic is working correctly!")
        except Exception as e:
            print(f"Error converting image: {e}")
        
        # 检查图像内容
        if cv_image.shape == (480, 640, 3):
            print("Image dimensions match expected simulation mode (480x640x3)")
        
        rclpy.shutdown()

def main():
    rclpy.init()
    checker = ImageChecker()
    
    # 等待一段时间接收消息
    print("Waiting for image messages...")
    rclpy.spin_once(checker, timeout_sec=5.0)
    
    if not checker.received:
        print("No image messages received after 5 seconds")
    
    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
