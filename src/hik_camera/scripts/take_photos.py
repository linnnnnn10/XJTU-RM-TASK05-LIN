#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime

class DebugPhotoCaptureNode(Node):
    def __init__(self):
        super().__init__('debug_photo_capture_node')
        
        self.save_dir = os.path.join(os.getcwd(), 'photos')
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.bridge = CvBridge()
        self.photo_count = 0
        self.latest_image = None
        self.image_received = False
        
        # 订阅图像
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('已订阅相机话题: /hik_camera/image_raw')
        
        # 订阅拍照触发话题
        self.trigger_subscription = self.create_subscription(
            Empty,
            '/photo_trigger',
            self.trigger_callback,
            10
        )
        self.get_logger().info('已订阅触发话题: /photo_trigger')
        
        # 创建定时器检查状态
        self.timer = self.create_timer(2.0, self.status_check)
        
        self.get_logger().info('调试拍照节点已启动')
        
    def status_check(self):
        """定期检查节点状态"""
        if not self.image_received:
            self.get_logger().info('状态: 等待相机图像...')
        else:
            self.get_logger().info('状态: 相机图像已就绪，等待触发信号')
        
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if not self.image_received:
                self.image_received = True
                self.get_logger().info('✓ 成功接收到相机图像')
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def trigger_callback(self, msg):
        """触发回调函数"""
        self.get_logger().info('收到拍照触发信号!')
        
        if not self.image_received or self.latest_image is None:
            self.get_logger().warning('拍照失败：尚未收到相机图像')
            return
        
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = f'photo_{timestamp}.jpg'
        filepath = os.path.join(self.save_dir, filename)
        
        try:
            cv2.imwrite(filepath, self.latest_image)
            self.photo_count += 1
            self.get_logger().info(f'✓ 照片已保存: {filename} (总计: {self.photo_count})')
            
            # 显示预览
            cv2.imshow('Latest Photo', self.latest_image)
            cv2.waitKey(100)  # 显示100ms
            
        except Exception as e:
            self.get_logger().error(f'保存照片失败: {e}')

def main():
    rclpy.init()
    node = DebugPhotoCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    except Exception as e:
        node.get_logger().error(f'节点错误: {e}')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()