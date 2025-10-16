import cv2
import sys

if len(sys.argv) > 1:
    image_path = sys.argv[1]
else:
    image_path = "/home/linnnnnn/ros2_ws/src/hik_camera/ArmorDetector/data/photos/photo_20251014_214215_556611.jpg"

image = cv2.imread(image_path)
print(f"图片尺寸: {image.shape}")

# 尝试不同的棋盘格尺寸
sizes = [(9,6), (8,5), (7,5), (6,4), (5,4)]

for size in sizes:
    found, corners = cv2.findChessboardCorners(image, size)
    print(f"棋盘格 {size[0]}x{size[1]}: {'找到' if found else '未找到'}")
    
    if found:
        # 显示结果
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        
        result = image.copy()
        cv2.drawChessboardCorners(result, size, corners, found)
        
        # 调整大小并显示
        result = cv2.resize(result, (800, 600))
        cv2.imshow(f"棋盘格 {size[0]}x{size[1]}", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
