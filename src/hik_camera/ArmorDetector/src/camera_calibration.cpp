#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include "CameraCalibrator.h"

int main(int argc, char** argv) {
    std::cout << "=== Camera Calibration Tool ===" << std::endl;
    
    // 标定图片路径（确保没有末尾斜杠）
    std::string photo_dir = "/home/linnnnnn/ros2_ws/src/hik_camera/ArmorDetector/data/photos";
    std::string output_file = "/home/linnnnnn/ros2_ws/src/hik_camera/ArmorDetector/data/calibration_params.yml";
    
    std::cout << "Looking for images in: " << photo_dir << std::endl;
    
    // 检查目录是否存在
    DIR* dir = opendir(photo_dir.c_str());
    if (dir == nullptr) {
        std::cerr << "Error: Cannot open directory: " << photo_dir << std::endl;
        std::cerr << "Please make sure the directory exists and contains calibration images." << std::endl;
        return -1;
    }
    
    // 手动列出目录内容进行调试
    std::cout << "Directory contents:" << std::endl;
    struct dirent* entry;
    int file_count = 0;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG) { // 普通文件
            std::string filename = entry->d_name;
            std::cout << "  " << filename << std::endl;
            file_count++;
        }
    }
    closedir(dir);
    
    std::cout << "Total files found: " << file_count << std::endl;
    
    // 获取标定图片文件列表 - 使用正确的路径拼接
    std::vector<std::string> image_paths;
    
    // 分别尝试不同扩展名
    std::vector<std::string> extensions = {".jpg", ".JPG", ".png", ".PNG", ".bmp", ".BMP"};
    
    for (const auto& ext : extensions) {
        std::string pattern = photo_dir + "/*" + ext;
        std::vector<std::string> files;
        cv::glob(pattern, files, false); // 不使用递归
        
        if (!files.empty()) {
            std::cout << "Found " << files.size() << " files with extension " << ext << std::endl;
            image_paths.insert(image_paths.end(), files.begin(), files.end());
        }
    }
    
    if (image_paths.empty()) {
        std::cerr << "Error: No calibration images found in " << photo_dir << std::endl;
        std::cerr << "Tried extensions: jpg, JPG, png, PNG, bmp, BMP" << std::endl;
        return -1;
    }
    
    std::cout << "Successfully found " << image_paths.size() << " calibration images" << std::endl;
    
    // 显示前几个文件的完整路径
    std::cout << "First few images:" << std::endl;
    for (size_t i = 0; i < std::min(image_paths.size(), size_t(3)); i++) {
        std::cout << "  " << image_paths[i] << std::endl;
    }
    
    // 创建标定器
    auto calibrator = std::make_shared<CameraCalibrator>();
    
    // 设置棋盘格参数
    cv::Size board_size(8, 5);  // 内部角点数量
    float square_size = 0.025f; // 每个方格的实际大小（米）
    
    std::cout << "Board size: " << board_size.width << "x" << board_size.height << std::endl;
    std::cout << "Square size: " << square_size << " meters" << std::endl;
    std::cout << "Starting calibration..." << std::endl;
    
    // 进行相机标定
    if (calibrator->calibrateCamera(image_paths, board_size, square_size)) {
        std::cout << "Calibration successful!" << std::endl;
        
        // 保存标定参数
        if (calibrator->saveCalibrationParams(output_file)) {
            std::cout << "Calibration parameters saved to: " << output_file << std::endl;
            
            // 显示标定结果
            cv::Mat camera_matrix = calibrator->getCameraMatrix();
            cv::Mat dist_coeffs = calibrator->getDistCoeffs();
            
            std::cout << "\nCamera Matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;
            std::cout << "\nDistortion Coefficients:" << std::endl;
            std::cout << dist_coeffs << std::endl;
            
            // 测试畸变校正
            if (!image_paths.empty()) {
                cv::Mat test_image = cv::imread(image_paths[0]);
                if (!test_image.empty()) {
                    cv::Mat undistorted;
                    calibrator->undistortImage(test_image, undistorted);
                    
                    // 显示原图和校正后的图
                    cv::Mat combined;
                    cv::hconcat(test_image, undistorted, combined);
                    cv::resize(combined, combined, cv::Size(1200, 400));
                    cv::putText(combined, "Original", cv::Point(10, 30), 
                               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                    cv::putText(combined, "Undistorted", cv::Point(610, 30), 
                               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                    
                    cv::imshow("Calibration Result - Left: Original, Right: Undistorted", combined);
                    std::cout << "\nPress any key to close the image and exit..." << std::endl;
                    cv::waitKey(0);
                    cv::destroyAllWindows();
                }
            }
            
        } else {
            std::cerr << "Error: Failed to save calibration parameters to " << output_file << std::endl;
            return -1;
        }
    } else {
        std::cerr << "Error: Camera calibration failed!" << std::endl;
        return -1;
    }
    
    return 0;
}