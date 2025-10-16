#include "CameraCalibrator.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

CameraCalibrator::CameraCalibrator() {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
}

bool CameraCalibrator::calibrateCamera(const std::vector<std::string>& imagePaths, 
                                      cv::Size boardSize, float squareSize) {
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    
    // 查找棋盘格角点
    if (!findChessboardCorners(imagePaths, boardSize, imagePoints)) {
        std::cerr << "Failed to find chessboard corners in images" << std::endl;
        return false;
    }
    
    // 准备世界坐标系中的角点坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    
    for (size_t i = 0; i < imagePoints.size(); i++) {
        objectPoints.push_back(obj);
    }
    
    // 执行相机标定
    double rms = cv::calibrateCamera(objectPoints, imagePoints, boardSize, 
                                    cameraMatrix, distCoeffs, rvecs, tvecs);
    
    std::cout << "Camera calibration completed with RMS error: " << rms << std::endl;
    return true;
}

bool CameraCalibrator::saveCalibrationParams(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }
    
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();
    
    std::cout << "Calibration parameters saved to: " << filename << std::endl;
    return true;
}

bool CameraCalibrator::loadCalibrationParams(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return false;
    }
    
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
    
    std::cout << "Calibration parameters loaded from: " << filename << std::endl;
    return true;
}

void CameraCalibrator::undistortImage(const cv::Mat& input, cv::Mat& output) {
    cv::undistort(input, output, cameraMatrix, distCoeffs);
}

bool CameraCalibrator::findChessboardCorners(const std::vector<std::string>& imagePaths,
                                           cv::Size boardSize,
                                           std::vector<std::vector<cv::Point2f>>& imagePoints) {
    imagePoints.clear();
    
    for (const auto& path : imagePaths) {
        cv::Mat image = cv::imread(path);
        if (image.empty()) {
            std::cerr << "Failed to load image: " << path << std::endl;
            continue;
        }
        
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners);
        
        if (found) {
            // 提高角点精度
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            imagePoints.push_back(corners);
        } else {
            std::cerr << "Chessboard not found in image: " << path << std::endl;
        }
    }
    
    return !imagePoints.empty();
}