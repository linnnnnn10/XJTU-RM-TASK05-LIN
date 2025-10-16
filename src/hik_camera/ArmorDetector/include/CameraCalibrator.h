#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class CameraCalibrator {
public:
    CameraCalibrator();
    
    // 相机标定主函数
    bool calibrateCamera(const std::vector<std::string>& imagePaths, 
                        cv::Size boardSize, float squareSize);
    
    // 保存标定参数
    bool saveCalibrationParams(const std::string& filename);
    
    // 加载标定参数
    bool loadCalibrationParams(const std::string& filename);
    
    // 畸变校正
    void undistortImage(const cv::Mat& input, cv::Mat& output);
    
    // 获取相机矩阵和畸变系数
    cv::Mat getCameraMatrix() const { return cameraMatrix; }
    cv::Mat getDistCoeffs() const { return distCoeffs; }
    
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    
    // 查找棋盘格角点
    bool findChessboardCorners(const std::vector<std::string>& imagePaths,
                              cv::Size boardSize,
                              std::vector<std::vector<cv::Point2f>>& imagePoints);
};

#endif // CAMERACALIBRATOR_H