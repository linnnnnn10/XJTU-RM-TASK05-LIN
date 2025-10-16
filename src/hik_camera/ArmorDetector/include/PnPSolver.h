#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

#include <opencv2/opencv.hpp>

class PnPSolver {
public:
    PnPSolver();
    
    // 设置相机参数
    void setCameraParams(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    
    // 设置装甲板3D模型（单位：米）
    void setArmorModel(float width, float height);
    
    // 求解姿态
    bool solvePnP(const std::vector<cv::Point2f>& imagePoints, 
                  cv::Mat& rvec, cv::Mat& tvec);
    
    // 获取相机坐标系下的3D坐标
    cv::Point3f getCameraCoordinates(const cv::Mat& tvec);
    
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Point3f> armorModel3D;
};

#endif // PNP_SOLVER_H