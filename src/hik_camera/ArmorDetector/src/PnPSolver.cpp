#include "PnPSolver.h"
#include <opencv2/calib3d.hpp>

PnPSolver::PnPSolver() {
    // 默认装甲板尺寸（单位：米）
    setArmorModel(0.135f, 0.055f);
}

void PnPSolver::setCameraParams(const cv::Mat& camMatrix, const cv::Mat& distCoeffs) {
    // 修正：克隆const参数以避免限定符丢弃
    cameraMatrix = camMatrix.clone();
    this->distCoeffs = distCoeffs.clone();
}

void PnPSolver::setArmorModel(float width, float height) {
    armorModel3D.clear();
    // 装甲板3D模型点（中心为原点）
    armorModel3D = {
        cv::Point3f(-width/2, -height/2, 0),  // 左上
        cv::Point3f(width/2, -height/2, 0),   // 右上
        cv::Point3f(width/2, height/2, 0),    // 右下
        cv::Point3f(-width/2, height/2, 0)    // 左下
    };
}

bool PnPSolver::solvePnP(const std::vector<cv::Point2f>& imagePoints, 
                         cv::Mat& rvec, cv::Mat& tvec) {
    if (imagePoints.size() != 4) return false;
    
    return cv::solvePnP(armorModel3D, imagePoints, cameraMatrix, distCoeffs, 
                       rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
}

cv::Point3f PnPSolver::getCameraCoordinates(const cv::Mat& tvec) {
    return cv::Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
}