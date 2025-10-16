#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "NeuralNetwork.h"

struct ArmorPlate {
    cv::Rect boundingRect;
    cv::Rect digitRect;  // 数字3的区域
    std::vector<cv::Point2f> corners;
    int digit;
    float confidence;
};

class ArmorDetector {
public:
    ArmorDetector();
    
    // 设置神经网络模型
    bool setNeuralNetwork(std::shared_ptr<NeuralNetwork> nn);
    
    // 处理单帧图像
    std::vector<ArmorPlate> detectArmor(const cv::Mat& frame);
    
    // 绘制检测结果（红色框框出数字3，绿色框框出装甲板）
    void drawResults(cv::Mat& frame, const std::vector<ArmorPlate>& armors);
    
private:
    std::shared_ptr<NeuralNetwork> neuralNetwork;
    std::vector<cv::Rect> digit3Regions;  // 存储检测到的数字3区域
    
    // 图像预处理
    void preprocessFrame(const cv::Mat& input, cv::Mat& output);
    
    // 提取白色区域（用于数字检测）
    void extractWhiteRegion(const cv::Mat& input, cv::Mat& output);
    
    // 检测灯条（不考虑颜色，只检测亮度）
    std::vector<cv::RotatedRect> detectLightBars(const cv::Mat& frame);
    
    // 在相邻灯条之间搜索数字3
    std::vector<ArmorPlate> findDigit3BetweenLightBars(const cv::Mat& frame, 
                                                      const std::vector<cv::RotatedRect>& lightBars);
    
    // 数字识别（使用训练好的resnet_model.onnx）
    int recognizeDigit(const cv::Mat& roi);
    
    // 计算两个灯条之间的距离
    float calculateDistance(const cv::RotatedRect& bar1, const cv::RotatedRect& bar2);
    
    // 在指定区域内搜索数字3
    cv::Rect findDigit3InRegion(const cv::Mat& frame, const cv::Rect& searchRegion);
};

#endif // ARMORDETECTOR_H