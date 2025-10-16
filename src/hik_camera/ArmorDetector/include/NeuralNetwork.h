#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>

class NeuralNetwork {
public:
    NeuralNetwork();
    
    // 加载模型（支持ONNX格式）
    bool loadModel(const std::string& modelPath);
    
    // 推理
    bool inference(const cv::Mat& input, cv::Mat& output);
    
    // 预处理输入图像
    void preprocess(const cv::Mat& input, cv::Mat& output);
    
private:
    cv::dnn::Net net;
    cv::Size inputSize;
};

#endif // NEURALNETWORK_H
