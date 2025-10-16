#include "NeuralNetwork.h"
#include <iostream>

NeuralNetwork::NeuralNetwork() : inputSize(224, 224) {}

bool NeuralNetwork::loadModel(const std::string& modelPath) {
    try {
        std::cout << "Loading model from: " << modelPath << std::endl;
        net = cv::dnn::readNetFromONNX(modelPath);
        
        if (net.empty()) {
            std::cerr << "Error: Failed to load model from " << modelPath << std::endl;
            return false;
        }
        
        // 设置推理后端
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        
        // 尝试使用CUDA（如果可用）
        #ifdef HAVE_OPENCV_CUDA
        try {
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            std::cout << "Using CUDA backend" << std::endl;
        } catch (const cv::Exception& e) {
            std::cout << "CUDA not available, using CPU" << std::endl;
        }
        #endif
        
        std::cout << "Model loaded successfully: " << modelPath << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        return false;
    }
}

void NeuralNetwork::preprocess(const cv::Mat& input, cv::Mat& output) {
    if (input.empty()) {
        std::cout << "Error: Input image is empty in preprocess" << std::endl;
        output = cv::Mat();
        return;
    }
    
    try {
        cv::Mat resized;
        cv::resize(input, resized, inputSize);
        
        // 转换为RGB（如果输入是BGR）
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
        
        // 转换为浮点数并归一化到[0,1]
        cv::Mat floatImage;
        rgb.convertTo(floatImage, CV_32F, 1.0/255.0);
        
        // 手动应用ImageNet标准化
        std::vector<cv::Mat> channels;
        cv::split(floatImage, channels);
        
        // ImageNet标准化参数
        float mean[] = {0.485, 0.456, 0.406};
        float std[] = {0.229, 0.224, 0.225};
        
        for (int i = 0; i < 3; i++) {
            channels[i] = (channels[i] - mean[i]) / std[i];
        }
        
        cv::merge(channels, floatImage);
        
        // 转换为NCHW格式 [1, 3, 224, 224]
        output = cv::dnn::blobFromImage(floatImage);
    } catch (const cv::Exception& e) {
        std::cout << "Preprocessing error: " << e.what() << std::endl;
        output = cv::Mat();
    }
}
bool NeuralNetwork::inference(const cv::Mat& input, cv::Mat& output) {
    if (net.empty()) {
        std::cerr << "Error: Model not loaded" << std::endl;
        return false;
    }
    
    try {
        // 设置输入
        net.setInput(input);
        
        // 前向传播
        cv::Mat result = net.forward();
        
        // 处理输出
        if (!result.empty()) {
            output = result.clone();
            return true;
        }
        
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Inference error: " << e.what() << std::endl;
        return false;
    }
}