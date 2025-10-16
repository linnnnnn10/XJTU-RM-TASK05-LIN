#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ArmorDetector.h"
#include "NeuralNetwork.h"
#include "CameraCalibrator.h"
#include "PnPSolver.h"

class ArmorDetectionNode : public rclcpp::Node {
public:
    ArmorDetectionNode() : Node("armor_detection_node") {
        // 声明参数
        this->declare_parameter<std::string>("camera_topic", "/hik_camera/image_raw");
        this->declare_parameter<std::string>("calibration_file", "/home/linnnnnn/ros2_ws/src/hik_camera/ArmorDetector/data/calibration_params.yml");
        this->declare_parameter<std::string>("model_path", "/home/linnnnnn/ros2_ws/src/hik_camera/ArmorDetector/data/model/resnet_model.onnx");
        this->declare_parameter<bool>("use_calibration", true);
        
        // 获取参数
        camera_topic_ = this->get_parameter("camera_topic").as_string();
        calibration_file_ = this->get_parameter("calibration_file").as_string();
        model_path_ = this->get_parameter("model_path").as_string();
        use_calibration_ = this->get_parameter("use_calibration").as_bool();
        
        // 初始化模块
        if (!initializeModules()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize modules");
            rclcpp::shutdown();
            return;
        }
        
        // 创建图像订阅器
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10,
            std::bind(&ArmorDetectionNode::imageCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Armor Detection Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", camera_topic_.c_str());
    }

private:
    bool initializeModules() {
        // 初始化神经网络
        neural_network_ = std::make_shared<NeuralNetwork>();
        if (!neural_network_->loadModel(model_path_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load neural network model from: %s", model_path_.c_str());
            return false;
        }
        
        // 初始化装甲板检测器
        detector_ = std::make_shared<ArmorDetector>();
        if (!detector_->setNeuralNetwork(neural_network_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set neural network for detector!");
            return false;
        }
        
        // 初始化相机标定器
        calibrator_ = std::make_shared<CameraCalibrator>();
        if (use_calibration_) {
            if (calibrator_->loadCalibrationParams(calibration_file_)) {
                RCLCPP_INFO(this->get_logger(), "Camera calibration parameters loaded successfully");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to load calibration file, running without calibration");
                use_calibration_ = false;
            }
        }
        
        // 初始化PnP求解器
        pnp_solver_ = std::make_shared<PnPSolver>();
        if (use_calibration_) {
            pnp_solver_->setCameraParams(calibrator_->getCameraMatrix(), 
                                       calibrator_->getDistCoeffs());
            RCLCPP_INFO(this->get_logger(), "PnP solver initialized with camera parameters");
        }
        
        // 设置装甲板3D模型尺寸（单位：米）
        pnp_solver_->setArmorModel(0.135f, 0.055f);
        
        RCLCPP_INFO(this->get_logger(), "All modules initialized successfully");
        return true;
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto start_time = std::chrono::steady_clock::now();
            
            // 转换ROS图像消息为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty image");
                return;
            }
            
            // 处理图像
            processFrame(frame);
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Processing exception: %s", e.what());
        }
    }
    
    void processFrame(cv::Mat& frame) {
        // 畸变校正（如果启用了标定）
        cv::Mat processed_frame;
        if (use_calibration_) {
            calibrator_->undistortImage(frame, processed_frame);
        } else {
            processed_frame = frame.clone();
        }
        
        // 装甲板检测
        auto armors = detector_->detectArmor(processed_frame);
        
        // 对每个检测到的装甲板进行PnP计算
        for (auto& armor : armors) {
            if (armor.corners.size() == 4) {
                cv::Mat rvec, tvec;
                if (pnp_solver_->solvePnP(armor.corners, rvec, tvec)) {
                    // 获取相机坐标系下的坐标
                    cv::Point3f camera_coords = pnp_solver_->getCameraCoordinates(tvec);
                    
                    // 在图像上显示信息
                    displayArmorInfo(processed_frame, armor, camera_coords);
                    
                    RCLCPP_DEBUG(this->get_logger(), "Armor detected at (%.2f, %.2f, %.2f)", 
                               camera_coords.x, camera_coords.y, camera_coords.z);
                }
            }
        }
        
        // 绘制检测结果
        detector_->drawResults(processed_frame, armors);
        
        // 显示帧率和检测数量
        displayStats(processed_frame, armors.size());
        
        // 显示图像
        cv::imshow("Armor Detection - Real Time", processed_frame);
        
        // 处理键盘输入
        handleKeyboardInput();
    }
    
    void displayArmorInfo(cv::Mat& frame, const ArmorPlate& armor, const cv::Point3f& coords) {
        // 显示距离信息
        std::string distance_text = "Dist: " + std::to_string(coords.z).substr(0, 4) + "m";
        cv::putText(frame, distance_text, 
                   armor.boundingRect.tl() + cv::Point(0, -20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        
        // 显示3D坐标
        std::string coord_text = "(" + 
            std::to_string(coords.x).substr(0, 4) + ", " +
            std::to_string(coords.y).substr(0, 4) + ", " +
            std::to_string(coords.z).substr(0, 4) + ")";
        cv::putText(frame, coord_text,
                   armor.boundingRect.tl() + cv::Point(0, -40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
    }
    
    void displayStats(cv::Mat& frame, size_t armor_count) {
        // 计算帧率
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time_);
        if (duration.count() > 0) {
            double fps = 1000.0 / duration.count();
            std::string fps_text = "FPS: " + std::to_string(fps).substr(0, 4);
            cv::putText(frame, fps_text, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
        }
        last_time_ = current_time;
        
        std::string armor_count_text = "Armors: " + std::to_string(armor_count);
        cv::putText(frame, armor_count_text, cv::Point(10, 70),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
    }
    
    void handleKeyboardInput() {
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // 'q' 或 ESC 退出
            RCLCPP_INFO(this->get_logger(), "Exiting by user request");
            rclcpp::shutdown();
        } else if (key == 'c') { // 'c' 切换标定模式
            use_calibration_ = !use_calibration_;
            RCLCPP_INFO(this->get_logger(), "Calibration %s", use_calibration_ ? "enabled" : "disabled");
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<NeuralNetwork> neural_network_;
    std::shared_ptr<ArmorDetector> detector_;
    std::shared_ptr<CameraCalibrator> calibrator_;
    std::shared_ptr<PnPSolver> pnp_solver_;
    
    std::string camera_topic_;
    std::string calibration_file_;
    std::string model_path_;
    bool use_calibration_;
    
    std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建显示窗口
    cv::namedWindow("Armor Detection - Real Time", cv::WINDOW_NORMAL);
    cv::resizeWindow("Armor Detection - Real Time", 800, 600);
    
    auto node = std::make_shared<ArmorDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    cv::destroyAllWindows();
    return 0;
}