#include "ArmorDetector.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

ArmorDetector::ArmorDetector() {}

bool ArmorDetector::setNeuralNetwork(std::shared_ptr<NeuralNetwork> nn) {
    neuralNetwork = nn;
    return neuralNetwork != nullptr;
}

std::vector<ArmorPlate> ArmorDetector::detectArmor(const cv::Mat& frame) {
    std::vector<ArmorPlate> armors;
    digit3Regions.clear();
    
    if (frame.empty()) {
        std::cout << "Error: Input frame is empty!" << std::endl;
        return armors;
    }
    
    // 图像预处理
    cv::Mat processed;
    preprocessFrame(frame, processed);
    
    if (processed.empty()) {
        std::cout << "Error: Preprocessed frame is empty!" << std::endl;
        return armors;
    }
    
    // 检测灯条（基于亮度）
    auto lightBars = detectLightBars(processed);
    std::cout << "Detected " << lightBars.size() << " light bars" << std::endl;
    
    // 绘制检测到的灯条用于调试
    cv::Mat debugFrame = frame.clone();
    for (const auto& bar : lightBars) {
        cv::Point2f vertices[4];
        bar.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(debugFrame, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 2);
        }
    }
    cv::imwrite("debug.jpg", debugFrame);
    // 在相邻灯条之间搜索数字3
    if (!lightBars.empty()) {
        armors = findDigit3BetweenLightBars(frame, lightBars);
    } else {
        std::cout << "No light bars detected" << std::endl;
    }
    
    std::cout << "Found " << armors.size() << " armors" << std::endl;
    
    return armors;
}

void ArmorDetector::preprocessFrame(const cv::Mat& input, cv::Mat& output) {
    if (input.empty()) {
        output = cv::Mat();
        return;
    }
    
    try {
        cv::Mat blurred;
        cv::GaussianBlur(input, blurred, cv::Size(3, 3), 0);
        output = blurred;
    } catch (const cv::Exception& e) {
        std::cout << "Preprocessing error: " << e.what() << std::endl;
        output = input.clone();
    }
}

void ArmorDetector::extractWhiteRegion(const cv::Mat& input, cv::Mat& output) {
    if (input.empty()) {
        output = cv::Mat();
        return;
    }
    
    try {
        cv::Mat hsv;
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
        //白色参数
        cv::Scalar lowerWhite(0, 0, 50);
        cv::Scalar upperWhite(180, 80, 200); 
        
        cv::Mat whiteMaskHsv;
        cv::inRange(hsv, lowerWhite, upperWhite, whiteMaskHsv);
        cv::Mat gray;
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
        cv::Mat whiteMaskGray;
        cv::threshold(gray, whiteMaskGray, 50, 200, cv::THRESH_BINARY);
        
        cv::bitwise_or(whiteMaskHsv, whiteMaskGray, output);
        
        // 形态学操作
        if (!output.empty()) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
            cv::morphologyEx(output, output, cv::MORPH_OPEN, kernel);
        }
    } catch (const cv::Exception& e) {
        std::cout << "White region extraction error: " << e.what() << std::endl;
        output = cv::Mat();
    }
}

std::vector<cv::RotatedRect> ArmorDetector::detectLightBars(const cv::Mat& frame) {
    std::vector<cv::RotatedRect> lightBars;
    
    if (frame.empty()) {
        return lightBars;
    }
    
    try {
        // 2. 亮度过滤：提取高亮区域
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::Mat combined_mask;
        cv::threshold(gray, combined_mask, 150, 255, cv::THRESH_BINARY);
        
        // 4. 形态学操作（优化掩码）
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel);
        
        // 5. 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            if (contour.size() < 5) continue;
            
            cv::RotatedRect rect = cv::minAreaRect(contour);
            float area = rect.size.width * rect.size.height;
            
            // 计算短边和长边
            float width = std::min(rect.size.width, rect.size.height);
            float height = std::max(rect.size.width, rect.size.height);
            float aspectRatio = height / width;
            
            // 计算长边与水平线的夹角（修正版）
            float theta;
            
            // OpenCV 的 RotatedRect.angle 范围通常是 [-90, 0]
            // 我们需要计算长边与水平线的夹角
            if (rect.size.width >= rect.size.height) {
                // 宽度是长边，角度就是 rect.angle 的绝对值
                theta = std::abs(rect.angle);
            } else {
                // 高度是长边，角度需要调整
                theta = std::abs(rect.angle + 90.0f);
            }
            
            // 规范化角度到 [0, 90] 范围
            if (theta > 90.0f) {
                theta = 180.0f - theta;
            }
            if (theta > 90.0f) {
                theta = 90.0f - (theta - 90.0f);
            }
            
            float min_vertical_angle = 60.0f;  // 最小垂直角度阈值
            float max_horizontal_angle = 15.0f; // 最大水平角度阈值（要过滤的）
            
            if (area > 200  && 
                aspectRatio >= 3.0&&
                theta >= min_vertical_angle) {  // 只保留足够垂直的灯条
                lightBars.push_back(rect);
            }
        }
        
        std::cout << "Filtered " << lightBars.size() << " light bars" << std::endl;
    } catch (const cv::Exception& e) {
        std::cout << "Light bar detection error: " << e.what() << std::endl;
    }
    
    return lightBars;
}
cv::Rect ArmorDetector::findDigit3InRegion(const cv::Mat& frame, const cv::Rect& searchRegion) {
    if (searchRegion.width <= 0 || searchRegion.height <= 0) {
        return cv::Rect();
    }
    
    cv::Rect safeRegion = searchRegion & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safeRegion.width <= 0 || safeRegion.height <= 0) {
        return cv::Rect();
    }
    
    cv::Mat roi = frame(safeRegion);
    
    // 提取白色区域
    cv::Mat whiteMask;
    extractWhiteRegion(roi, whiteMask);
    
    if (whiteMask.empty()) {
        return cv::Rect();
    }
    
    // 在白色区域内寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(whiteMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        if (contour.size() < 5) continue;
        
        // 计算轮廓的边界矩形
        cv::Rect rect = cv::boundingRect(contour);
        
        // 筛选可能的数字区域（根据宽高比和面积）
        float aspectRatio = (float)rect.width / rect.height;
        float area = rect.width * rect.height;
        
        if (area < 1000) continue;  // 面积过滤
        if (aspectRatio < 0.3 || aspectRatio > 1.0) continue;  // 宽高比过滤
        
        // 提取数字区域进行识别
        cv::Mat digitROI = roi(rect);
        
        // 确保ROI有效
        if (digitROI.empty() || digitROI.cols < 10 || digitROI.rows < 10) {
            continue;
        }
        
        // 数字识别
        int digit = recognizeDigit(digitROI);
        
        if (digit == 3) {
            // 返回在原始图像中的坐标
            return cv::Rect(safeRegion.x + rect.x, safeRegion.y + rect.y, 
                           rect.width, rect.height);
        }
    }
    
    return cv::Rect();
}
std::vector<ArmorPlate> ArmorDetector::findDigit3BetweenLightBars(const cv::Mat& frame, 
                                                                  const std::vector<cv::RotatedRect>& lightBars) {
    std::vector<ArmorPlate> armors;
    
    if (lightBars.size() < 2) {
        return armors;
    }
    
    // 按灯条中心x坐标排序
    std::vector<cv::RotatedRect> sortedBars = lightBars;
    std::sort(sortedBars.begin(), sortedBars.end(), 
              [](const cv::RotatedRect& a, const cv::RotatedRect& b) {
                  return a.center.x < b.center.x;
              });
    
    // 遍历所有相邻的灯条对
    for (size_t i = 0; i < sortedBars.size() - 1; ++i) {
        const auto& leftBar = sortedBars[i];
        const auto& rightBar = sortedBars[i + 1];
        
        // 计算两个灯条之间的搜索区域
        cv::Point2f leftCenter = leftBar.center;
        cv::Point2f rightCenter = rightBar.center;
        
        // 扩大搜索区域
        int minX = static_cast<int>(std::min(leftCenter.x, rightCenter.x));
        int maxX = static_cast<int>(std::max(leftCenter.x, rightCenter.x));
        int minY = static_cast<int>(std::min(leftCenter.y, rightCenter.y) - 300); // 上下扩展100像素
        int maxY = static_cast<int>(std::max(leftCenter.y, rightCenter.y) + 300);
        
        cv::Rect searchRegion(minX, minY, maxX - minX, maxY - minY);
        
        std::cout << "Searching for digit 3 in region: (" << searchRegion.x << ", " 
                  << searchRegion.y << ") " << searchRegion.width << "x" << searchRegion.height << std::endl;
        
        // 在搜索区域中查找数字3
        cv::Rect digitRect = findDigit3InRegion(frame, searchRegion);
        
        if (!digitRect.empty()) {
            // 获取灯条的边界矩形
            cv::Rect leftRect = leftBar.boundingRect();
            cv::Rect rightRect = rightBar.boundingRect();
            
            // 计算包含数字和两个灯条的边界框
            cv::Rect armorRect = leftRect | rightRect;
            armorRect |= digitRect;
            
            // 确保边界框在图像范围内
            armorRect = armorRect & cv::Rect(0, 0, frame.cols, frame.rows);
            
            // 稍微扩大边界框
            armorRect.x = std::max(0, armorRect.x - 5);
            armorRect.y = std::max(0, armorRect.y - 5);
            armorRect.width = std::min(frame.cols - armorRect.x, armorRect.width + 10);
            armorRect.height = std::min(frame.rows - armorRect.y, armorRect.height + 10);

            // ↗️ 新增条件：装甲板面积不大于数字3区域面积的三倍
            float armorArea = armorRect.area(); // 装甲板面积[1,2](@ref)
            float digitArea = digitRect.area(); // 数字3区域面积
            
            std::cout << "Armor area: " << armorArea << ", Digit area: " << digitArea 
                      << ", Ratio: " << (armorArea / digitArea) << std::endl;
            
            // 检查装甲板面积是否超过数字区域面积的三倍
            if (armorArea > digitArea * 3.0f) {
                std::cout << "Armor area too large, skipping. Ratio: " 
                          << (armorArea / digitArea) << " > 3.0" << std::endl;
                continue; // 跳过这个装甲板
            }
            
            // 创建装甲板
            ArmorPlate armor;
            armor.digitRect = digitRect;
            armor.digit = 3;
            armor.boundingRect = armorRect;
            
            // 计算装甲板的四个角点（用于PnP计算）
            armor.corners.clear();
            armor.corners.push_back(cv::Point2f(armorRect.x, armorRect.y));  // 左上
            armor.corners.push_back(cv::Point2f(armorRect.x + armorRect.width, armorRect.y));  // 右上
            armor.corners.push_back(cv::Point2f(armorRect.x + armorRect.width, armorRect.y + armorRect.height));  // 右下
            armor.corners.push_back(cv::Point2f(armorRect.x, armorRect.y + armorRect.height));  // 左下
            
            armors.push_back(armor);
            std::cout << "Found armor with digit 3 between light bars " << i << " and " << i+1 << std::endl;
        }
    }
    
    return armors;
}
/*std::vector<ArmorPlate> ArmorDetector::findDigit3BetweenLightBars(const cv::Mat& frame, 
                                                                  const std::vector<cv::RotatedRect>& lightBars) {
    std::vector<ArmorPlate> armors;
    
    if (lightBars.size() < 2) {
        return armors;
    }
    
    // 按灯条中心x坐标排序
    std::vector<cv::RotatedRect> sortedBars = lightBars;
    std::sort(sortedBars.begin(), sortedBars.end(), 
              [](const cv::RotatedRect& a, const cv::RotatedRect& b) {
                  return a.center.x < b.center.x;
              });
    
    // 遍历所有相邻的灯条对
    for (size_t i = 0; i < sortedBars.size() - 1; ++i) {
        const auto& leftBar = sortedBars[i];
        const auto& rightBar = sortedBars[i + 1];
        
        // 计算两个灯条之间的搜索区域
        cv::Point2f leftCenter = leftBar.center;
        cv::Point2f rightCenter = rightBar.center;
        
        // 扩大搜索区域
        int minX = static_cast<int>(std::min(leftCenter.x, rightCenter.x));
        int maxX = static_cast<int>(std::max(leftCenter.x, rightCenter.x));
        int minY = static_cast<int>(std::min(leftCenter.y, rightCenter.y) - 100); // 上下扩展100像素
        int maxY = static_cast<int>(std::max(leftCenter.y, rightCenter.y) + 100);
        
        cv::Rect searchRegion(minX, minY, maxX - minX, maxY - minY);
        
        std::cout << "Searching for digit 3 in region: (" << searchRegion.x << ", " 
                  << searchRegion.y << ") " << searchRegion.width << "x" << searchRegion.height << std::endl;
        
        // 在搜索区域中查找数字3
        cv::Rect digitRect = findDigit3InRegion(frame, searchRegion);
        
        if (!digitRect.empty()) {
            // 找到数字3，创建装甲板
            ArmorPlate armor;
            armor.digitRect = digitRect;
            armor.digit = 3;
            
            // 获取灯条的边界矩形
            cv::Rect leftRect = leftBar.boundingRect();
            cv::Rect rightRect = rightBar.boundingRect();
            
            // 计算包含数字和两个灯条的边界框
            cv::Rect armorRect = leftRect | rightRect;
            armorRect |= digitRect;
            
            // 确保边界框在图像范围内
            armorRect = armorRect & cv::Rect(0, 0, frame.cols, frame.rows);
            
            // 稍微扩大边界框
            armorRect.x = std::max(0, armorRect.x - 5);
            armorRect.y = std::max(0, armorRect.y - 5);
            armorRect.width = std::min(frame.cols - armorRect.x, armorRect.width + 10);
            armorRect.height = std::min(frame.rows - armorRect.y, armorRect.height + 10);

            armor.boundingRect = armorRect;
            // 计算装甲板的四个角点（用于PnP计算）
            armor.corners.clear();
            armor.corners.push_back(cv::Point2f(armorRect.x, armorRect.y));  // 左上
            armor.corners.push_back(cv::Point2f(armorRect.x + armorRect.width, armorRect.y));  // 右上
            armor.corners.push_back(cv::Point2f(armorRect.x + armorRect.width, armorRect.y + armorRect.height));  // 右下
            armor.corners.push_back(cv::Point2f(armorRect.x, armorRect.y + armorRect.height));  // 左下
            
            armors.push_back(armor);
            std::cout << "Found armor with digit 3 between light bars " << i << " and " << i+1 << std::endl;
        }
    }
    
    return armors;
}*/

int ArmorDetector::recognizeDigit(const cv::Mat& Roi) {
    if (!neuralNetwork) {
        std::cout << "Neural network not available" << std::endl;
        return -1;
    }
    
    if (Roi.empty()) {
        std::cout << "Error: Empty ROI for digit recognition" << std::endl;
        return -1;
    }
    cv::Mat roi, roi_gray;
    // 第一步：先将ROI区域转换为灰度图（单通道）
    cv::cvtColor(Roi, roi_gray, cv::COLOR_BGR2GRAY);

    // 第二步：对灰度图进行二值化
    cv::threshold(roi_gray, roi, 50, 255, cv::THRESH_BINARY);
    cv::imwrite("debug_roi.jpg", roi);
    try {
        cv::Mat processed;
        neuralNetwork->preprocess(roi, processed);
        
        if (processed.empty()) {
            std::cout << "Digit preprocessing failed: empty result" << std::endl;
            return -1;
        }
        
        cv::Mat output;
        if (neuralNetwork->inference(processed, output)) {
            
            // 获取logits值
            float* logits = output.ptr<float>(0);
            
            // 计算softmax概率
            float max_logit = *std::max_element(logits, logits + 6);
            float sum_exp = 0.0f;
            for (int i = 0; i < 6; i++) {
                sum_exp += std::exp(logits[i] - max_logit); // 数值稳定性
            }
            
            float probabilities[6];
            for (int i = 0; i < 6; i++) {
                probabilities[i] = std::exp(logits[i] - max_logit) / sum_exp;
            }
            
            std::cout << "result: ";
            std::cout << "1: " << probabilities[0] << " ";
            std::cout << "2: " << probabilities[1] << " ";
            std::cout << "3: " << probabilities[2] << " ";
            std::cout << "4: " << probabilities[3] << " ";
            std::cout << "5: " << probabilities[4] << " ";
            std::cout << "neg: " << probabilities[5] << std::endl;
            
            // 找到最大logits的索引
            int maxIndex = 0;
            float maxLogit = logits[0];
            for (int i = 1; i < 6; i++) {
                if (logits[i] > maxLogit) {
                    maxLogit = logits[i];
                    maxIndex = i;
                }
            }
            
            // 添加置信度阈值
            float confidence_threshold = 0.8f;
            
            // 如果最大索引是5（负样本），返回-1；否则返回数字1-5
            if (maxIndex == 5) {
                if (probabilities[5] > confidence_threshold) {
                    std::cout << "identified as a negative sample."<< std::endl;
                    return -1;
                } else {
                    // 如果负样本置信度不高，尝试找数字
                    int digitIndex = 0;
                    float maxDigitProb = probabilities[0];
                    for (int i = 1; i < 5; i++) {
                        if (probabilities[i] > maxDigitProb) {
                            maxDigitProb = probabilities[i];
                            digitIndex = i;
                        }
                    }
                    
                    if (maxDigitProb > confidence_threshold) {
                        int digit = digitIndex + 1;
                        std::cout << "identified as number " << digit << ": " << maxDigitProb << std::endl;
                        return digit;
                    } else {
                        std::cout << "identified as a negative sample." << std::endl;
                        return -1;
                    }
                }
            } else {
                int digit = maxIndex + 1;
                if (probabilities[maxIndex] > confidence_threshold) {
                    std::cout << "identified as number " << digit << ": " << probabilities[maxIndex] << std::endl;
                    return digit;
                } else {
                    std::cout << "identified as a negative sample." << std::endl;
                    return -1;
                }
            }
        }
        
        std::cout << "Digit recognition failed" << std::endl;
        return -1;
    } catch (const cv::Exception& e) {
        std::cout << "Digit recognition error: " << e.what() << std::endl;
        return -1;
    }
}
void ArmorDetector::drawResults(cv::Mat& frame, const std::vector<ArmorPlate>& armors) {
    if (frame.empty()) return;
    
    // 绘制装甲板（绿色框）
    for (const auto& armor : armors) {
        cv::rectangle(frame, armor.boundingRect, cv::Scalar(0, 255, 0), 2);
        std::string text = "Armor: " + std::to_string(armor.digit);
        cv::putText(frame, text, armor.boundingRect.tl() + cv::Point(0, -5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    
    // 标记数字3区域（红色框）
    for (const auto& armor : armors) {
        cv::rectangle(frame, armor.digitRect, cv::Scalar(0, 0, 255), 2);
        cv::putText(frame, "Digit 3", armor.digitRect.tl() + cv::Point(0, -5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }
    
    // 如果没有检测到任何装甲板，显示提示
    if (armors.empty()) {
        cv::putText(frame, "No armor detected", cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }
}