/**
 * @file FocusAnalyzer.cpp
 * @brief 散焦分析器实现文件
 */

#include "FocusAnalyzer.h"
#include <iostream>
#include <chrono>
namespace MicroscopeVision {

bool FocusAnalyzer::convertToGray(const cv::Mat& src, cv::Mat& dst) {
    if (src.empty()) return false;
    if (src.channels() == 1) {
        src.copyTo(dst);
    } else if (src.channels() == 3) {
        cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
    } else if (src.channels() == 4) {
        cv::cvtColor(src, dst, cv::COLOR_BGRA2GRAY);
    } else {
        return false;
    }
    return true;
}

FocusMetrics FocusAnalyzer::computeGradientEnergy(const cv::Mat& inputImg, const cv::Rect& roi, bool useGaussianPreprocess) {
    FocusMetrics result;
    auto start = std::chrono::high_resolution_clock::now();

    // 1. 基础校验
    if (inputImg.empty()) {
        std::cerr << "[Error] Input image is empty." << std::endl;
        return result;
    }

    // 2. 确定实际的处理区域 (Effective ROI)
    cv::Mat processingSource;

    if (roi.area() > 0) {
        // 安全性检查：计算传入 ROI 与 图像边界 的交集
        // 防止用户传入的 ROI 越界导致程序崩溃 (Segfault)
        cv::Rect imageBounds(0, 0, inputImg.cols, inputImg.rows);
        cv::Rect intersection = roi & imageBounds;

        if (intersection.area() == 0) {
            std::cerr << "[Error] ROI is outside of image bounds." << std::endl;
            return result; // isValid 默认为 false
        }

        // 创建 ROI header (这是 O(1) 操作，不产生深拷贝，只引用数据)
        processingSource = inputImg(intersection);
    } else {
        // ROI 为空，则处理全图
        processingSource = inputImg;
    }

    // 3. 预处理：转灰度
    // 注意：此时只转换 ROI 区域，大大节省算力
    cv::Mat grayImg;
    if (!convertToGray(processingSource, grayImg)) {
        std::cerr << "[Error] Unsupported image format." << std::endl;
        return result;
    }

    // 计算区域亮度
    cv::Scalar meanScalar = cv::mean(grayImg);
    result.meanIntensity = meanScalar[0];

    // 4. 预处理：降噪
    cv::Mat processedImg;
    if (useGaussianPreprocess) {
        cv::GaussianBlur(grayImg, processedImg, cv::Size(3, 3), 0);
    } else {
        processedImg = grayImg;
    }

    // 5. 核心算法：梯度能量 (Sobel)
    cv::Mat gradX, gradY;

    // 使用 CV_64F 保证精度，防止平方和溢出
    cv::Sobel(processedImg, gradX, CV_64F, 1, 0, 3);
    cv::Sobel(processedImg, gradY, CV_64F, 0, 1, 3);

    cv::Mat gradMagSqX, gradMagSqY, energyMap;
    cv::pow(gradX, 2, gradMagSqX);
    cv::pow(gradY, 2, gradMagSqY);
    cv::add(gradMagSqX, gradMagSqY, energyMap);

    // 6. 计算评分 (Mean Energy)
    cv::Scalar meanEnergy = cv::mean(energyMap);
    result.sharpnessScore = meanEnergy[0];
    result.isValid = true;

    // 7. 统计耗时
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    result.processTimeMs = elapsed.count();

    return result;
}

} // namespace MicroscopeVision
