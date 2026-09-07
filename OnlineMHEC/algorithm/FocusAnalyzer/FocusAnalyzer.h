/**
 * @file FocusAnalyzer.h
 * @brief 基于梯度能量法的显微图像散焦分析器 (支持 ROI)
 * @author Experienced Vision Engineer
 * @date 2025-11-29
 */

#ifndef FOCUS_ANALYZER_H
#define FOCUS_ANALYZER_H

#include <opencv2/opencv.hpp>

namespace MicroscopeVision {

struct FocusMetrics {
    double sharpnessScore;      // 清晰度评分
    double meanIntensity;       // 区域平均亮度
    bool isValid;
    double processTimeMs;

    FocusMetrics() : sharpnessScore(0.0), meanIntensity(0.0), isValid(false), processTimeMs(0.0) {}
};

class FocusAnalyzer {
public:
    /**
         * @brief 计算图像的梯度能量（清晰度）
         * * @param inputImg 输入图像 (支持 CV_8U, CV_16U, 彩色或灰度)
         * @param roi      感兴趣区域 (Region of Interest)。
         * 默认为 cv::Rect() (空矩形)，此时将处理整张图像。
         * 如果传入有效矩形，则仅计算该区域内的梯度。
         * @param useGaussianPreprocess 是否使用高斯模糊预处理 (推荐 True)
         * @return FocusMetrics
         */
    static FocusMetrics computeGradientEnergy(
        const cv::Mat& inputImg,
        const cv::Rect& roi = cv::Rect(),
        bool useGaussianPreprocess = true
        );

private:
    // 辅助函数：转灰度
    static bool convertToGray(const cv::Mat& src, cv::Mat& dst);
};

} // namespace MicroscopeVision

#endif // FOCUS_ANALYZER_H
