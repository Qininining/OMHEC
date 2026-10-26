#include "TargetDectect.h"
#include <QDebug>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

cv::Mat toGray8(const cv::Mat& img)
{
    if (img.empty()) {
        return cv::Mat();
    }

    cv::Mat gray;
    if (img.channels() == 1) {
        if (img.type() == CV_8UC1) {
            gray = img.clone();
        } else {
            img.convertTo(gray, CV_8UC1);
        }
        return gray;
    }

    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else if (img.channels() == 4) {
        cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
    } else {
        img.convertTo(gray, CV_8UC1);
    }

    if (gray.empty()) {
        return gray;
    }

    if (gray.type() != CV_8UC1) {
        cv::Mat normalized;
        cv::normalize(gray, normalized, 0, 255, cv::NORM_MINMAX);
        normalized.convertTo(gray, CV_8UC1);
    }

    return gray;
}

struct CircleDetectionResult {
    cv::Vec3d circle = cv::Vec3d(-1, -1, -1);
    double score = -1.0;
};

double contourCircularity(const std::vector<cv::Point>& contour)
{
    const double area = std::abs(cv::contourArea(contour));
    const double perimeter = cv::arcLength(contour, true);
    if (area <= 0.0 || perimeter <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    return 4.0 * CV_PI * area / (perimeter * perimeter);
}

CircleDetectionResult detectBestCircleFromBinary(const cv::Mat& binaryImage, const cv::Point& offset = cv::Point())
{
    CircleDetectionResult best;
    if (binaryImage.empty()) {
        return best;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double totalPixels = static_cast<double>(binaryImage.total());
    for (const auto& contour : contours) {
        const double area = std::abs(cv::contourArea(contour));
        if (area < 50.0) {
            continue;
        }

        const double areaRatio = area / totalPixels;
        if (areaRatio < 0.0001 || areaRatio > 0.35) {
            continue;
        }

        // 非均匀照明经全局阈值分割后常形成贴着图像边缘的大块背景，
        // 它不可能是视野内部的白色标定圆斑。
        const cv::Rect bounds = cv::boundingRect(contour);
        if (bounds.x <= 1 || bounds.y <= 1 ||
            bounds.x + bounds.width >= binaryImage.cols - 1 ||
            bounds.y + bounds.height >= binaryImage.rows - 1) {
            continue;
        }

        cv::Moments moments = cv::moments(contour);
        if (std::abs(moments.m00) <= std::numeric_limits<double>::epsilon()) {
            continue;
        }

        const double circularity = contourCircularity(contour);
        // 失焦圆斑的阈值轮廓会变钝甚至略有变形，圆度阈值不宜过严。
        if (circularity < 0.25) {
            continue;
        }

        cv::Point2f center;
        float radius = 0.0f;
        cv::minEnclosingCircle(contour, center, radius);

        const double centroidX = moments.m10 / moments.m00 + offset.x;
        const double centroidY = moments.m01 / moments.m00 + offset.y;
        const double finalRadius = std::max(1.0f, radius);

        // 视野中只有一个明显白圆，面积应比轮廓的轻微圆度变化更可靠。
        const double score = area * (0.5 + 0.5 * std::min(1.0, circularity));
        if (score > best.score) {
            best.score = score;
            best.circle = cv::Vec3d(centroidX, centroidY, finalRadius);
        }
    }

    return best;
}

CircleDetectionResult detectBestCircleCandidate(const cv::Mat& grayImage)
{
    CircleDetectionResult best;
    if (grayImage.empty()) {
        return best;
    }

    cv::Mat blurred;
    // 较强平滑可连接失焦圆斑内部，同时抑制传感器散斑和孤立亮点。
    cv::GaussianBlur(grayImage, blurred, cv::Size(9, 9), 0.0);

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Scalar meanValue, stdValue;
    cv::meanStdDev(blurred, meanValue, stdValue);
    const double statisticalThreshold =
        std::min(254.0, meanValue[0] + 0.75 * stdValue[0]);

    double minimumValue = 0.0;
    double maximumValue = 0.0;
    cv::minMaxLoc(blurred, &minimumValue, &maximumValue);
    const double intensityRange = maximumValue - minimumValue;

    // Otsu和统计阈值处理一般图像；后三个阈值直接截取动态范围顶部，
    // 专门应对带大范围照明渐变的近饱和白圆。
    const double thresholds[] = {
        -1.0,
        statisticalThreshold,
        minimumValue + 0.70 * intensityRange,
        minimumValue + 0.80 * intensityRange,
        minimumValue + 0.90 * intensityRange
    };
    for (double thresholdValue : thresholds) {
        cv::Mat binary;
        if (thresholdValue < 0.0)
            cv::threshold(blurred, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        else
            cv::threshold(blurred, binary, thresholdValue, 255, cv::THRESH_BINARY);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

        CircleDetectionResult candidate = detectBestCircleFromBinary(binary);
        if (candidate.score > best.score) {
            best = candidate;
        }
    }

    return best;
}

} // namespace

cv::Vec3d TargetDectect::checkBestCircle(cv::Mat img) {
    const cv::Mat gray = toGray8(img);
    const CircleDetectionResult result = detectBestCircleCandidate(gray);
    return result.circle;
}



cv::Vec3d TargetDectect::checkBestCircleM(cv::Mat img) {
    const cv::Mat gray = toGray8(img);
    const CircleDetectionResult result = detectBestCircleCandidate(gray);
    if (result.score < 0.0) {
        return cv::Vec3d(-1, -1, -1);
    }
    return cv::Vec3d(result.circle[0], result.circle[1], 200);
}


cv::Vec3d TargetDectect::findPreciseCenter(const cv::Mat& grayImage_8U, const cv::Vec3d& coarseCircle) {
    // 1. 从粗略结果中提取信息
    double u = coarseCircle[0];
    double v = coarseCircle[1];
    double r = coarseCircle[2];

    // 2. 定义一个围绕 (u, v) 的ROI
    // 我们使用一个比半径稍小的方形区域（例如 1.5 * r），以确保我们只关注圆的内部
    int roiHalfSize = static_cast<int>(r * 1.5);

    // 确保ROI有合理的最小尺寸
    if (roiHalfSize < 30) roiHalfSize = 30;

    // 3. 计算ROI坐标，并进行严格的边界检查
    int x_start = (std::max)(0, static_cast<int>(u - roiHalfSize));
    int y_start = (std::max)(0, static_cast<int>(v - roiHalfSize));
    int x_end = (std::min)(grayImage_8U.cols, static_cast<int>(u + roiHalfSize));
    int y_end = (std::min)(grayImage_8U.rows, static_cast<int>(v + roiHalfSize));

    cv::Rect roiRect(x_start, y_start, x_end - x_start, y_end - y_start);

    // 检查ROI是否有效
    if (roiRect.width <= 0 || roiRect.height <= 0) {
        qWarning() << "Invalid ROI generated in findPreciseCenter. Returning coarse center.";
        return coarseCircle; // 发生错误，返回粗略结果
    }

    cv::Mat roiImage = grayImage_8U(roiRect);

    // 以ROI边缘像素的平均值近似局部背景；对白圆超出背景的亮度求加权
    // 质心。即使轮廓因失焦而模糊，该中心仍比二值轮廓矩稳定。
    const int border = std::max(1, std::min(5, std::min(roiImage.rows, roiImage.cols) / 4));
    cv::Mat borderMask(roiImage.size(), CV_8U, cv::Scalar(255));
    if (roiImage.cols > 2 * border && roiImage.rows > 2 * border)
        borderMask(cv::Rect(border, border,
                            roiImage.cols - 2 * border,
                            roiImage.rows - 2 * border)).setTo(0);
    const double background = cv::mean(roiImage, borderMask)[0];

    cv::Mat weights;
    roiImage.convertTo(weights, CV_64F);
    weights -= background;
    cv::max(weights, 0.0, weights);
    const cv::Moments weightedMoments = cv::moments(weights, false);
    if (weightedMoments.m00 <= std::numeric_limits<double>::epsilon()) {
        return coarseCircle;
    }
    return cv::Vec3d(weightedMoments.m10 / weightedMoments.m00 + roiRect.x,
                     weightedMoments.m01 / weightedMoments.m00 + roiRect.y,
                     r);
}

cv::Vec3d TargetDectect::findPreciseCenterHighMagnification(
    const cv::Mat& grayImage_8U, const cv::Vec3d& coarseCircle) {
    if (grayImage_8U.empty() || coarseCircle[0] < 0.0 || coarseCircle[1] < 0.0 ||
        coarseCircle[2] <= 0.0) {
        return cv::Vec3d(-1, -1, -1);
    }

    // 10X/20X下圆斑边缘可能带有不均匀光晕。单次灰度质心容易被光晕拉偏，
    // 因此在多个高阈值下定位内层亮圆，并取合格轮廓中心的中位数。
    const double radius = coarseCircle[2];
    const int halfSize = std::max(40, cvRound(radius * 1.35));
    const cv::Rect imageBounds(0, 0, grayImage_8U.cols, grayImage_8U.rows);
    const cv::Rect requested(cvRound(coarseCircle[0]) - halfSize,
                             cvRound(coarseCircle[1]) - halfSize,
                             2 * halfSize + 1, 2 * halfSize + 1);
    const cv::Rect roiRect = requested & imageBounds;
    if (roiRect.width < 20 || roiRect.height < 20) {
        return findPreciseCenter(grayImage_8U, coarseCircle);
    }

    cv::Mat roi = grayImage_8U(roiRect);
    cv::Mat smooth;
    cv::GaussianBlur(roi, smooth, cv::Size(5, 5), 0.0);

    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(smooth, &minValue, &maxValue);
    if (maxValue - minValue < 8.0) {
        return findPreciseCenter(grayImage_8U, coarseCircle);
    }

    std::vector<double> centerXs;
    std::vector<double> centerYs;
    const double levels[] = {0.62, 0.72, 0.80, 0.87, 0.92};
    const cv::Point2d expected(coarseCircle[0] - roiRect.x,
                               coarseCircle[1] - roiRect.y);

    for (double level : levels) {
        cv::Mat binary;
        const double thresholdValue = minValue + level * (maxValue - minValue);
        cv::threshold(smooth, binary, thresholdValue, 255, cv::THRESH_BINARY);
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double bestScore = -1.0;
        cv::Point2d bestCenter;
        for (const auto& contour : contours) {
            const double area = std::abs(cv::contourArea(contour));
            if (area < 20.0 || area > CV_PI * radius * radius * 1.6) {
                continue;
            }
            const double circularity = contourCircularity(contour);
            if (circularity < 0.45) {
                continue;
            }
            const cv::Moments moments = cv::moments(contour);
            if (std::abs(moments.m00) <= std::numeric_limits<double>::epsilon()) {
                continue;
            }
            const cv::Point2d center(moments.m10 / moments.m00,
                                     moments.m01 / moments.m00);
            const double distance = cv::norm(center - expected);
            if (distance > std::max(12.0, radius * 0.45)) {
                continue;
            }
            const double score = circularity * std::sqrt(area) / (1.0 + distance);
            if (score > bestScore) {
                bestScore = score;
                bestCenter = center;
            }
        }

        if (bestScore >= 0.0) {
            centerXs.push_back(bestCenter.x + roiRect.x);
            centerYs.push_back(bestCenter.y + roiRect.y);
        }
    }

    if (centerXs.size() < 3) {
        return findPreciseCenter(grayImage_8U, coarseCircle);
    }

    const size_t middle = centerXs.size() / 2;
    std::nth_element(centerXs.begin(), centerXs.begin() + middle, centerXs.end());
    std::nth_element(centerYs.begin(), centerYs.begin() + middle, centerYs.end());
    return cv::Vec3d(centerXs[middle], centerYs[middle], radius);
}
