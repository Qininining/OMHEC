#ifndef TARGETDECTECT_H
#define TARGETDECTECT_H

#include <opencv2/opencv.hpp>

class TargetDectect
{
public:
    static cv::Vec3d checkBestCircle(cv::Mat img);
    static cv::Vec3d checkBestCircleM(cv::Mat img);
    static cv::Vec3d findPreciseCenter(const cv::Mat& grayImage_8U, const cv::Vec3d& coarseCircle);
    static cv::Vec3d findPreciseCenterHighMagnification(const cv::Mat& grayImage_8U,
                                                        const cv::Vec3d& coarseCircle);
};

#endif // TARGETDECTECT_H
