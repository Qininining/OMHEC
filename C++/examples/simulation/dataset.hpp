#pragma once
#include <mhec/math.hpp>
#include <vector>
#include <random>

// Synthetic data belongs to the example, never to the estimator library.
struct Dataset {
    cv::Mat truth, initial;
    std::vector<cv::Mat> poses, stageTwists;
    std::vector<cv::Vec3d> points;
};
inline Dataset generateDataset() {
    Dataset d;
    Eigen::VectorXd x(6); x << .2,-.1,.15,1.,-.5,.8;
    d.truth=cvMatFromEigen(MatrixExp6(VecTose3(x))); // Tsc
    x << .03,-.02,.01,.1,-.1,.05;
    d.initial=cvMatFromEigen(MatrixExp6(VecTose3(x)))*d.truth;
    cv::Mat pose=cv::Mat::eye(4,4,CV_64F),Tcs=d.truth.inv();
    std::mt19937 generator(20260818);
    // Explicit Box-Muller avoids implementation-specific normal_distribution.
    auto noise=[&]() {
        double u=(generator()+1.)/4294967297.,v=(generator()+1.)/4294967297.;
        return .0001*std::sqrt(-2*std::log(u))*std::cos(6.283185307179586*v);
    };
    for(int k=0;k<=36;++k) {
        d.poses.push_back(pose.clone());
        cv::Mat tool=(cv::Mat_<double>(4,1)<<.5,.2,2.,1.);
        cv::Mat p=Tcs*pose*tool;
        d.points.emplace_back(p.at<double>(0)+noise(),p.at<double>(1)+noise(),p.at<double>(2)+10*noise());
        if(k==36) break;
        x << .02*std::sin(k+1.),.03*std::cos(k*1.3),.025*std::sin(k*.7+.4),
             .1*std::cos(k*.9),.1*std::sin(k*.8),.1*std::cos(k*.6);
        d.stageTwists.push_back(cvMatFromEigen(x));
        pose=cvMatFromEigen(MatrixExp6(VecTose3(x)))*pose;
    }
    return d;
}
