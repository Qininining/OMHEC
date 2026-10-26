#pragma once
#include <omhec/legacy.hpp>
#include <omhec/math.hpp>

namespace omhec {
using Config = MHEC::CalibrationCfg;
struct State {
    // Tsc maps camera coordinates to stage/base coordinates.
    cv::Mat Tsc;
    Config config;
    MHEC::CalibrationState filter;
    MHEC::CalibrationData observations;
    bool initialized = false;
};
// P/Q: 6x6 left-error covariance of Tcs=inverse(Tsc); R: 3x3 noise covariance.
// With config.iekf_propagate_point_covariance=true, R describes one point;
// otherwise it describes the residual. Defaults retain the hardware convention.
State initialize(const cv::Mat& Tsc, const cv::Mat& P, const cv::Mat& Q,
                 const cv::Mat& R, const Config& config = Config{});
// Each twist is the current CAMERA-frame motion increment [omega(rad); v].
// From a stage twist: xi_camera = Adjoint(inverse(Tsc)) * xi_stage.
void update_iekf(State&, const cv::Mat& cameraTwist,
                 const cv::Vec3d& previous, const cv::Vec3d& current);
void update_fgd(State&, const cv::Mat& cameraTwist,
                const cv::Vec3d& previous, const cv::Vec3d& current,
                double characteristicLength = 1.0);
cv::Mat solve_position_only_ls(const std::vector<cv::Mat>& stagePoses,
                              const std::vector<cv::Vec3d>& cameraPoints);
// H is the instantaneous, first-order Jacobian for a left correction of Tcs.
void residual_model(const cv::Vec3d&, const cv::Vec3d&, const cv::Mat& cameraTwist,
                    cv::Mat& residual, cv::Mat& H);
}
