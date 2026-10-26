#ifndef MHEC_H
#define MHEC_H


#include "qmath.h"
#include <mhec/legacy.hpp>
#include <opencv2/opencv.hpp>

namespace MHEC {

    cv::Mat computeRotOLS(const CalibrationData& log, double dt);
    cv::Mat computeTransOLS(const CalibrationData& log, const cv::Mat& R_cs, double dt);

    void updateRotRLS(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est);
    void updateTransRLS(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est);
    void rlsUpdateLinearSystem(cv::Mat& x, cv::Mat& P, const cv::Mat& A, const cv::Mat& b);

    void updateYKronT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est);

    void updateIEKF(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est);

    void updateEKF(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est);

    void updateServoOnce(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est, double scale_factor);

    int stackedJacobianRank(const CalibrationData& log, int observationCount,
                            double scaleFactor);

    void updateServoRot(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_cs_est);
    void updateServoT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_cs_est);

    void updateIEKFRot(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est);
    void updateIEKFT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est);

    void calculateCalibration(std::vector<cv::Vec3d> DetectPoints, std::vector<cv::Vec3d> handPoints, double& scale, cv::Mat& T_sc);

    void planCamMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, double vel_v, double vel_w, double dt);
    void resetCamMotionSequence(int startIndex = 0);

    void planCamTransMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, double vel_v, double dt);

    void planCamRotMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, cv::Vec3d P_c, double vel_w, double dt);

    void planSMotion(cv::Mat& T_se_curr, cv::Mat& twist_s, double vel_w, double vel_v, double dt);

    void planSTransMotion(cv::Mat& T_se_curr, cv::Mat& twist_s, double vel_v, double dt);

    void saveCalbrationData(const CalibrationData& log, const std::string& filename);
}


#endif // MHEC_H
