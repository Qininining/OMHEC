#ifndef MHEC_H
#define MHEC_H


#include "qmath.h"
#include <opencv2/opencv.hpp>

namespace MHEC {

    struct CalibrationCfg {
        double dt = 1;
        double noiseStd = 0.0001;

        int max_rot_iter = 20;
        double rot_err_tolerance = 1e-5;
        int max_p_iter = 20;
        double trans_err_tolerance = 1e-5;
        int max_iter = max_rot_iter + max_p_iter;
        int initialization_motions = 6;
        int validation_motions = 20;
        int repetitions = 12;
        int excitation_sequence_offset = 0;

        double step_len_R = 1.0 / 180.0 * M_PI;  // 5X
        double step_len_t = 100000;    // 5X
        double decay_gain = 0.03;
        double state_length_scale = step_len_t / step_len_R;
        // Relative SVD cutoff used by the single-observation J-FGD update.
        double jfgd_svd_relative_threshold = 0.05;
        // 99% chi-square threshold for a three-dimensional innovation.
        double innovation_gate_threshold = 11.345;
        bool innovation_gate_enabled = true;
        // False uses the independent three-axis measurement-noise model.
        bool physical_use_full_measurement_covariance = false;
        // True means KF.R is the covariance Sigma of one point observation.
        // The relative-residual covariance is then propagated per motion.
        bool iekf_propagate_point_covariance = false;
        int gate_inflation_after_rejections = 3;
        double gate_rejection_inflation = 4.0;
        // Per-axis prior standard deviations for the physical experiment.
        double physical_initial_rotation_std = 5.0 / 180.0 * M_PI;
        double physical_initial_translation_std = 1.0e6; // nm

        // double step_len_R = 1.0 / 180.0 * M_PI;  // 10X
        // double step_len_t = 80000;    // 10X

        bool hot_start = true;
        CalibrationCfg() {

        }
    };

    struct CalibrationData {
        struct Data {
            std::vector<cv::Mat> Twist_s_list;
            std::vector<cv::Mat> Twist_c_list;
            std::vector<cv::Vec3d> P_c_prev_list;
            std::vector<cv::Vec3d> P_c_curr_list;
            std::vector<cv::Vec3d> P_s_prev_list;
            std::vector<cv::Vec3d> P_s_curr_list;
            std::vector<cv::Mat> T_se_list;
            std::vector<cv::Mat> T_cs_list;
            cv::Mat T_cs;
            Data() {
                T_cs = cv::Mat::eye(4, 4, CV_32F);
            }
        };
        Data rot, t, all;
        CalibrationData() : rot(), t(), all() {
        }
    };

    struct CalibrationState {
        struct Data {
            struct KFState {
                cv::Mat Q;
                cv::Mat P;
                cv::Mat R;
                KFState() {
                    Q = cv::Mat();
                    P = cv::Mat();
                    R = cv::Mat();
                }
            };
            double Lambda;
            int consecutiveGateRejections;
            KFState KF;
            Data() : KF() {
                Lambda = 1;
                consecutiveGateRejections = 0;
            }
        };
        Data rot, t, all;
        CalibrationState() : rot(), t(), all() {
        }
    };

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
