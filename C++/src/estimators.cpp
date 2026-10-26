// Extracted from the hardware implementation; numerical defaults are retained.
#include <omhec/calibration.hpp>
#include <omhec/math.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
void MHEC::updateYKronT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    
    if (T_sc_est.type() != CV_64F) {
        T_sc_est.convertTo(T_sc_est, CV_64F);
    }

    if (log.all.P_c_curr_list.empty() || log.all.T_se_list.empty()) return;

    double dt = cfg.dt;
    if (dt <= 0) return;

    int n_total = static_cast<int>(log.all.P_c_curr_list.size());
    int window_size = n_total; 
    int start_idx = std::max(0, n_total - window_size);
    int n_window = n_total - start_idx;

    cv::Mat A_window = cv::Mat::zeros(3 * n_window, 15, CV_64F);
    cv::Mat b_window = cv::Mat::zeros(3 * n_window, 1, CV_64F);
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);

    for (int i = start_idx, row_idx = 0; i < n_total; ++i, ++row_idx) {
        
        cv::Vec3d P_c_curr_vec = log.all.P_c_curr_list[i];
        cv::Mat P_c_curr_mat = cv::Mat(P_c_curr_vec, true);
        P_c_curr_mat.convertTo(P_c_curr_mat, CV_64F);

        cv::Mat T_se = log.all.T_se_list[i];
        cv::Mat R_se = T_se(cv::Rect(0, 0, 3, 3)); 
        cv::Mat t_se = T_se(cv::Rect(3, 0, 1, 3));

        cv::Mat A_kron = kron(P_c_curr_mat.t(), I3); 
        cv::Mat A_rot = cv::Mat::eye(3, 3, CV_64F);  
        cv::Mat A_trans = -R_se;                     

        cv::Mat A_i;
        std::vector<cv::Mat> blocks = {A_kron, A_rot, A_trans};
        cv::hconcat(blocks, A_i); 

        A_i.copyTo(A_window.rowRange(row_idx * 3, row_idx * 3 + 3));
        t_se.copyTo(b_window.rowRange(row_idx * 3, row_idx * 3 + 3));
    }

    cv::Mat x_new;
    auto numericalRank = [](const cv::Mat& matrix) {
        cv::SVD svd(matrix, cv::SVD::NO_UV);
        if (svd.w.empty() || svd.w.at<double>(0) <= 0.0) return 0;
        int rank = 0;
        const double tolerance = svd.w.at<double>(0) * 1e-10;
        for (int i = 0; i < svd.w.rows; ++i)
            if (svd.w.at<double>(i) > tolerance) ++rank;
        return rank;
    };
    if (numericalRank(A_window) < 15) {
        
        return;
    }
    
    bool success = cv::solve(A_window, b_window, x_new, cv::DECOMP_SVD);

    if (!success || x_new.empty()) {
        
        return;
    }

    cv::Mat R_sc_raw = x_new(cv::Rect(0, 0, 1, 9)).clone().reshape(1, 3).t();
    
    cv::Mat t_sc_raw = x_new(cv::Rect(0, 9, 1, 3)).clone();

    cv::Mat w, u, vt;
    cv::SVD::compute(R_sc_raw, w, u, vt);
    R_sc_raw = u * vt;

    if (cv::determinant(R_sc_raw) < 0) {
        u.col(2) *= -1.0;
        R_sc_raw = u * vt;
    }

    cv::Mat A_translation = cv::Mat::zeros(3 * n_window, 6, CV_64F);
    cv::Mat b_translation = cv::Mat::zeros(3 * n_window, 1, CV_64F);
    for (int i = start_idx, row = 0; i < n_total; ++i, ++row) {
        cv::Mat p = cv::Mat(log.all.P_c_curr_list[i], true);
        p.convertTo(p, CV_64F);
        const cv::Mat& T_se = log.all.T_se_list[i];
        cv::Mat R_se = T_se(cv::Rect(0, 0, 3, 3));
        cv::Mat t_se = T_se(cv::Rect(3, 0, 1, 3));
        cv::Mat A_i;
        cv::Mat minus_R_se = -R_se;
        cv::hconcat(std::vector<cv::Mat>{cv::Mat::eye(3, 3, CV_64F), minus_R_se}, A_i);
        A_i.copyTo(A_translation.rowRange(3 * row, 3 * row + 3));
        cv::Mat rhs = t_se - R_sc_raw * p;
        rhs.copyTo(b_translation.rowRange(3 * row, 3 * row + 3));
    }
    cv::Mat translation_solution;
    if (numericalRank(A_translation) < 6) {
        
        return;
    }
    if (!cv::solve(A_translation, b_translation, translation_solution, cv::DECOMP_SVD)) {
        
        return;
    }
    t_sc_raw = translation_solution.rowRange(0, 3).clone();

    R_sc_raw.copyTo(T_sc_est(cv::Rect(0, 0, 3, 3)));
    t_sc_raw.copyTo(T_sc_est(cv::Rect(3, 0, 1, 3)));

}

void MHEC::updateIEKF(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    if (log.all.P_c_curr_list.empty() || log.all.P_c_prev_list.empty() ||
        log.all.Twist_c_list.empty()) return;

    auto makeObservation = [&](int i, cv::Mat& e, cv::Mat& H,
                               cv::Mat& observationCovariance) {
        const cv::Vec3d& p0 = log.all.P_c_prev_list[i];
        const cv::Vec3d& p1 = log.all.P_c_curr_list[i];
        const cv::Mat& xi = log.all.Twist_c_list[i];
        cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(xi))));
        omhec::residual_model(p0, p1, xi, e, H);

        if (cfg.iekf_propagate_point_covariance) {

            const cv::Mat deltaRotation = dT(cv::Rect(0, 0, 3, 3));
            observationCovariance = sta.all.KF.R +
                deltaRotation * sta.all.KF.R * deltaRotation.t();
            observationCovariance = 0.5 *
                (observationCovariance + observationCovariance.t());
            observationCovariance += cv::Mat::eye(3, 3, CV_64F);
        } else {
            observationCovariance = sta.all.KF.R;
        }
    };

    const int count = static_cast<int>(log.all.P_c_curr_list.size());
    const bool initialization = count == cfg.initialization_motions;
    cv::Mat Ppred, H, e, V;
    if (initialization) {
        Ppred = sta.all.KF.P + count * sta.all.KF.Q;
        H = cv::Mat::zeros(3 * count, 6, CV_64F);
        e = cv::Mat::zeros(3 * count, 1, CV_64F);
        V = cv::Mat::zeros(3 * count, 3 * count, CV_64F);
        for (int i = 0; i < count; ++i) {
            cv::Mat ei, Hi, Vi;
            makeObservation(i, ei, Hi, Vi);
            Hi.copyTo(H.rowRange(3 * i, 3 * i + 3));
            ei.copyTo(e.rowRange(3 * i, 3 * i + 3));
            Vi.copyTo(V(cv::Rect(3 * i, 3 * i, 3, 3)));
        }
    } else {
        Ppred = sta.all.KF.P + sta.all.KF.Q;
        makeObservation(count - 1, e, H, V);
    }

    cv::Mat S = H * Ppred * H.t() + V;
    cv::Mat Sinv;
    if (!cv::invert(S, Sinv, cv::DECOMP_SVD)) {
        sta.all.KF.P = Ppred;
        
        return;
    }

    if (!initialization) {
        cv::Mat nisMatrix = e.t() * Sinv * e;
        const double nis = nisMatrix.at<double>(0, 0);

        if (cfg.innovation_gate_enabled &&
            (!std::isfinite(nis) || nis > cfg.innovation_gate_threshold)) {
            ++sta.all.consecutiveGateRejections;
            sta.all.KF.P = Ppred;
            bool covarianceInflated = false;
            if (sta.all.consecutiveGateRejections >= cfg.gate_inflation_after_rejections) {
                sta.all.KF.P *= cfg.gate_rejection_inflation;
                
                sta.all.consecutiveGateRejections = 0;
                sta.all.consecutiveGateRejections = 0;
                covarianceInflated = true;
            }
            
            return;
        }
        if (cfg.innovation_gate_enabled)
            sta.all.consecutiveGateRejections = 0;
    }

    cv::Mat K = Ppred * H.t() * Sinv;
    cv::Mat delta = K * e;
    cv::Mat T_cs_est = T_sc_est.inv();
    T_cs_est = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta)))) * T_cs_est;
    T_sc_est = T_cs_est.inv();
    cv::Mat I6 = cv::Mat::eye(6, 6, CV_64F);
    cv::Mat ImKH = I6 - K * H;
    cv::Mat P = ImKH * Ppred * ImKH.t() + K * V * K.t();
    sta.all.KF.P = 0.5 * (P + P.t());
    
}

void MHEC::updateServoOnce(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est, double scale_factor)
{
    
    if (log.all.P_c_curr_list.empty() || log.all.P_c_prev_list.empty() || log.all.Twist_c_list.empty()) {
        
        return;
    }

    double dt = cfg.dt;
    if (dt <= 0) dt = 0.01; 

    double lambda = cfg.decay_gain;

    if (static_cast<int>(log.all.P_c_curr_list.size()) == cfg.initialization_motions) {
        const int count = cfg.initialization_motions;
        cv::Mat J_stack = cv::Mat::zeros(3 * count, 6, CV_64F);
        cv::Mat e_stack = cv::Mat::zeros(3 * count, 1, CV_64F);
        for (int i = 0; i < count; ++i) {
            cv::Vec3d p0 = log.all.P_c_prev_list[i] / scale_factor;
            cv::Vec3d p1 = log.all.P_c_curr_list[i] / scale_factor;
            const cv::Mat& xi = log.all.Twist_c_list[i];
            cv::Vec3d w_i(xi.at<double>(0), xi.at<double>(1), xi.at<double>(2));
            cv::Vec3d v_i(xi.at<double>(3) / scale_factor,
                          xi.at<double>(4) / scale_factor,
                          xi.at<double>(5) / scale_factor);
            cv::Mat xi_scaled;
            cv::vconcat(w_i, v_i, xi_scaled);
            cv::Mat dT_i = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(xi_scaled))));
            cv::Mat p0_h = (cv::Mat_<double>(4, 1) << p0[0], p0[1], p0[2], 1.0);
            cv::Mat p1_hat = dT_i * p0_h;
            cv::Mat e_i = (cv::Mat_<double>(3, 1) <<
                p1[0] - p1_hat.at<double>(0),
                p1[1] - p1_hat.at<double>(1),
                p1[2] - p1_hat.at<double>(2));
            cv::Mat px = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(p0))));
            cv::Mat wx = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(w_i))));
            cv::Mat vx = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(v_i))));
            cv::Mat J_i = cv::Mat::zeros(3, 6, CV_64F);
            cv::Mat J_left_i = -(px * wx - vx);
            J_left_i.copyTo(J_i(cv::Rect(0, 0, 3, 3)));
            wx.copyTo(J_i(cv::Rect(3, 0, 3, 3)));
            J_i.copyTo(J_stack.rowRange(3 * i, 3 * i + 3));
            e_i.copyTo(e_stack.rowRange(3 * i, 3 * i + 3));
        }
        cv::Mat J_pinv;
        cv::invert(J_stack, J_pinv, cv::DECOMP_SVD);
        cv::Mat delta = -J_pinv * e_stack;
        delta.rowRange(3, 6) *= scale_factor;
        cv::Mat T_cs = T_sc_est.inv();
        T_cs = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta)))) * T_cs;
        T_sc_est = T_cs.inv();
        
        return;
    }

    cv::Vec3d P_c_curr = log.all.P_c_curr_list.back() / scale_factor;
    cv::Vec3d P_c_prev = log.all.P_c_prev_list.back() / scale_factor;
    cv::Mat twist_c = log.all.Twist_c_list.back(); 

    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));
    v = v / scale_factor;

    cv::Mat twist_c_scale;
    cv::vconcat(w, v, twist_c_scale);

    cv::Mat T_cs_est = T_sc_est.inv();

    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c_scale))));

    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;
    cv::Vec3d P_c_pred(P_c_pred_homo.at<double>(0), P_c_pred_homo.at<double>(1), P_c_pred_homo.at<double>(2));

    cv::Vec3d dP_pred = P_c_pred - P_c_prev;
    cv::Vec3d dP_meas = P_c_curr - P_c_prev;
    cv::Mat e = (cv::Mat_<double>(3, 1) << (dP_meas[0] - dP_pred[0]) / dt,
                 (dP_meas[1] - dP_pred[1]) / dt,
                 (dP_meas[2] - dP_pred[2]) / dt);

    double e_norm = cv::norm(e);

    cv::Mat skew_p = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(P_c_prev))));
    cv::Mat skew_w = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(w))));
    cv::Mat skew_v = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(v))));

    cv::Mat L = cv::Mat::zeros(3, 6, CV_64F);
    cv::Mat L_left = -(skew_p * skew_w - skew_v);
    cv::Mat L_right = skew_w; 

    L_left.copyTo(L(cv::Rect(0, 0, 3, 3)));
    L_right.copyTo(L(cv::Rect(3, 0, 3, 3)));

    cv::SVD svd(L, cv::SVD::FULL_UV);
    const double sigmaMax = svd.w.empty() ? 0.0 : svd.w.at<double>(0);
    const double sigmaThreshold = cfg.jfgd_svd_relative_threshold * sigmaMax;
    cv::Mat sigmaInv = cv::Mat::zeros(6, 3, CV_64F);
    int retainedRank = 0;
    double sigmaMinRetained = std::numeric_limits<double>::infinity();
    for (int i = 0; i < svd.w.rows; ++i) {
        const double sigma = svd.w.at<double>(i);
        if (sigma > sigmaThreshold) {
            sigmaInv.at<double>(i, i) = 1.0 / sigma;
            sigmaMinRetained = std::min(sigmaMinRetained, sigma);
            ++retainedRank;
        }
    }
    if (retainedRank == 0) {
        
        return;
    }
    cv::Mat L_pinv = svd.vt.t() * sigmaInv * svd.u.t();
    cv::Mat delta_xi = -lambda * L_pinv * e;

    delta_xi.rowRange(3, 6) = delta_xi.rowRange(3, 6) * scale_factor;
    cv::Mat delta_xi_scaled = delta_xi * dt;

    cv::Mat dT_corr = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi_scaled))));

    cv::Mat T_cs_new = dT_corr * T_cs_est;

    T_sc_est = T_cs_new.inv();

}
