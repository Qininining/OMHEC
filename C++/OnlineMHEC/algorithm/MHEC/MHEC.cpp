#include "MHEC.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <QDebug>
#include <random>
#include <fstream>
#include <limits>
#include <QFileInfo>

#include "modern_robotic_lib.h"


void MHEC::calculateCalibration(std::vector<cv::Vec3d> DetectPoints, std::vector<cv::Vec3d> handPoints, double& scale, cv::Mat& T_sc)
{
    int n = static_cast<int>(DetectPoints.size());
    if (n < 3 || handPoints.size() != DetectPoints.size()) {
        qWarning() << "Not enough data points or size mismatch for calibration. Need at least 3 pairs.";
        return;
    }
    cv::Mat Pc_raw(n, 3, CV_64F);
    cv::Mat Ps(n, 3, CV_64F);
    for (int i = 0; i < n; ++i) {
        Pc_raw.at<double>(i, 0) = static_cast<double>(DetectPoints[i][0]);
        Pc_raw.at<double>(i, 1) = static_cast<double>(DetectPoints[i][1]);
        Pc_raw.at<double>(i, 2) = static_cast<double>(DetectPoints[i][2]);
        Ps.at<double>(i, 0) = static_cast<double>(handPoints[i][0]);
        Ps.at<double>(i, 1) = static_cast<double>(handPoints[i][1]);
        Ps.at<double>(i, 2) = static_cast<double>(handPoints[i][2]);
    }

    cv::Mat A(n, 3, CV_64F);
    for (int i = 0; i < n; ++i) {
        A.at<double>(i, 0) = Ps.at<double>(i, 0);
        A.at<double>(i, 1) = Ps.at<double>(i, 1);
        A.at<double>(i, 2) = 1.0;
    }

    // 线性回归求解 Pc_X 和 Pc_Y 方向的映射系数
    cv::Mat cu, cv_coeffs;
    cv::solve(A, Pc_raw.col(0), cu, cv::DECOMP_SVD);
    cv::solve(A, Pc_raw.col(1), cv_coeffs, cv::DECOMP_SVD);

    double s_u = 1.0 / std::sqrt(std::pow(cu.at<double>(0), 2) + std::pow(cu.at<double>(1), 2));
    double s_v = 1.0 / std::sqrt(std::pow(cv_coeffs.at<double>(0), 2) + std::pow(cv_coeffs.at<double>(1), 2));

    // 强制缩放一致，取平均值
    scale = (s_u + s_v) / 2.0;

    // 4. 单位归一化 (Pc 转为物理长度单位 nm)
    cv::Mat Pc_phys = Pc_raw.clone();
    Pc_phys.col(0) *= scale;
    Pc_phys.col(1) *= scale;
    // Pc_phys.col(2) 在您的算法中保持原样

    // 5. Kabsch 算法准备：计算质心并去质心
    // Ps/Pc_phys是单通道N×3矩阵。cv::mean()会把所有元素混成一个
    // 标量，不能得到三个坐标分量各自的质心，因此必须按行分别归约每一列。
    cv::Mat meanPs;
    cv::Mat meanPc;
    cv::reduce(Ps, meanPs, 0, cv::REDUCE_AVG, CV_64F);
    cv::reduce(Pc_phys, meanPc, 0, cv::REDUCE_AVG, CV_64F);

    cv::Mat Psc(n, 3, CV_64F);
    cv::Mat Pcc(n, 3, CV_64F);
    for (int i = 0; i < n; ++i) {
        Psc.row(i) = Ps.row(i) - meanPs;
        Pcc.row(i) = Pc_phys.row(i) - meanPc;
    }

    // 6. SVD 求解旋转矩阵 R
    // 计算协方差矩阵 H = Psc' * Pcc
    cv::Mat H = Psc.t() * Pcc;
    cv::SVD svd(H);
    cv::Mat R = svd.vt.t() * svd.u.t();

    // 保证为右手坐标系（防止镜像）
    if (cv::determinant(R) < 0) {
        cv::Mat V = svd.vt.t();
        V.col(2) *= -1.0;
        R = V * svd.u.t();
    }
    cv::Mat R_sc = R.clone();

    // 7. 求解平移向量 t (注意：Matlab 中 T = meanPc' - R * meanPs')
    cv::Mat t_sc = meanPc.t() - R_sc * meanPs.t();

    R_sc.copyTo(T_sc(cv::Rect(0, 0, 3, 3)));
    t_sc.copyTo(T_sc(cv::Rect(3, 0, 1, 3)));

    // 8. 信息打印与验证
    qDebug() << "==========================================";
    qDebug() << "Kabsch Calibration Result (Uniform Scale)";
    qDebug() << "Scale (nm/pixel):" << scale;
    qDebug() << "R_sc Matrix:\n"
             << R_sc.at<double>(0,0) << R_sc.at<double>(0,1) << R_sc.at<double>(0,2) << "\n"
             << R_sc.at<double>(1,0) << R_sc.at<double>(1,1) << R_sc.at<double>(1,2) << "\n"
             << R_sc.at<double>(2,0) << R_sc.at<double>(2,1) << R_sc.at<double>(2,2);
    qDebug() << "t_sc Vector (nm):" << t_sc.at<double>(0) << t_sc.at<double>(1) << t_sc.at<double>(2);

    // 计算简单的平均误差验证
    double total_err = 0;
    for (int i = 0; i < n; ++i) {
        cv::Mat ps_vec = Ps.row(i).t();
        cv::Mat pc_phys_pred = R_sc * ps_vec + t_sc;

        double dx = Pc_phys.at<double>(i, 0) - pc_phys_pred.at<double>(0);
        double dy = Pc_phys.at<double>(i, 1) - pc_phys_pred.at<double>(1);
        double dz = Pc_phys.at<double>(i, 2) - pc_phys_pred.at<double>(2);
        total_err += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    qDebug() << "Mean Reprojection Error (nm):" << (total_err / n);
}



cv::Mat MHEC::computeRotOLS(const CalibrationData& log, double dt) {
    int N = log.rot.Twist_s_list.size();
    cv::Mat X(3, N, CV_64F);
    cv::Mat Y(3, N, CV_64F);

    for (int k = 0; k < N; ++k) {
        cv::Mat twist_v = log.rot.Twist_s_list[k].rowRange(3, 6); // 取 v 部分
        cv::Mat dx = twist_v * dt;
        dx.copyTo(X.col(k));

        cv::Vec3d dp = log.rot.P_c_curr_list[k] - log.rot.P_c_prev_list[k];
        Y.at<double>(0, k) = dp[0];
        Y.at<double>(1, k) = dp[1];
        Y.at<double>(2, k) = dp[2];
    }

    cv::Mat XXt = X * X.t();
    if (std::abs(cv::determinant(XXt)) < 1e-9) {
        qWarning() << "Degenerate motion detected! Rotation estimation might be invalid.";
        return cv::Mat::eye(3, 3, CV_64F);
    }
    cv::Mat R_raw = Y * X.t() * XXt.inv();
    // cv::Mat R_raw = Y * X.t() * (X * X.t()).inv();

    cv::SVD svd(R_raw, cv::SVD::FULL_UV);
    cv::Mat R_cs = svd.u * svd.vt;
    if (cv::determinant(R_cs) < 0) {
        cv::Mat D = cv::Mat::eye(3, 3, CV_64F);
        D.at<double>(2, 2) = -1.0;
        R_cs = svd.u * D * svd.vt;
    }
    return R_cs;
}


cv::Mat MHEC::computeTransOLS(const CalibrationData& log, const cv::Mat& R_cs, double dt) {
    int N = log.t.Twist_s_list.size();
    cv::Mat A_total = cv::Mat::zeros(3 * N, 3, CV_64F);
    cv::Mat b_total = cv::Mat::zeros(3 * N, 1, CV_64F);

    for (int k = 0; k < N; ++k) {
        // 计算 T_step = Exp6(Twist_s * dt)
        cv::Mat T_step = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(log.t.Twist_s_list[k] * dt))));
        cv::Mat R_step = T_step(cv::Rect(0, 0, 3, 3));
        cv::Mat t_step = T_step(cv::Rect(3, 0, 1, 3));

        cv::Mat R_prime = R_cs * R_step * R_cs.t();
        cv::Mat Ak = cv::Mat::eye(3, 3, CV_64F) - R_prime;

        cv::Vec3d Pc_curr = log.t.P_c_curr_list[k];
        cv::Vec3d Pc_prev = log.t.P_c_prev_list[k];
        cv::Mat Pc_curr_m = (cv::Mat_<double>(3, 1) << Pc_curr[0], Pc_curr[1], Pc_curr[2]);
        cv::Mat Pc_prev_m = (cv::Mat_<double>(3, 1) << Pc_prev[0], Pc_prev[1], Pc_prev[2]);

        cv::Mat bk = Pc_curr_m - R_prime * Pc_prev_m - R_cs * t_step;

        Ak.copyTo(A_total.rowRange(k * 3, k * 3 + 3));
        bk.copyTo(b_total.rowRange(k * 3, k * 3 + 3));
    }

    cv::Mat t_cs;
    cv::solve(A_total, b_total, t_cs, cv::DECOMP_SVD);
    return t_cs;
}

void MHEC::updateRotRLS(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    // 1. 确保输入矩阵是 CV_64F，如果是 CV_32F 则会导致 copyTo 失效
    if (T_sc_est.type() != CV_64F) {
        T_sc_est.convertTo(T_sc_est, CV_64F);
    }

    if (log.rot.P_c_curr_list.empty() || log.rot.Twist_s_list.empty()) return;

    double dt = cfg.dt;
    if (dt <= 0) return;

    // 2. 提取旋转部分（此时 R_sc_est 是 T_sc_est 的一个引用/Header）
    cv::Mat R_sc_est = T_sc_est(cv::Rect(0, 0, 3, 3)).clone(); // 先 clone 出来计算，避免计算中途脱钩
    cv::Mat& P = sta.rot.KF.P;

    // 计算观测和预测位移
    cv::Vec3d dP_c_vec = log.rot.P_c_curr_list.back() - log.rot.P_c_prev_list.back();
    cv::Mat twist_s = log.rot.Twist_s_list.back();

    // 注意：twist_s 布局通常是 [w_x, w_y, w_z, v_x, v_y, v_z]
    // 你的代码是用 cv::Rect(0, 3, 1, 3) 提取，请确认这确实是线速度 v
    cv::Mat v_s = twist_s(cv::Rect(0, 3, 1, 3));
    cv::Mat dP_s = v_s * dt;

    // 3. RLS 更新
    // 确保 A 和 b 的维度与 RLS 内部逻辑匹配 (1x3)
    cv::Mat A = dP_s.t();
    cv::Mat b = cv::Mat(dP_c_vec).t();

    MHEC::rlsUpdateLinearSystem(R_sc_est, P, A, b);

    // 4. 正交化约束 (SVD)
    cv::Mat w, u, vt;
    cv::SVD::compute(R_sc_est, w, u, vt);
    R_sc_est = u * vt;

    // 5. 关键：确保写回 T_sc_est
    // 使用 convertTo 配合 ROI，即使类型不同也能强制写回
    cv::Mat targetROI = T_sc_est(cv::Rect(0, 0, 3, 3));
    R_sc_est.convertTo(targetROI, targetROI.type());

    // 调试打印：检查 R 矩阵是否有微小变化
    // qInfo() << "R_sc Trace:" << cv::trace(R_sc_est)[0];
}

void MHEC::updateTransRLS(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    // 1. 数据校验与类型对齐
    if (log.t.P_c_curr_list.empty() || log.t.Twist_s_list.empty()) return;
    if (T_sc_est.type() != CV_64F) T_sc_est.convertTo(T_sc_est, CV_64F);

    double dt = cfg.dt;
    cv::Mat& P = sta.t.KF.P;

    // 2. 变换到 Camera 系 (T_cs) 进行平移向量更新
    cv::Mat T_cs_est = T_sc_est.inv();
    cv::Mat R_cs_est = T_cs_est(cv::Rect(0, 0, 3, 3)).clone(); // 保持旋转不变
    cv::Mat t_cs_est = T_cs_est(cv::Rect(3, 0, 1, 3)).clone(); // 这是我们要更新的量

    // 3. 计算运动增量与 A, b 矩阵
    cv::Mat twist_s = log.t.Twist_s_list.back();
    if (twist_s.type() != CV_64F) twist_s.convertTo(twist_s, CV_64F);
    cv::Mat T_s_step = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s * dt))));
    cv::Mat R_s_step = T_s_step(cv::Rect(0, 0, 3, 3));
    cv::Mat t_s_step = T_s_step(cv::Rect(3, 0, 1, 3));

    // A = I - R_cs * R_s_step * R_cs.t()
    cv::Mat R_prime = R_cs_est * R_s_step * R_cs_est.t();
    cv::Mat A = cv::Mat::eye(3, 3, CV_64F) - R_prime;

    // b = P_c_curr - R_prime * P_c_prev - R_cs * t_s_step
    // cv::Mat P_c_curr = cv::Mat(log.t.P_c_curr_list.back()); // 3x1
    // cv::Mat P_c_prev = cv::Mat(log.t.P_c_prev_list.back()); // 3x1
    cv::Mat P_c_curr, P_c_prev;
    cv::Mat(log.t.P_c_curr_list.back()).convertTo(P_c_curr, CV_64F);
    cv::Mat(log.t.P_c_prev_list.back()).convertTo(P_c_prev, CV_64F);
    cv::Mat b = P_c_curr - R_prime * P_c_prev - R_cs_est * t_s_step;

    // 4. RLS 更新 (更新 t_cs_est)
    MHEC::rlsUpdateLinearSystem(t_cs_est, P, A, b);

    // 5. 重新组合并回写
    cv::Mat T_cs_new = cv::Mat::eye(4, 4, CV_64F);
    R_cs_est.copyTo(T_cs_new(cv::Rect(0, 0, 3, 3)));
    t_cs_est.copyTo(T_cs_new(cv::Rect(3, 0, 1, 3)));

    // 将更新后的结果重新赋值给 T_sc_est
    T_sc_est = T_cs_new.inv();

    // 调试打印：检查位移向量前后的模长变化
    qInfo() << "t_cs norm update:" << cv::norm(t_cs_est);
}

void MHEC::rlsUpdateLinearSystem(cv::Mat& x, cv::Mat& P, const cv::Mat& A, const cv::Mat& b)
{
    int dim_x = x.rows;
    int dim_b = b.rows;

    cv::Mat I_meas = cv::Mat::eye(dim_b, dim_b, CV_64F);
    cv::Mat S = A * P * A.t() + I_meas;
    cv::Mat K = P * A.t() * S.inv();
    cv::Mat e = b - A * x;

    x = x + K * e;
    cv::Mat I_state = cv::Mat::eye(dim_x, dim_x, CV_64F);
    P = (I_state - K * A) * P;
}


// void MHEC::updateIEKF(CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
// {
//     // 1. 提取最新数据
//     if (log.all.P_c_curr_list.empty()) return;

//     cv::Mat P = sta.all.KF.P;
//     cv::Mat Q = sta.all.KF.Q;
//     cv::Mat R = sta.all.KF.R;
//     cv::Vec3d P_c_curr = log.all.P_c_curr_list.back();
//     cv::Vec3d P_c_prev = log.all.P_c_prev_list.back();
//     cv::Mat twist_c = log.all.Twist_c_list.back(); // 假设是 6x1

//     cv::Mat T_cs_est = T_sc_est.inv();

//     // 2. 状态预测与残差计算
//     cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
//     cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));

//     cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));
//     cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
//     cv::Mat P_c_pred_homo = dT * P_c_prev_homo;

//     cv::Vec3d dP_meas = P_c_curr - P_c_prev;
//     cv::Vec3d dP_pred(P_c_pred_homo.at<double>(0) - P_c_prev[0],
//                       P_c_pred_homo.at<double>(1) - P_c_prev[1],
//                       P_c_pred_homo.at<double>(2) - P_c_prev[2]);

//     cv::Mat e = cv::Mat(dP_meas - dP_pred); // 3x1

//     // H = [skew(P_c_prev)*skew(w) - skew(v), -skew(w)]
//     cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
//     cv::Mat H_left = cvMatFromEigen(VecToso3(eigenFromCvMat(P_c_prev))
//                                         * VecToso3(eigenFromCvMat(w))
//                                     - VecToso3(eigenFromCvMat(v)));
//     cv::Mat H_right = -cvMatFromEigen(VecToso3(eigenFromCvMat(w)));
//     H_left.copyTo(H(cv::Rect(0, 0, 3, 3)));
//     H_right.copyTo(H(cv::Rect(3, 0, 3, 3)));

//     P = P + Q;
//     cv::Mat S = H * P * H.t() + R;
//     cv::Mat K = P * H.t() * S.inv();

//     // 5. NIS 检查 (卡方分布检验)
//     cv::Mat nis_mat = e.t() * S.inv() * e;
//     double nis = nis_mat.at<double>(0, 0);
//     if (nis > 16.27) {
//         return; // 观测异常，丢弃
//     }

//     // 6. 更新状态 (流形上的更新)
//     cv::Mat delta_xi = K * e; // 6x1
//     cv::Mat T_cs_new = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi)))) * T_cs_est;

//     // 写回结果
//     T_sc_est = T_cs_new.inv();

//     // 7. 协方差更新 (Joseph Form 保证正定性)
//     cv::Mat I6 = cv::Mat::eye(6, 6, CV_64F);
//     P = (I6 - K * H) * P * (I6 - K * H).t() + K * R * K.t();

//     sta.all.KF.P = P;
// }


// void MHEC::updateYKronT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
// {
//     // 1. 确保输入矩阵是 CV_64F，如果是 CV_32F 则会导致 copyTo 失效
//     if (T_sc_est.type() != CV_64F) {
//         T_sc_est.convertTo(T_sc_est, CV_64F);
//     }

//     if (log.all.P_c_curr_list.empty() || log.all.T_se_list.empty()) return;

//     double dt = cfg.dt;
//     if (dt <= 0) return;

//     cv::Vec3d P_c_curr_vec = log.all.P_c_curr_list.back();
//     cv::Mat P_c_curr_mat = cv::Mat(P_c_curr_vec, true); // true 表示深拷贝
//     P_c_curr_mat.convertTo(P_c_curr_mat, CV_64F);

//     cv::Mat T_se = log.all.T_se_list.back();
//     cv::Mat R_se = T_se(cv::Rect(0, 0, 3, 3)).clone();
//     cv::Mat t_se = T_se(cv::Rect(3, 0, 1, 3)).clone();

//     cv::Mat& P = sta.all.KF.P;

//     // 1. 获取齐次变换矩阵
//     cv::Mat R_sc_est = T_sc_est(cv::Rect(0, 0, 3, 3)).clone();
//     cv::Mat t_sc_est = T_sc_est(cv::Rect(3, 0, 1, 3)).clone();

//     cv::Mat R_sc_vec = R_sc_est.reshape(1, 9);
//     cv::Mat zeros3 = cv::Mat::zeros(3, 1, CV_64F);

//     std::vector<cv::Mat> x_blocks = {R_sc_vec, t_sc_est, zeros3};
//     cv::Mat x_new;
//     cv::vconcat(x_blocks, x_new);

//     cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);
//     cv::Mat A_kron = kron(P_c_curr_mat.t(), I3); // .t() 实现矩阵转置
//     cv::Mat A_rot = cv::Mat::eye(3, 3, CV_64F);
//     cv::Mat A_trans = -R_se;
//     cv::Mat A;
//     std::vector<cv::Mat> blocks = {A_kron, A_rot, A_trans};
//     cv::hconcat(blocks, A);
//     cv::Mat b = t_se.clone(); // 推荐加上 .clone() 以确保底层数据独立和内存连续

//     MHEC::rlsUpdateLinearSystem(x_new, P, A, b);

//     cv::Mat R_sc_raw = x_new(cv::Rect(0, 0, 1, 9)).clone().reshape(1, 3);
//     // 提取接下来的 3 个元素 (9~11) 作为平移向量
//     cv::Mat t_sc_raw = x_new(cv::Rect(0, 9, 1, 3)).clone();

//     // 4. 正交化约束 (SVD)
//     cv::Mat w, u, vt;
//     cv::SVD::compute(R_sc_raw, w, u, vt);
//     R_sc_raw = u * vt;

//     R_sc_raw.copyTo(T_sc_est(cv::Rect(0, 0, 3, 3)));
//     t_sc_raw.copyTo(T_sc_est(cv::Rect(3, 0, 1, 3)));

//     // 调试打印：检查 R 矩阵是否有微小变化
//     qInfo() << "R_sc Trace:" << cv::trace(R_sc_est)[0];

// }


void MHEC::updateYKronT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    // 1. 确保输入矩阵是 CV_64F，如果是 CV_32F 则会导致 copyTo 失效
    if (T_sc_est.type() != CV_64F) {
        T_sc_est.convertTo(T_sc_est, CV_64F);
    }

    if (log.all.P_c_curr_list.empty() || log.all.T_se_list.empty()) return;

    double dt = cfg.dt;
    if (dt <= 0) return;

    // 2. 确定滑动窗口的起始索引
    // 假设 window_size 存在 sta 或 sta.all 中 (请根据你的实际变量名修改，比如 sta.window_size)
    int n_total = static_cast<int>(log.all.P_c_curr_list.size());
    int window_size = n_total; // cumulative LS: retain every observation in this run
    int start_idx = std::max(0, n_total - window_size);
    int n_window = n_total - start_idx;

    // 3. 预分配大矩阵 (A_window: 3N x 15, b_window: 3N x 1)
    cv::Mat A_window = cv::Mat::zeros(3 * n_window, 15, CV_64F);
    cv::Mat b_window = cv::Mat::zeros(3 * n_window, 1, CV_64F);
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);

    // 4. 遍历窗口内的数据，填充 A_window 和 b_window
    for (int i = start_idx, row_idx = 0; i < n_total; ++i, ++row_idx) {
        // 提取当前点
        cv::Vec3d P_c_curr_vec = log.all.P_c_curr_list[i];
        cv::Mat P_c_curr_mat = cv::Mat(P_c_curr_vec, true);
        P_c_curr_mat.convertTo(P_c_curr_mat, CV_64F);

        // 提取当前的 T_se
        cv::Mat T_se = log.all.T_se_list[i];
        cv::Mat R_se = T_se(cv::Rect(0, 0, 3, 3)); // 这里不需要 clone，直接取 ROI 参与计算即可
        cv::Mat t_se = T_se(cv::Rect(3, 0, 1, 3));

        // 构建当前帧的 A 矩阵分块
        cv::Mat A_kron = kron(P_c_curr_mat.t(), I3); // 3x9
        cv::Mat A_rot = cv::Mat::eye(3, 3, CV_64F);  // 3x3
        cv::Mat A_trans = -R_se;                     // 3x3

        cv::Mat A_i;
        std::vector<cv::Mat> blocks = {A_kron, A_rot, A_trans};
        cv::hconcat(blocks, A_i); // 拼接成 3x15

        // 将当前 A_i 和 t_se 放入大矩阵对应行
        A_i.copyTo(A_window.rowRange(row_idx * 3, row_idx * 3 + 3));
        t_se.copyTo(b_window.rowRange(row_idx * 3, row_idx * 3 + 3));
    }

    // 5. 求解滑动窗口的超定方程组 A_window * x = b_window
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
        qWarning() << "Cumulative PO-LS system is rank deficient.";
        return;
    }
    // 推荐使用 DECOMP_SVD，对可能出现的共面/病态数据有更好的鲁棒性
    bool success = cv::solve(A_window, b_window, x_new, cv::DECOMP_SVD);

    if (!success || x_new.empty()) {
        qWarning() << "Sliding window LS solve failed!";
        return;
    }

    // 6. 解析求解结果
    // 提取前 9 个元素作为旋转矩阵 R_sc_raw
    cv::Mat R_sc_raw = x_new(cv::Rect(0, 0, 1, 9)).clone().reshape(1, 3).t();
    // 提取接下来的 3 个元素 (9~11) 作为平移向量
    cv::Mat t_sc_raw = x_new(cv::Rect(0, 9, 1, 3)).clone();

    // 7. 正交化约束 (SVD)
    cv::Mat w, u, vt;
    cv::SVD::compute(R_sc_raw, w, u, vt);
    R_sc_raw = u * vt;

    // 强制右手系（防止镜像），这是 SVD 恢复旋转矩阵的必要步骤
    if (cv::determinant(R_sc_raw) < 0) {
        u.col(2) *= -1.0;
        R_sc_raw = u * vt;
    }

    // 8. 写入输出变量
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
        qWarning() << "Cumulative PO-LS translation system is rank deficient.";
        return;
    }
    if (!cv::solve(A_translation, b_translation, translation_solution, cv::DECOMP_SVD)) {
        qWarning() << "Cumulative PO-LS translation re-estimation failed.";
        return;
    }
    t_sc_raw = translation_solution.rowRange(0, 3).clone();

    R_sc_raw.copyTo(T_sc_est(cv::Rect(0, 0, 3, 3)));
    t_sc_raw.copyTo(T_sc_est(cv::Rect(3, 0, 1, 3)));

    // 调试打印：检查当前窗口大小与迹
    // qInfo() << "Sliding Window Elements:" << n_window << "R_sc Trace:" << cv::trace(R_sc_raw)[0];
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
        cv::Vec3d w(xi.at<double>(0), xi.at<double>(1), xi.at<double>(2));
        cv::Vec3d v(xi.at<double>(3), xi.at<double>(4), xi.at<double>(5));
        cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(xi))));
        cv::Mat p0h = (cv::Mat_<double>(4, 1) << p0[0], p0[1], p0[2], 1.0);
        cv::Mat pHat = dT * p0h;
        e = (cv::Mat_<double>(3, 1) << p1[0] - pHat.at<double>(0),
             p1[1] - pHat.at<double>(1), p1[2] - pHat.at<double>(2));
        H = cv::Mat::zeros(3, 6, CV_64F);
        cv::Mat px = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(p0))));
        cv::Mat wx = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(w))));
        cv::Mat vx = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(v))));
        cv::Mat Hleft = px * wx - vx;
        cv::Mat Hright = -wx;
        Hleft.copyTo(H(cv::Rect(0, 0, 3, 3)));
        Hright.copyTo(H(cv::Rect(3, 0, 3, 3)));

        if (cfg.iekf_propagate_point_covariance) {
            // e_k = p_k - (DeltaR_k*p_{k-1} + Delta t_k). For anisotropic
            // point noise, V_k is generally neither diagonal nor 2*Sigma.
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
        qWarning() << "J-RIEKF skipped: singular innovation covariance.";
        return;
    }

    if (!initialization) {
        cv::Mat nisMatrix = e.t() * Sinv * e;
        const double nis = nisMatrix.at<double>(0, 0);
        // 无论是否启用门控都记录NIS，便于用无异常值的实物数据核验理论阈值。
        qInfo() << "J-RIEKF NIS =" << nis
                << "threshold =" << cfg.innovation_gate_threshold
                << "gate enabled =" << cfg.innovation_gate_enabled;
        if (cfg.innovation_gate_enabled &&
            (!std::isfinite(nis) || nis > cfg.innovation_gate_threshold)) {
            ++sta.all.consecutiveGateRejections;
            sta.all.KF.P = Ppred;
            bool covarianceInflated = false;
            if (sta.all.consecutiveGateRejections >= cfg.gate_inflation_after_rejections) {
                sta.all.KF.P *= cfg.gate_rejection_inflation;
                // 一次恢复动作对应一组连续拒绝，防止后续每帧都重复放大协方差。
                sta.all.consecutiveGateRejections = 0;
                sta.all.consecutiveGateRejections = 0;
                covarianceInflated = true;
            }
            qWarning() << "J-RIEKF gate rejected observation; NIS =" << nis
                       << "consecutive =" << sta.all.consecutiveGateRejections
                       << "covariance inflated =" << covarianceInflated;
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
    qInfo() << (initialization ? "J-RIEKF stacked initialization" : "J-RIEKF update")
            << "residual norm =" << cv::norm(e) << "correction norm =" << cv::norm(delta);
}

int MHEC::stackedJacobianRank(const CalibrationData& log, int observationCount,
                              double scaleFactor)
{
    const int available = static_cast<int>(std::min(
        log.all.P_c_prev_list.size(), log.all.Twist_c_list.size()));
    if (observationCount <= 0 || available < observationCount ||
        !std::isfinite(scaleFactor) || scaleFactor <= 0.0) {
        return 0;
    }

    // PO-LS比J方法多保存一个零运动初始样本，因此统一取最后N条真实运动观测。
    const int first = available - observationCount;
    cv::Mat stacked = cv::Mat::zeros(3 * observationCount, 6, CV_64F);
    for (int row = 0; row < observationCount; ++row) {
        const int index = first + row;
        const cv::Vec3d point = log.all.P_c_prev_list[index] / scaleFactor;
        const cv::Mat& twist = log.all.Twist_c_list[index];
        if (twist.empty() || twist.total() < 6) return 0;

        const cv::Vec3d omega(twist.at<double>(0), twist.at<double>(1),
                              twist.at<double>(2));
        const cv::Vec3d velocity(twist.at<double>(3) / scaleFactor,
                                 twist.at<double>(4) / scaleFactor,
                                 twist.at<double>(5) / scaleFactor);
        const cv::Mat px = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(point))));
        const cv::Mat wx = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(omega))));
        const cv::Mat vx = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(velocity))));
        cv::Mat jacobian = cv::Mat::zeros(3, 6, CV_64F);
        const cv::Mat leftBlock = px * wx - vx;
        const cv::Mat rightBlock = -wx;
        leftBlock.copyTo(jacobian(cv::Rect(0, 0, 3, 3)));
        rightBlock.copyTo(jacobian(cv::Rect(3, 0, 3, 3)));
        jacobian.copyTo(stacked.rowRange(3 * row, 3 * row + 3));
    }

    cv::SVD svd(stacked, cv::SVD::NO_UV);
    if (svd.w.empty() || svd.w.at<double>(0) <= 0.0) return 0;
    const double tolerance = svd.w.at<double>(0) * 1e-10;
    int rank = 0;
    for (int i = 0; i < svd.w.rows; ++i)
        if (svd.w.at<double>(i) > tolerance) ++rank;
    return rank;
}

#if 0
void updateIEKFLegacy(const MHEC::CalibrationCfg& cfg, MHEC::CalibrationState& sta, const MHEC::CalibrationData& log, cv::Mat& T_sc_est)
{
    // 1. 提取最新数据
    if (log.all.P_c_curr_list.empty() || log.all.P_c_prev_list.empty() || log.all.Twist_c_list.empty()) {
        qWarning() << "IEKF Update failed: Data lists are empty.";
        return;
    }

    cv::Mat P = sta.all.KF.P.clone();
    cv::Mat Q = sta.all.KF.Q;
    cv::Mat R = sta.all.KF.R;

    cv::Vec3d P_c_curr = log.all.P_c_curr_list.back();
    cv::Vec3d P_c_prev = log.all.P_c_prev_list.back();
    cv::Mat twist_c = log.all.Twist_c_list.back(); // 6x1: [w_x, w_y, w_z, v_x, v_y, v_z]

    // 状态量：T_cs (Camera to Sensor)
    cv::Mat T_cs_est = T_sc_est.inv();

    // 2. 状态预测 (Predict)
    // 提取速度矢量
    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));

    // 计算运动增量 dT (Exp map)
    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));

    // 预测观测位移
    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;

    cv::Vec3d dP_meas = P_c_curr - P_c_prev;
    cv::Vec3d dP_pred(P_c_pred_homo.at<double>(0) - P_c_prev[0],
                      P_c_pred_homo.at<double>(1) - P_c_prev[1],
                      P_c_pred_homo.at<double>(2) - P_c_prev[2]);

    // --- 新增调试打印 ---
    // double norm_meas = cv::norm(dP_meas);
    // double norm_pred = cv::norm(dP_pred);

    // qInfo() << "---------------------------------------------";
    // qInfo() << "Norm Measured dP:" << norm_meas;
    // qInfo() << "Norm Predicted dP:" << norm_pred;
    // qInfo() << "Scale Ratio (Meas/Pred):" << (norm_pred != 0 ? norm_meas / norm_pred : 0);


    // 计算残差 e (Innovation)
    cv::Mat e = (cv::Mat_<double>(3, 1) << (dP_meas[0] - dP_pred[0]),
                 (dP_meas[1] - dP_pred[1]),
                 (dP_meas[2] - dP_pred[2]));

    // 3. 构建观测矩阵 H (Jacobian)
    // H = [ [P_c_prev]x*[w]x - [v]x, -[w]x ]
    cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
    cv::Mat H_left = cvMatFromEigen(VecToso3(eigenFromCvMat(P_c_prev)) * VecToso3(eigenFromCvMat(w))
                                    - VecToso3(eigenFromCvMat(v)));
    cv::Mat H_right = -cvMatFromEigen(VecToso3(eigenFromCvMat(w)));

    H_left.copyTo(H(cv::Rect(0, 0, 3, 3)));
    H_right.copyTo(H(cv::Rect(3, 0, 3, 3)));

    // 4. 协方差时间更新
    P = P + Q;

    // auto checkMatrixHealth = [](const cv::Mat& mat, const QString& name) {
    //     bool ok = cv::checkRange(mat, true, nullptr, -1e10, 1e10);
    //     if (!ok) {
    //         qCritical() << "☠️ FATAL: Matrix" << name << "contains NaN or Inf!";
    //         // 可以选择打印出来看看具体是哪个元素：
    //         // std::cout << name.toStdString() << " = \n" << mat << std::endl;
    //     }
    //     return ok;
    // };
    // checkMatrixHealth(H, "H (Jacobian)");
    // checkMatrixHealth(P, "P (Covariance)");
    // checkMatrixHealth(R, "R (Measurement Noise)");

    // 5. 测量更新
    cv::Mat S = H * P * H.t() + R;
    double detS = cv::determinant(S);
    // 新增 std::isnan 和 std::isinf 检查
    if (std::isnan(detS) || std::isinf(detS) || std::abs(detS) < 1e-12) {
        qWarning() << "IEKF Update aborted: S matrix is invalid (Singular/NaN/Inf)! detS:" << detS;
        return;
    }

    cv::Mat K = P * H.t() * S.inv();

    // 6. NIS 检查 (卡方分布检验)
    cv::Mat nis_mat = e.t() * S.inv() * e;
    double nis = nis_mat.at<double>(0, 0);
    // qInfo() << "NIS Value:" << nis << (nis > 16.27 ? " (REJECTED)" : " (ACCEPTED)");
    // if (nis > 16.27) {
    //     // 如果 NIS 过大，说明当前帧噪声异常（可能是刚才算的 Z 轴 6294 的抖动造成的）
    //     return;
    // }

    // qInfo() << "--- IEKF Debug Info ---";
    qInfo() << "Residual e: [" << e.at<double>(0) << "," << e.at<double>(1) << "," << e.at<double>(2) << "]";
    double norm_e = cv::norm(e);
    qInfo() << "Norm norm_e:" << norm_e;

    // 7. 状态更新 (流形上的指数映射)
    cv::Mat delta_xi = K * e; // 6x1
    cv::Mat T_cs_new = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi)))) * T_cs_est;

    // 写回结果并更新 T_sc
    T_sc_est = T_cs_new.inv();

    // 8. 协方差更新 (Joseph Form 保证对称性和正定性)
    cv::Mat I6 = cv::Mat::eye(6, 6, CV_64F);
    cv::Mat ImKH = I6 - K * H;
    P = ImKH * P * ImKH.t() + K * R * K.t();

    // 更新状态量
    sta.all.KF.P = P;

    qInfo() << "Correction delta_xi norm:" << cv::norm(delta_xi);
}

#endif

void MHEC::updateEKF(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    // 1. 数据校验与初始化
    if (log.all.P_c_curr_list.empty() || log.all.P_c_prev_list.empty() || log.all.Twist_s_list.empty()) {
        qWarning() << "EKF Update failed: Data lists are empty.";
        return;
    }

    // 获取协方差矩阵 (如果未初始化则在此赋予初值)
    cv::Mat P = sta.all.KF.P;
    cv::Mat Q = sta.all.KF.Q;
    cv::Mat R = sta.all.KF.R;

    // 2. 提取输入数据
    cv::Vec3d P_c_curr = log.all.P_c_curr_list.back();
    cv::Vec3d P_c_prev = log.all.P_c_prev_list.back();
    cv::Mat twist_s = log.all.Twist_s_list.back(); // 6x1: [w, v] 在传感器坐标系下

    cv::Mat T_cs_est = T_sc_est.inv();

    // 3. 预测阶段 (Covariance Time Update)
    P = P + Q;

    // 4. 几何量计算
    // 计算 T_s_step = Exp(twist_s * dt)
    cv::Mat twist_scaled = twist_s;
    cv::Mat T_s_step = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_scaled))));

    // 预测观测位移 (Predict P_c_curr)
    // P_c_curr_pred = T_cs * T_s_step * T_sc * P_c_prev
    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_s_prev_est = T_sc_est * P_c_prev_homo;
    cv::Mat P_s_curr_pred = T_s_step * P_s_prev_est;
    cv::Mat P_c_curr_pred_homo = T_cs_est * P_s_curr_pred;

    // 计算残差 y (Innovation)
    cv::Vec3d P_c_curr_pred(P_c_curr_pred_homo.at<double>(0),
                            P_c_curr_pred_homo.at<double>(1),
                            P_c_curr_pred_homo.at<double>(2));
    cv::Mat y = (cv::Mat_<double>(3, 1) << (P_c_curr[0] - P_c_curr_pred[0]),
                 (P_c_curr[1] - P_c_curr_pred[1]),
                 (P_c_curr[2] - P_c_curr_pred[2]));

    // 5. 构建 Jacobian 矩阵 H
    // M = T_cs * T_s_step * T_sc
    cv::Mat M = T_cs_est * T_s_step * T_sc_est;

    // --- D_p 构建 ---
    cv::Vec3d p(P_c_prev[0], P_c_prev[1], P_c_prev[2]);
    cv::Mat skew_p = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(p))));
    cv::Mat D_p = cv::Mat::zeros(4, 6, CV_64F);
    // 使用赋值操作符 = 代替 copyTo，OpenCV 会自动处理 MatExpr
    cv::Mat D_p_top_left = D_p(cv::Rect(0, 0, 3, 3));
    D_p_top_left = -skew_p;
    cv::Mat D_p_top_right = D_p(cv::Rect(3, 0, 3, 3));
    D_p_top_right = cv::Mat::eye(3, 3, CV_64F);
    // --- D_z0 构建 ---
    cv::Mat skew_z0 = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(P_c_curr_pred))));
    cv::Mat D_z0 = cv::Mat::zeros(4, 6, CV_64F);
    cv::Mat D_z0_top_left = D_z0(cv::Rect(0, 0, 3, 3));
    D_z0_top_left = -skew_z0;
    cv::Mat D_z0_top_right = D_z0(cv::Rect(3, 0, 3, 3));
    D_z0_top_right = cv::Mat::eye(3, 3, CV_64F);


    // H_full = M * D_p - D_z0
    cv::Mat H_full = M * D_p - D_z0;
    cv::Mat H = H_full(cv::Rect(0, 0, 6, 3)); // 提取前3行

    // 6. 测量更新 (Measurement Update)
    cv::Mat S = H * P * H.t() + R;

    if (cv::determinant(S) < 1e-12) {
        qWarning() << "EKF Update aborted: S matrix is near singular!";
        return;
    }

    cv::Mat K = P * H.t() * S.inv();
    cv::Mat delta_xi = K * y;

    // 7. 状态更新 (Manifold Update)
    // T_sc = T_sc * Exp(delta_xi)
    cv::Mat dT_correction = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi))));
    T_sc_est = T_sc_est * dT_correction;

    // 8. 协方差更新 (Joseph Form)
    cv::Mat I6 = cv::Mat::eye(6, 6, CV_64F);
    cv::Mat ImKH = I6 - K * H;
    P = ImKH * P * ImKH.t() + K * R * K.t();

    // 回写状态
    sta.all.KF.P = P;

    // NIS 调试信息
    cv::Mat nis_mat = y.t() * S.inv() * y;
    double nis = nis_mat.at<double>(0, 0);
    qInfo() << "EKF Update: NIS =" << nis << " Correction norm =" << cv::norm(delta_xi);
}


void MHEC::updateServoRot(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_cs_est)
{
    // 1. 数据校验
    if (log.rot.P_c_curr_list.empty() || log.rot.P_c_prev_list.empty() || log.rot.Twist_c_list.empty()) {
        qWarning() << "Servo Update failed: Data lists are empty.";
        return;
    }

    double dt = cfg.dt;
    if (dt <= 0) dt = 0.01; // 防止除零

    double lambda = sta.rot.Lambda;

    // 2. 提取输入
    cv::Vec3d P_c_curr = log.rot.P_c_curr_list.back();
    cv::Vec3d P_c_prev = log.rot.P_c_prev_list.back();
    cv::Mat twist_c = log.rot.Twist_c_list.back(); // 6x1: [w; v] (Camera frame)

    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));

    // cv::Mat T_cs_est = T_sc_est.inv();

    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));

    // 预测位置 P_c_pred
    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;
    cv::Vec3d P_c_pred(P_c_pred_homo.at<double>(0), P_c_pred_homo.at<double>(1), P_c_pred_homo.at<double>(2));

    // 计算 e = (dP_meas - dP_pred) / dt
    cv::Vec3d dP_pred = P_c_pred - P_c_prev;
    cv::Vec3d dP_meas = P_c_curr - P_c_prev;
    cv::Mat e = (cv::Mat_<double>(3, 1) << (dP_meas[0] - dP_pred[0]) / dt,
                 (dP_meas[1] - dP_pred[1]) / dt,
                 (dP_meas[2] - dP_pred[2]) / dt);

    double e_norm = cv::norm(e);
    qInfo() << "Norm norm_e:" << e_norm;
    // double w_norm = cv::norm(w);
    // qInfo() << "Norm norm_w:" << w_norm;

    // 5. 构建雅可比矩阵 L (3x6)
    // L = -[ [P_c_prev]x * [w]x - [v]x, -[w]x ]
    cv::Mat skew_p = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(P_c_prev))));
    cv::Mat skew_w = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(w))));
    cv::Mat skew_v = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(v))));

    // cv::Mat L = cv::Mat::zeros(3, 6, CV_64F);
    cv::Mat L_left = -(skew_p * skew_w - skew_v);
    // cv::Mat L_right = skew_w; // 注意 Matlab 中是 -(-skew_w) = skew_w

    // L_left.copyTo(L(cv::Rect(0, 0, 3, 3)));
    // L_right.copyTo(L(cv::Rect(3, 0, 3, 3)));

    // 6. 更新状态 (梯度下降/伪逆)
    // delta_xi = -lambda * pinv(L) * e
    cv::Mat L_pinv;
    cv::invert(L_left, L_pinv, cv::DECOMP_SVD);
    cv::Mat delta_seita = -lambda * L_pinv * e;

    cv::Mat delta_xi = cv::Mat::zeros(6, 1, CV_64F);
    delta_seita.copyTo(delta_xi.rowRange(0, 3));

    // T_cs_new = Exp(delta_xi * dt) * T_cs_est
    cv::Mat delta_xi_scaled = delta_xi * dt;

    cv::Mat dT_corr = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi_scaled))));

    cv::Mat T_cs_new = dT_corr * T_cs_est;

    // 7. 写回结果
    T_cs_est = T_cs_new;

    // 调试打印
    qInfo() << "Servo Update: e_norm =" << e_norm << " lambda =" << lambda << " correction norm =" << cv::norm(delta_xi);
}



void MHEC::updateServoT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_cs_est)
{
    // 1. 数据校验
    if (log.t.P_c_curr_list.empty() || log.t.P_c_prev_list.empty() || log.t.Twist_c_list.empty()) {
        qWarning() << "Servo Update failed: Data lists are empty.";
        return;
    }

    double dt = cfg.dt;
    if (dt <= 0) dt = 0.01; // 防止除零

    double lambda = sta.t.Lambda;

    // 2. 提取输入
    cv::Vec3d P_c_curr = log.t.P_c_curr_list.back();
    cv::Vec3d P_c_prev = log.t.P_c_prev_list.back();
    cv::Mat twist_c = log.t.Twist_c_list.back(); // 6x1: [w; v] (Camera frame)

    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));

    // cv::Mat T_cs_est = T_sc_est.inv();

    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));

    // 预测位置 P_c_pred
    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;
    cv::Vec3d P_c_pred(P_c_pred_homo.at<double>(0), P_c_pred_homo.at<double>(1), P_c_pred_homo.at<double>(2));

    // 计算 e = (dP_meas - dP_pred) / dt
    cv::Vec3d dP_pred = P_c_pred - P_c_prev;
    cv::Vec3d dP_meas = P_c_curr - P_c_prev;
    cv::Mat e = (cv::Mat_<double>(3, 1) << (dP_meas[0] - dP_pred[0]) / dt,
                 (dP_meas[1] - dP_pred[1]) / dt,
                 (dP_meas[2] - dP_pred[2]) / dt);

    double e_norm = cv::norm(e);
    qInfo() << "Norm norm_e:" << e_norm;

    // 5. 构建雅可比矩阵 L (3x6)
    // L = -[ [P_c_prev]x * [w]x - [v]x, -[w]x ]
    cv::Mat skew_p = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(P_c_prev))));
    cv::Mat skew_w = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(w))));
    cv::Mat skew_v = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(v))));

    // cv::Mat L = cv::Mat::zeros(3, 6, CV_64F);
    // cv::Mat L_left = -(skew_p * skew_w - skew_v);
    cv::Mat L_right = skew_w; // 注意 Matlab 中是 -(-skew_w) = skew_w

    // L_left.copyTo(L(cv::Rect(0, 0, 3, 3)));
    // L_right.copyTo(L(cv::Rect(3, 0, 3, 3)));

    // 6. 更新状态 (梯度下降/伪逆)
    // delta_xi = -lambda * pinv(L) * e
    cv::Mat L_pinv;
    cv::invert(L_right, L_pinv, cv::DECOMP_SVD);
    cv::Mat delta_t = -lambda * L_pinv * e;

    cv::Mat delta_xi = cv::Mat::zeros(6, 1, CV_64F);
    delta_t.copyTo(delta_xi.rowRange(3, 6));

    // T_cs_new = Exp(delta_xi * dt) * T_cs_est
    cv::Mat delta_xi_scaled = delta_xi * dt;

    cv::Mat dT_corr = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi_scaled))));

    cv::Mat T_cs_new = dT_corr * T_cs_est;

    // 7. 写回结果
    T_cs_est = T_cs_new;

    // 调试打印
    qInfo() << "Servo Update: e_norm =" << e_norm << " lambda =" << lambda << " correction norm =" << cv::norm(delta_xi);
}


void MHEC::updateIEKFRot(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    // 1. 提取最新数据
    if (log.rot.P_c_curr_list.empty() || log.rot.P_c_prev_list.empty() || log.rot.Twist_c_list.empty()) {
        qWarning() << "IEKF Update failed: Data lists are empty.";
        return;
    }

    cv::Mat P = sta.rot.KF.P.clone();
    cv::Mat Q = sta.rot.KF.Q;
    cv::Mat R = sta.rot.KF.R;

    cv::Vec3d P_c_curr = log.rot.P_c_curr_list.back();
    cv::Vec3d P_c_prev = log.rot.P_c_prev_list.back();
    cv::Mat twist_c = log.rot.Twist_c_list.back(); // 6x1: [w_x, w_y, w_z, v_x, v_y, v_z]

    cv::Mat T_cs_est = T_sc_est.inv();

    // 状态预测
    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));

    // 计算运动增量 dT (Exp map)
    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));

    // 预测观测位移
    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;

    cv::Vec3d dP_meas = P_c_curr - P_c_prev;
    cv::Vec3d dP_pred(P_c_pred_homo.at<double>(0) - P_c_prev[0],
                      P_c_pred_homo.at<double>(1) - P_c_prev[1],
                      P_c_pred_homo.at<double>(2) - P_c_prev[2]);

    // --- 新增调试打印 ---
    // double norm_meas = cv::norm(dP_meas);
    // double norm_pred = cv::norm(dP_pred);

    // qInfo() << "---------------------------------------------";
    // qInfo() << "Norm Measured dP:" << norm_meas;
    // qInfo() << "Norm Predicted dP:" << norm_pred;
    // qInfo() << "Scale Ratio (Meas/Pred):" << (norm_pred != 0 ? norm_meas / norm_pred : 0);

    // 计算残差 e
    cv::Mat e = (cv::Mat_<double>(3, 1) << (dP_meas[0] - dP_pred[0]),
                 (dP_meas[1] - dP_pred[1]),
                 (dP_meas[2] - dP_pred[2]));

    // 构建观测矩阵 H (Jacobian)
    // H = [ [P_c_prev]x*[w]x - [v]x, -[w]x ]
    cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat H_left = cvMatFromEigen(VecToso3(eigenFromCvMat(P_c_prev)) * VecToso3(eigenFromCvMat(w))
                                    - VecToso3(eigenFromCvMat(v)));
    // cv::Mat H_right = -cvMatFromEigen(VecToso3(eigenFromCvMat(w)));

    H_left.copyTo(H(cv::Rect(0, 0, 3, 3)));
    // H_right.copyTo(H(cv::Rect(3, 0, 3, 3)));

    // 协方差时间更新
    P = P + Q;
    cv::Mat S = H * P * H.t() + R;
    // 增加奇异性检查，防止求逆失败
    if (cv::determinant(S) < 1e-12) {
        qWarning() << "IEKF Update aborted: S matrix is near singular!";
        return;
    }
    cv::Mat K = P * H.t() * S.inv();

    // 6. NIS 检查 (卡方分布检验)
    cv::Mat nis_mat = e.t() * S.inv() * e;
    double nis = nis_mat.at<double>(0, 0);

    // 打印关键调试信息
    // qInfo() << "--- IEKF Debug Info ---";
    qInfo() << "Residual e: [" << e.at<double>(0) << "," << e.at<double>(1) << "," << e.at<double>(2) << "]";
    double norm_e = cv::norm(e);
    qInfo() << "Norm norm_e:" << norm_e;

    // qInfo() << "NIS Value:" << nis << (nis > 16.27 ? " (REJECTED)" : " (ACCEPTED)");
    // if (nis > 16.27) {
    //     // 如果 NIS 过大，说明当前帧噪声异常（可能是刚才算的 Z 轴 6294 的抖动造成的）
    //     return;
    // }

    // 7. 状态更新 (流形上的指数映射)
    cv::Mat delta_xi_3d = K * e; // 6x1
    cv::Mat delta_xi = cv::Mat::zeros(6, 1, CV_64F);
    delta_xi_3d.copyTo(delta_xi.rowRange(0, 3));
    qInfo() << "delta_xi: [" << delta_xi.at<double>(0) << "," << delta_xi.at<double>(1) << "," << delta_xi.at<double>(2)
            << delta_xi.at<double>(3) << "," << delta_xi.at<double>(4) << ","<< delta_xi.at<double>(5) << "]";

    cv::Mat T_cs_new = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi)))) * T_cs_est;

    // 写回结果并更新 T_sc
    T_sc_est = T_cs_new.inv();

    // 8. 协方差更新 (Joseph Form 保证对称性和正定性)
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat ImKH = I3 - K * H;
    P = ImKH * P * ImKH.t() + K * R * K.t();

    // 更新状态量
    sta.rot.KF.P = P;

    // qInfo() << "Correction delta_xi norm:" << cv::norm(delta_xi);
}
void MHEC::updateIEKFT(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est)
{
    if (log.t.P_c_curr_list.empty() || log.t.P_c_prev_list.empty() || log.t.Twist_c_list.empty()) {
        qWarning() << "IEKF Update failed: Data lists are empty.";
        return;
    }

    cv::Mat P = sta.t.KF.P.clone();
    cv::Mat Q = sta.t.KF.Q;
    cv::Mat R = sta.t.KF.R;

    cv::Vec3d P_c_curr = log.t.P_c_curr_list.back();
    cv::Vec3d P_c_prev = log.t.P_c_prev_list.back();
    cv::Mat twist_c = log.t.Twist_c_list.back(); // 6x1: [w_x, w_y, w_z, v_x, v_y, v_z]

    cv::Mat T_cs_est = T_sc_est.inv();

    // 状态预测
    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));

    // 计算运动增量 dT (Exp map)
    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));

    // 预测观测位移
    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;

    cv::Vec3d dP_meas = P_c_curr - P_c_prev;
    cv::Vec3d dP_pred(P_c_pred_homo.at<double>(0) - P_c_prev[0],
                      P_c_pred_homo.at<double>(1) - P_c_prev[1],
                      P_c_pred_homo.at<double>(2) - P_c_prev[2]);

    // --- 新增调试打印 ---
    // double norm_meas = cv::norm(dP_meas);
    // double norm_pred = cv::norm(dP_pred);

    // qInfo() << "---------------------------------------------";
    // qInfo() << "Norm Measured dP:" << norm_meas;
    // qInfo() << "Norm Predicted dP:" << norm_pred;
    // qInfo() << "Scale Ratio (Meas/Pred):" << (norm_pred != 0 ? norm_meas / norm_pred : 0);

    // 计算残差 e
    cv::Mat e = (cv::Mat_<double>(3, 1) << (dP_meas[0] - dP_pred[0]),
                 (dP_meas[1] - dP_pred[1]),
                 (dP_meas[2] - dP_pred[2]));

    // 构建观测矩阵 H (Jacobian)
    // H = [ [P_c_prev]x*[w]x - [v]x, -[w]x ]
    cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
    // cv::Mat H_left = cvMatFromEigen(VecToso3(eigenFromCvMat(P_c_prev)) * VecToso3(eigenFromCvMat(w))
    //                                 - VecToso3(eigenFromCvMat(v)));
    cv::Mat H_right = -cvMatFromEigen(VecToso3(eigenFromCvMat(w)));

    H_right.copyTo(H(cv::Rect(0, 0, 3, 3)));
    // H_right.copyTo(H(cv::Rect(3, 0, 3, 3)));

    // 协方差时间更新
    P = P + Q;
    cv::Mat S = H * P * H.t() + R;
    // 增加奇异性检查，防止求逆失败
    if (cv::determinant(S) < 1e-12) {
        qWarning() << "IEKF Update aborted: S matrix is near singular!";
        return;
    }
    cv::Mat K = P * H.t() * S.inv();

    // 6. NIS 检查 (卡方分布检验)
    cv::Mat nis_mat = e.t() * S.inv() * e;
    double nis = nis_mat.at<double>(0, 0);

    // 打印关键调试信息
    // qInfo() << "--- IEKF Debug Info ---";
    qInfo() << "Residual e: [" << e.at<double>(0) << "," << e.at<double>(1) << "," << e.at<double>(2) << "]";
    double norm_e = cv::norm(e);
    qInfo() << "Norm norm_e:" << norm_e;

    // qInfo() << "NIS Value:" << nis << (nis > 16.27 ? " (REJECTED)" : " (ACCEPTED)");
    // if (nis > 16.27) {
    //     // 如果 NIS 过大，说明当前帧噪声异常（可能是刚才算的 Z 轴 6294 的抖动造成的）
    //     return;
    // }

    // 7. 状态更新 (流形上的指数映射)
    cv::Mat delta_xi_3d = K * e; // 6x1
    cv::Mat delta_xi = cv::Mat::zeros(6, 1, CV_64F);
    delta_xi_3d.copyTo(delta_xi.rowRange(3, 6));
    qInfo() << "delta_xi: [" << delta_xi.at<double>(0) << "," << delta_xi.at<double>(1) << "," << delta_xi.at<double>(2)
            << delta_xi.at<double>(3) << "," << delta_xi.at<double>(4) << ","<< delta_xi.at<double>(5) << "]";
    cv::Mat T_cs_new = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi)))) * T_cs_est;

    // 写回结果并更新 T_sc
    T_sc_est = T_cs_new.inv();

    // 8. 协方差更新 (Joseph Form 保证对称性和正定性)
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat ImKH = I3 - K * H;
    P = ImKH * P * ImKH.t() + K * R * K.t();

    // 更新状态量
    sta.t.KF.P = P;

    qInfo() << "Correction delta_xi norm:" << cv::norm(delta_xi);
}


void MHEC::updateServoOnce(const MHEC::CalibrationCfg& cfg, CalibrationState& sta, const CalibrationData& log, cv::Mat& T_sc_est, double scale_factor)
{
    // 1. 数据校验
    if (log.all.P_c_curr_list.empty() || log.all.P_c_prev_list.empty() || log.all.Twist_c_list.empty()) {
        qWarning() << "Servo Update failed: Data lists are empty.";
        return;
    }

    double dt = cfg.dt;
    if (dt <= 0) dt = 0.01; // 防止除零

    double lambda = cfg.decay_gain;

    // At the sixth observation, use the complete stacked system once.  This
    // gives all methods the same six-motion observability initialization.
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
        qInfo() << "J-FGD stacked initialization correction:" << cv::norm(delta);
        return;
    }

    // 2. 提取输入
    cv::Vec3d P_c_curr = log.all.P_c_curr_list.back() / scale_factor;
    cv::Vec3d P_c_prev = log.all.P_c_prev_list.back() / scale_factor;
    cv::Mat twist_c = log.all.Twist_c_list.back(); // 6x1: [w; v] (Camera frame)

    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));
    v = v / scale_factor;

    cv::Mat twist_c_scale;
    cv::vconcat(w, v, twist_c_scale);

    cv::Mat T_cs_est = T_sc_est.inv();

    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c_scale))));

    // 预测位置 P_c_pred
    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;
    cv::Vec3d P_c_pred(P_c_pred_homo.at<double>(0), P_c_pred_homo.at<double>(1), P_c_pred_homo.at<double>(2));

    // 计算 e = (dP_meas - dP_pred) / dt
    cv::Vec3d dP_pred = P_c_pred - P_c_prev;
    cv::Vec3d dP_meas = P_c_curr - P_c_prev;
    cv::Mat e = (cv::Mat_<double>(3, 1) << (dP_meas[0] - dP_pred[0]) / dt,
                 (dP_meas[1] - dP_pred[1]) / dt,
                 (dP_meas[2] - dP_pred[2]) / dt);

    double e_norm = cv::norm(e);
    qInfo() << "Norm norm_e:" << e_norm;

    // // 4. 计算自适应 Lambda
    // double lambda_gain = 1000.0; // 对应 Matlab 中的默认值
    // // 如果 sta 中有这个成员，可以取消注释：
    // // lambda_gain = sta.lambda_gain;

    // double alpha = 0.00008;

    // // lambda = gain * e_norm * exp(-alpha / (e_norm + 1e-9))
    // double lambda = lambda_gain * e_norm * std::exp(-alpha / (e_norm + 1e-9));

    // 5. 构建雅可比矩阵 L (3x6)
    // L = -[ [P_c_prev]x * [w]x - [v]x, -[w]x ]
    cv::Mat skew_p = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(P_c_prev))));
    cv::Mat skew_w = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(w))));
    cv::Mat skew_v = cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(v))));

    cv::Mat L = cv::Mat::zeros(3, 6, CV_64F);
    cv::Mat L_left = -(skew_p * skew_w - skew_v);
    cv::Mat L_right = skew_w; // 注意 Matlab 中是 -(-skew_w) = skew_w

    L_left.copyTo(L(cv::Rect(0, 0, 3, 3)));
    L_right.copyTo(L(cv::Rect(3, 0, 3, 3)));

    // 6. 更新状态 (梯度下降/伪逆)
    // delta_xi = -lambda * pinv(L) * e
    // 单次观测的3x6 Jacobian容易包含弱可观方向。OpenCV默认伪逆会保留
    // 所有非零奇异值，较小奇异值可能把几十微米的观测误差放大成毫米级校正。
    // 这里仍采用SVD伪逆，但显式截断相对过小的奇异值。
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
        qWarning() << "J-FGD skipped: no singular direction passed the SVD threshold.";
        return;
    }
    cv::Mat L_pinv = svd.vt.t() * sigmaInv * svd.u.t();
    cv::Mat delta_xi = -lambda * L_pinv * e;

    // T_cs_new = Exp(delta_xi * dt) * T_cs_est
    delta_xi.rowRange(3, 6) = delta_xi.rowRange(3, 6) * scale_factor;
    cv::Mat delta_xi_scaled = delta_xi * dt;

    cv::Mat dT_corr = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(delta_xi_scaled))));

    cv::Mat T_cs_new = dT_corr * T_cs_est;

    // 7. 写回结果
    T_sc_est = T_cs_new.inv();

    // 调试打印
    qInfo() << "Servo Update: e_norm =" << e_norm
            << "lambda =" << lambda
            << "SVD rank =" << retainedRank
            << "retained condition =" << sigmaMax / sigmaMinRetained
            << "correction norm =" << cv::norm(delta_xi);
}


// void MHEC::planCamMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, double vel_v, double vel_w, double dt) {
//     // 初始化随机数生成器
//     static std::random_device rd;
//     static std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> dist(-1.0, 1.0);
//     // std::uniform_real_distribution<double> step_dist(0.01, max_step);

//     cv::Mat w = (cv::Mat_<double>(3, 1) << dist(gen), dist(gen), dist(gen));
//     double w_norm = cv::norm(w);
//     if (w_norm < 1e-6) { // 防止极小值
//         w = (cv::Mat_<double>(3, 1) << 0, 0, 1);
//     } else {
//         w /= w_norm;
//     }

//     cv::Mat v = (cv::Mat_<double>(3, 1) << dist(gen), dist(gen), dist(gen));
//     double v_norm = cv::norm(v);
//     if (v_norm < 1e-6) { // 防止极小值
//         v = (cv::Mat_<double>(3, 1) << 0, 0, 1);
//     } else {
//         v /= v_norm;
//     }

//     cv::Mat S_c = cv::Mat(6, 1, CV_64F);
//     w.copyTo(S_c.rowRange(0, 3));
//     v.copyTo(S_c.rowRange(3, 6));

//     twist_c = cv::Mat(6, 1, CV_64F);
//     double theta_w = vel_w * dt;
//     double theta_v = vel_v * dt;
//     cv::Mat w_final = w * theta_w;
//     cv::Mat v_final = v * theta_v;
//     w_final.copyTo(twist_c.rowRange(0, 3));
//     v_final.copyTo(twist_c.rowRange(3, 6));

//     twist_s = cv::Mat(6, 1, CV_64F);
//     twist_s = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est))) * twist_c;

//     // 指数映射 Exp([S]theta)
//     cv::Mat T_inc = cv::Mat::eye(4, 4, CV_64F);
//     T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));

//     // 更新位姿
//     cv::Mat T_new = cv::Mat::eye(4, 4, CV_64F);
//     T_new = T_inc * T_se_curr;
//     T_se_curr = T_new;
// }

namespace { int g_cam_motion_index = 0; }

void MHEC::resetCamMotionSequence(int startIndex) { g_cam_motion_index = std::max(0, startIndex); }

void MHEC::planCamMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, double vel_v, double vel_w, double dt) {
    static const std::vector<cv::Vec3d> initial_trans_dirs = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    static const std::vector<cv::Vec3d> initial_rot_axes = {
        {1, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1}
    };
    // 预定义的最优平移激励方向 (14个方向：6个面 + 8个顶点)
    static const std::vector<cv::Vec3d> trans_dirs = {
        {0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},  // 正交轴
        {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1},                      // 空间对角线(上半球)
        {1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}                   // 空间对角线(下半球)
    };

    // 预定义的最优旋转激励轴 (13个方向：为了与平移数量14互素，产生丰富的螺旋运动组合)
    static const std::vector<cv::Vec3d> rot_axes = {
        {0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, // 优先主轴
        {1, 1, 0}, {-1, -1, 0}, {1, -1, 0}, {-1, 1, 0},                      // XY 平面斜轴
        {1, 0, 1}, {0, 1, 1}, {1, 1, 1}                                      // 空间耦合轴
    };

    int& idx = g_cam_motion_index;

    // 分别从两个互素的列表中提取方向
    cv::Vec3d current_v;
    cv::Vec3d current_w;
    if (idx < static_cast<int>(initial_trans_dirs.size())) {
        current_v = initial_trans_dirs[idx];
        current_w = initial_rot_axes[idx];
    } else {
        const int sequence_idx = idx - static_cast<int>(initial_trans_dirs.size());
        current_v = trans_dirs[sequence_idx % trans_dirs.size()];
        current_w = rot_axes[sequence_idx % rot_axes.size()];
    }
    idx++; // 增加计数

    // 1. 构建角速度 w 并归一化
    cv::Mat w = (cv::Mat_<double>(3, 1) << current_w[0], current_w[1], current_w[2]);
    double w_norm = cv::norm(w);
    if (w_norm > 1e-6) {
        w /= w_norm;
    } else {
        w = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    }

    // 2. 构建线速度 v 并归一化
    cv::Mat v = (cv::Mat_<double>(3, 1) << current_v[0], current_v[1], current_v[2]);
    double v_norm = cv::norm(v);
    if (v_norm > 1e-6) {
        v /= v_norm;
    } else {
        v = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    }

    // 3. 组装相机坐标系下的速度旋量 S_c
    cv::Mat S_c = cv::Mat(6, 1, CV_64F);
    w.copyTo(S_c.rowRange(0, 3));
    v.copyTo(S_c.rowRange(3, 6));

    // 4. 赋予步长缩放 (vel_v, vel_w) 并生成 twist_c
    twist_c = cv::Mat(6, 1, CV_64F);
    double theta_w = vel_w * dt;
    double theta_v = vel_v * dt;
    cv::Mat w_final = w * theta_w;
    cv::Mat v_final = v * theta_v;
    w_final.copyTo(twist_c.rowRange(0, 3));
    v_final.copyTo(twist_c.rowRange(3, 6));

    // 5. 转换到基座坐标系下的 twist_s
    twist_s = cv::Mat(6, 1, CV_64F);
    twist_s = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est))) * twist_c;

    // 6. 指数映射 Exp([S]theta) 更新位姿
    cv::Mat T_inc = cv::Mat::eye(4, 4, CV_64F);
    T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));

    // 7. 更新 T_se_curr
    cv::Mat T_new = cv::Mat::eye(4, 4, CV_64F);
    T_new = T_inc * T_se_curr;
    T_se_curr = T_new;
}

// void MHEC::planCamTransMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, double vel_v, double dt) {
//     // 初始化随机数生成器
//     static std::random_device rd;
//     static std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> dist(-1.0, 1.0);
//     // std::uniform_real_distribution<double> step_dist(0.01, max_step);

//     cv::Mat w = (cv::Mat_<double>(3, 1) << 0, 0, 0);

//     cv::Mat v = (cv::Mat_<double>(3, 1) << dist(gen), dist(gen), dist(gen));
//     double v_norm = cv::norm(v);
//     if (v_norm < 1e-6) { // 防止极小值
//         v = (cv::Mat_<double>(3, 1) << 0, 0, 1);
//     } else {
//         v /= v_norm;
//     }

//     cv::Mat S_c = cv::Mat(6, 1, CV_64F);
//     w.copyTo(S_c.rowRange(0, 3));
//     v.copyTo(S_c.rowRange(3, 6));

//     twist_c = cv::Mat(6, 1, CV_64F);
//     double theta_w = 0;
//     double theta_v = vel_v * dt;
//     cv::Mat w_final = w * theta_w;
//     cv::Mat v_final = v * theta_v;
//     w_final.copyTo(twist_c.rowRange(0, 3));
//     v_final.copyTo(twist_c.rowRange(3, 6));

//     twist_s = cv::Mat(6, 1, CV_64F);
//     twist_s = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est))) * twist_c;

//     // 指数映射 Exp([S]theta)
//     cv::Mat T_inc = cv::Mat::eye(4, 4, CV_64F);
//     T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));

//     // 更新位姿
//     cv::Mat T_new = cv::Mat::eye(4, 4, CV_64F);
//     T_new = T_inc * T_se_curr;
//     T_se_curr = T_new;
// }

void MHEC::planCamTransMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, double vel_v, double dt) {
    // 预定义的最优空间激励方向序列 (共26个方向，涵盖正交、平面斜向、空间对角线)
    static const std::vector<cv::Vec3d> optimal_dirs = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, // 正交轴 (6)
        {1, 1, 0}, {-1, -1, 0}, {1, -1, 0}, {-1, 1, 0},                      // XY平面 (4)
        {1, 0, 1}, {-1, 0, -1}, {1, 0, -1}, {-1, 0, 1},                      // XZ平面 (4)
        {0, 1, 1}, {0, -1, -1}, {0, 1, -1}, {0, -1, 1},                      // YZ平面 (4)
        {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1},                      // 空间对角线 (上半球) (4)
        {1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}                   // 空间对角线 (下半球) (4)
    };

    // 使用静态变量记录当前的索引，每次调用按顺序提取一个最优方向
    static int trans_idx = 0;
    cv::Vec3d current_dir = optimal_dirs[trans_idx % optimal_dirs.size()];
    trans_idx++;

    // 赋值并归一化
    cv::Mat v = (cv::Mat_<double>(3, 1) << current_dir[0], current_dir[1], current_dir[2]);
    double v_norm = cv::norm(v);
    if (v_norm > 1e-6) {
        v /= v_norm;
    } else {
        v = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    }

    cv::Mat w = cv::Mat::zeros(3, 1, CV_64F); // 纯平移，角速度为0

    // 构建相机坐标系下的 twist_c
    cv::Mat S_c = cv::Mat(6, 1, CV_64F);
    w.copyTo(S_c.rowRange(0, 3));
    v.copyTo(S_c.rowRange(3, 6));

    twist_c = cv::Mat(6, 1, CV_64F);
    double theta_w = 0;
    double theta_v = vel_v * dt;
    cv::Mat w_final = w * theta_w;
    cv::Mat v_final = v * theta_v;
    w_final.copyTo(twist_c.rowRange(0, 3));
    v_final.copyTo(twist_c.rowRange(3, 6));

    // 转换到基座坐标系下的 twist_s
    twist_s = cv::Mat(6, 1, CV_64F);
    twist_s = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est))) * twist_c;

    // 指数映射 Exp([S]theta) 更新位姿
    cv::Mat T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));
    T_se_curr = T_inc * T_se_curr;
}

// void MHEC::planCamRotMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, cv::Vec3d P_c, double vel_w, double dt) {
//     // 初始化随机数生成器
//     static std::random_device rd;
//     static std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> dist(-1.0, 1.0);
//     // std::uniform_real_distribution<double> step_dist(0.01, max_step);

//     cv::Mat w = (cv::Mat_<double>(3, 1) << dist(gen), dist(gen), dist(gen));
//     double w_norm = cv::norm(w);
//     if (w_norm < 1e-6) { // 防止极小值
//         w = (cv::Mat_<double>(3, 1) << 0, 0, 1);
//     } else {
//         w /= w_norm;
//     }

//     w = w * vel_w;

//     // v = P_c x w
//     cv::Mat Pc_mat = (cv::Mat_<double>(3, 1) << P_c[0], P_c[1], P_c[2]);
//     cv::Mat v = -w.cross(Pc_mat); // 线速度

//     cv::Mat S_c = cv::Mat(6, 1, CV_64F);
//     w.copyTo(S_c.rowRange(0, 3));
//     v.copyTo(S_c.rowRange(3, 6));

//     // cv::Mat vp = v+w.cross(Pc_mat);
//     // qDebug() << "vp values:" << vp.at<double>(0) << vp.at<double>(1) << vp.at<double>(2);
//     // qDebug() << "Pc_mat values:" << Pc_mat.at<double>(0) << Pc_mat.at<double>(1) << Pc_mat.at<double>(2);
//     // qDebug() << "w values:" << w.at<double>(0) << w.at<double>(1) << w.at<double>(2);


//     twist_c = cv::Mat(6, 1, CV_64F);
//     // double theta =   dt;
//     twist_c = S_c * dt;
//     // qDebug() << "twist_c values:" << twist_c.at<double>(0) << twist_c.at<double>(1) << twist_c.at<double>(2)
//     //         << twist_c.at<double>(3) << twist_c.at<double>(4) << twist_c.at<double>(5);


//     // cv::Mat w_t = twist_c.rowRange(0, 3);
//     // cv::Mat v_t = twist_c.rowRange(3, 6);
//     // vp = v_t + w_t.cross(Pc_mat);
//     // qDebug() << "--- After dt scaling ---";
//     // qDebug() << "w_t (scaled w):" << w_t.at<double>(0) << w_t.at<double>(1) << w_t.at<double>(2);
//     // qDebug() << "v_t (scaled v):" << v_t.at<double>(0) << v_t.at<double>(1) << v_t.at<double>(2);
//     // qDebug() << "vp (should be near 0 if pure rotation around Pc):"
//     //          << vp.at<double>(0) << vp.at<double>(1) << vp.at<double>(2);

//     twist_s = cv::Mat(6, 1, CV_64F);
//     twist_s = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est))) * twist_c;

//     // 指数映射 Exp([S]theta)
//     cv::Mat T_inc = cv::Mat::eye(4, 4, CV_64F);
//     T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));

//     // 更新位姿
//     cv::Mat T_new = cv::Mat::eye(4, 4, CV_64F);
//     T_new = T_inc * T_se_curr;
//     T_se_curr = T_new;

// }

void MHEC::planCamRotMotion(cv::Mat& T_se_curr, cv::Mat& twist_c, cv::Mat& twist_s, cv::Mat T_sc_est, cv::Vec3d P_c, double vel_w, double dt) {
    // 旋转激励序列：优先保证三个主轴的正反向旋转，再加入耦合面旋转
    static const std::vector<cv::Vec3d> optimal_axes = {
        {0, 0, 1}, {0, 0, -1},               // 绕 Z 轴 (最安全，特征变形小)
        {1, 0, 0}, {-1, 0, 0},               // 绕 X 轴
        {0, 1, 0}, {0, -1, 0},               // 绕 Y 轴
        {1, 1, 0}, {-1, -1, 0},              // XY 平面对角线轴
        {1, 0, 1}, {-1, 0, -1},              // XZ 平面对角线轴
        {0, 1, 1}, {0, -1, -1},              // YZ 平面对角线轴
        {1, 1, 1}, {-1, -1, -1},             // 空间全耦合轴
        {1, -1, 1}, {-1, 1, -1}
    };

    static int rot_idx = 0;
    cv::Vec3d current_axis = optimal_axes[rot_idx % optimal_axes.size()];
    rot_idx++;

    cv::Mat w = (cv::Mat_<double>(3, 1) << current_axis[0], current_axis[1], current_axis[2]);
    double w_norm = cv::norm(w);
    if (w_norm > 1e-6) {
        w /= w_norm;
    } else {
        w = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    }

    w = w * vel_w; // 赋予旋转速度

    // 为了保持特征点在视野中心，计算相机的补偿线速度 v = P_c x w
    cv::Mat Pc_mat = (cv::Mat_<double>(3, 1) << P_c[0], P_c[1], P_c[2]);
    cv::Mat v = -w.cross(Pc_mat);

    // 构建 twist_c
    cv::Mat S_c = cv::Mat(6, 1, CV_64F);
    w.copyTo(S_c.rowRange(0, 3));
    v.copyTo(S_c.rowRange(3, 6));

    twist_c = S_c * dt; // 直接乘 dt，此时 twist_c 已经是位移增量量级

    // 转换到基座坐标系
    twist_s = cv::Mat(6, 1, CV_64F);
    twist_s = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est))) * twist_c;

    // 指数映射 Exp([S]theta) 更新位姿
    cv::Mat T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));
    T_se_curr = T_inc * T_se_curr;
}

void MHEC::planSMotion(cv::Mat& T_se_curr, cv::Mat& twist_s, double vel_w, double vel_v, double dt)
{
    // 初始化随机数生成器
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    // std::uniform_real_distribution<double> step_dist(0.01, max_step);

    cv::Mat w = (cv::Mat_<double>(3, 1) << dist(gen), dist(gen), dist(gen));
    double w_norm = cv::norm(w);
    if (w_norm < 1e-6) { // 防止极小值
        w = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    } else {
        w /= w_norm;
    }

    cv::Mat v = (cv::Mat_<double>(3, 1) << dist(gen), dist(gen), dist(gen));
    double v_norm = cv::norm(v);
    if (v_norm < 1e-6) { // 防止极小值
        v = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    } else {
        v /= v_norm;
    }

    cv::Mat S_s = cv::Mat(6, 1, CV_64F);
    w.copyTo(S_s.rowRange(0, 3));
    v.copyTo(S_s.rowRange(3, 6));

    twist_s = cv::Mat(6, 1, CV_64F);
    double theta_w = vel_w * dt;
    double theta_v = vel_v * dt;
    cv::Mat w_final = w * theta_w;
    cv::Mat v_final = v * theta_v;
    w_final.copyTo(twist_s.rowRange(0, 3));
    v_final.copyTo(twist_s.rowRange(3, 6));

    // 指数映射 Exp([S]theta)
    cv::Mat T_inc = cv::Mat::eye(4, 4, CV_64F);
    T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));

    // 更新位姿
    cv::Mat T_new = cv::Mat::eye(4, 4, CV_64F);
    T_new = T_inc * T_se_curr;
    T_se_curr = T_new;
}

void MHEC::planSTransMotion(cv::Mat& T_se_curr, cv::Mat& twist_s, double vel_v, double dt)
{
    // 初始化随机数生成器
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    // std::uniform_real_distribution<double> step_dist(0.01, max_step);

    cv::Mat w = (cv::Mat_<double>(3, 1) << 0, 0, 0);

    cv::Mat v = (cv::Mat_<double>(3, 1) << dist(gen), dist(gen), dist(gen));
    double v_norm = cv::norm(v);
    if (v_norm < 1e-6) { // 防止极小值
        v = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    } else {
        v /= v_norm;
    }

    cv::Mat S_s = cv::Mat(6, 1, CV_64F);
    w.copyTo(S_s.rowRange(0, 3));
    v.copyTo(S_s.rowRange(3, 6));

    twist_s = cv::Mat(6, 1, CV_64F);
    double theta_v = vel_v * dt;
    cv::Mat w_final = w;
    cv::Mat v_final = v * theta_v;
    w_final.copyTo(twist_s.rowRange(0, 3));
    v_final.copyTo(twist_s.rowRange(3, 6));

    // 指数映射 Exp([S]theta)
    cv::Mat T_inc = cv::Mat::eye(4, 4, CV_64F);
    T_inc = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_s))));

    // 更新位姿
    cv::Mat T_new = cv::Mat::eye(4, 4, CV_64F);
    T_new = T_inc * T_se_curr;
    T_se_curr = T_new;
}


// void MHEC::saveCalbrationData(const CalibrationData& log, const std::string& filename) {
//     // 1. 路径与文件名处理：增加时间戳
//     QFileInfo fileInfo(QString::fromStdString(filename));
//     QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

//     QString newFileName;
//     if (fileInfo.suffix().isEmpty()) {
//         newFileName = fileInfo.absoluteFilePath() + "_" + timestamp + ".csv";
//     } else {
//         newFileName = fileInfo.absolutePath() + "/" + fileInfo.completeBaseName() + "_" + timestamp + "." + fileInfo.suffix();
//     }

//     std::ofstream outFile(newFileName.toStdString());
//     if (!outFile.is_open()) {
//         qDebug() << "Error: Could not open file for writing:" << newFileName;
//         return;
//     }

//     // 设置浮点数保存精度
//     outFile << std::fixed << std::setprecision(8);

//     // 2. 核心 Lambda 函数：负责保存 data 对象（rot/t/all）
//     auto saveDataSection = [&](const std::string& sectionName, const CalibrationData::Data& data) {
//         // --- 逻辑修正点 1: 只要任一列表不为空，就视为有数据 ---
//         size_t rowCount = std::max({
//             data.T_cs_list.size(),
//             data.Twist_c_list.size(),
//             data.Twist_s_list.size(),
//             data.P_c_prev_list.size(),
//             data.P_c_curr_list.size(),
//             data.P_s_prev_list.size(),
//             data.P_s_curr_list.size()
//         });

//         if (rowCount == 0) {
//             outFile << "\n# === SECTION: " << sectionName << " (NO DATA) ===\n";
//             return;
//         }

//         outFile << "\n# === SECTION: " << sectionName << " ===\n";

//         // 3. 写入表头
//         outFile << "Index,";
//         for(int i=0; i<16; ++i) outFile << "Tcs_" << i << ",";
//         for(int i=0; i<6; ++i)  outFile << "TwistC_" << i << ",";
//         for(int i=0; i<6; ++i)  outFile << "TwistS_" << i << ",";
//         outFile << "Pc_prev_X,Pc_prev_Y,Pc_prev_Z,Pc_curr_X,Pc_curr_Y,Pc_curr_Z,";
//         outFile << "Ps_prev_X,Ps_prev_Y,Ps_prev_Z,Ps_curr_X,Ps_curr_Y,Ps_curr_Z\n";

//         // 4. 遍历所有行
//         for (size_t i = 0; i < rowCount; ++i) {
//             outFile << i << ",";

//             // --- A. 保存 T_cs_list (4x4) ---
//             if (i < data.T_cs_list.size() && !data.T_cs_list[i].empty()) {
//                 cv::Mat T;
//                 data.T_cs_list[i].convertTo(T, CV_64F);
//                 for(int j=0; j<16; ++j) outFile << T.at<double>(j) << ",";
//             } else {
//                 for(int j=0; j<16; ++j) outFile << "0,";
//             }

//             // --- B. 保存 Twist (相机端 & 机器人端) ---
//             auto writeTwistFunc = [&](const std::vector<cv::Mat>& list, size_t idx) {
//                 if (idx < list.size() && !list[idx].empty()) {
//                     cv::Mat temp;
//                     list[idx].convertTo(temp, CV_64F);
//                     int total = (int)temp.total();
//                     for(int j=0; j<6; ++j) outFile << (j < total ? temp.at<double>(j) : 0.0) << ",";
//                 } else {
//                     for(int j=0; j<6; ++j) outFile << "0,";
//                 }
//             };
//             writeTwistFunc(data.Twist_c_list, i);
//             writeTwistFunc(data.Twist_s_list, i);

//             // --- C. 保存 Camera 点 Pc ---
//             auto writePointFunc = [&](const std::vector<cv::Vec3d>& list, size_t idx) {
//                 if (idx < list.size()) {
//                     outFile << list[idx][0] << "," << list[idx][1] << "," << list[idx][2] << ",";
//                 } else {
//                     outFile << "0,0,0,";
//                 }
//             };
//             writePointFunc(data.P_c_prev_list, i);
//             writePointFunc(data.P_c_curr_list, i);

//             // --- D. 保存 Sensor 点 Ps ---
//             writePointFunc(data.P_s_prev_list, i);
//             // Ps_curr 是最后的一组数据，处理一下逗号/换行
//             if (i < data.P_s_curr_list.size()) {
//                 outFile << data.P_s_curr_list[i][0] << "," << data.P_s_curr_list[i][1] << "," << data.P_s_curr_list[i][2] << "\n";
//             } else {
//                 outFile << "0,0,0\n";
//             }
//         }

//         // --- E. 保存标定结果矩阵 T_cs ---
//         outFile << "FINAL_CALIB_MATRIX_TCS,";
//         if (!data.T_cs.empty() && data.T_cs.rows == 4) {
//             cv::Mat resT;
//             data.T_cs.convertTo(resT, CV_64F);
//             for(int j=0; j<16; ++j) outFile << resT.at<double>(j) << (j==15 ? "" : ",");
//         } else {
//             // 如果没标定结果，输出单位阵
//             for(int j=0; j<16; ++j) outFile << (j%5==0 ? "1" : "0") << (j==15 ? "" : ",");
//         }
//         outFile << "\n";
//     };

//     // 5. 执行保存
//     saveDataSection("ALL_DATA", log.all);
//     saveDataSection("ROT_ONLY", log.rot);
//     saveDataSection("TRANS_ONLY", log.t);

//     outFile.close();
//     qDebug() << "Calibration Data saved successfully to:" << newFileName;
// }

void MHEC::saveCalbrationData(const CalibrationData& log, const std::string& filename) {
    // 1. 路径与文件名处理：增加时间戳
    QFileInfo fileInfo(QString::fromStdString(filename));
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

    QString newFileName;
    if (fileInfo.suffix().isEmpty()) {
        newFileName = fileInfo.absoluteFilePath() + "_" + timestamp + ".csv";
    } else {
        newFileName = fileInfo.absolutePath() + "/" + fileInfo.completeBaseName() + "_" + timestamp + "." + fileInfo.suffix();
    }

    std::ofstream outFile(newFileName.toStdString());
    if (!outFile.is_open()) {
        qDebug() << "Error: Could not open file for writing:" << newFileName;
        throw std::runtime_error("Could not open calibration-data output file");
    }

    // 设置浮点数保存精度
    outFile << std::fixed << std::setprecision(8);

    // 2. 核心 Lambda 函数：负责保存 data 对象（rot/t/all）
    auto saveDataSection = [&](const std::string& sectionName, const CalibrationData::Data& data) {
        // --- 逻辑修正点 1: 只要任一列表不为空，就视为有数据 ---
        size_t rowCount = std::max({
            data.T_cs_list.size(),
            data.T_se_list.size(),
            data.Twist_c_list.size(),
            data.Twist_s_list.size(),
            data.P_c_prev_list.size(),
            data.P_c_curr_list.size(),
            data.P_s_prev_list.size(),
            data.P_s_curr_list.size()
        });

        if (rowCount == 0) {
            outFile << "\n# === SECTION: " << sectionName << " (NO DATA) ===\n";
            return;
        }

        outFile << "\n# === SECTION: " << sectionName << " ===\n";

        // 3. 写入表头
        outFile << "Index,";
        for(int i=0; i<16; ++i) outFile << "Tcs_" << i << ",";
        for(int i=0; i<16; ++i) outFile << "Tse_" << i << ",";
        for(int i=0; i<6; ++i)  outFile << "TwistC_" << i << ",";
        for(int i=0; i<6; ++i)  outFile << "TwistS_" << i << ",";
        outFile << "Pc_prev_X,Pc_prev_Y,Pc_prev_Z,Pc_curr_X,Pc_curr_Y,Pc_curr_Z,";
        outFile << "Ps_prev_X,Ps_prev_Y,Ps_prev_Z,Ps_curr_X,Ps_curr_Y,Ps_curr_Z\n";

        // 4. 遍历所有行
        for (size_t i = 0; i < rowCount; ++i) {
            outFile << i << ",";

            // --- A. 保存 T_cs_list (4x4) ---
            if (i < data.T_cs_list.size() && !data.T_cs_list[i].empty()) {
                cv::Mat T;
                data.T_cs_list[i].convertTo(T, CV_64F);
                for(int j=0; j<16; ++j) outFile << T.at<double>(j) << ",";
            } else {
                for(int j=0; j<16; ++j) outFile << "0,";
            }

            // --- B. 保存 T_se_list (4x4) ---
            if (i < data.T_se_list.size() && !data.T_se_list[i].empty()) {
                cv::Mat T;
                data.T_se_list[i].convertTo(T, CV_64F);
                for(int j=0; j<16; ++j) outFile << T.at<double>(j) << ",";
            } else {
                for(int j=0; j<16; ++j) outFile << "0,";
            }

            // --- C. 保存 Twist (相机端 & 机器人端) ---
            auto writeTwistFunc = [&](const std::vector<cv::Mat>& list, size_t idx) {
                if (idx < list.size() && !list[idx].empty()) {
                    cv::Mat temp;
                    list[idx].convertTo(temp, CV_64F);
                    int total = (int)temp.total();
                    for(int j=0; j<6; ++j) outFile << (j < total ? temp.at<double>(j) : 0.0) << ",";
                } else {
                    for(int j=0; j<6; ++j) outFile << "0,";
                }
            };
            writeTwistFunc(data.Twist_c_list, i);
            writeTwistFunc(data.Twist_s_list, i);

            // --- D. 保存 Camera 点 Pc ---
            auto writePointFunc = [&](const std::vector<cv::Vec3d>& list, size_t idx) {
                if (idx < list.size()) {
                    outFile << list[idx][0] << "," << list[idx][1] << "," << list[idx][2] << ",";
                } else {
                    outFile << "0,0,0,";
                }
            };
            writePointFunc(data.P_c_prev_list, i);
            writePointFunc(data.P_c_curr_list, i);

            // --- E. 保存 Sensor 点 Ps ---
            writePointFunc(data.P_s_prev_list, i);
            // Ps_curr 是最后的一组数据，处理一下逗号/换行
            if (i < data.P_s_curr_list.size()) {
                outFile << data.P_s_curr_list[i][0] << "," << data.P_s_curr_list[i][1] << "," << data.P_s_curr_list[i][2] << "\n";
            } else {
                outFile << "0,0,0\n";
            }
        }

        // --- F. 保存标定结果矩阵 T_cs ---
        outFile << "FINAL_CALIB_MATRIX_TCS,";
        if (!data.T_cs.empty() && data.T_cs.rows == 4) {
            cv::Mat resT;
            data.T_cs.convertTo(resT, CV_64F);
            for(int j=0; j<16; ++j) outFile << resT.at<double>(j) << (j==15 ? "" : ",");
        } else {
            // 如果没标定结果，输出单位阵
            for(int j=0; j<16; ++j) outFile << (j%5==0 ? "1" : "0") << (j==15 ? "" : ",");
        }
        outFile << "\n";
    };

    // 5. 执行保存
    saveDataSection("ALL_DATA", log.all);
    saveDataSection("ROT_ONLY", log.rot);
    saveDataSection("TRANS_ONLY", log.t);

    outFile.flush();
    if (!outFile.good()) {
        outFile.close();
        throw std::runtime_error("Failed while writing calibration-data output file");
    }
    outFile.close();
    qDebug() << "Calibration Data saved successfully to:" << newFileName;
}
