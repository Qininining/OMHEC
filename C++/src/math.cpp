#include <mhec/math.hpp>

MatrixXd Adjoint(Matrix4d T)
{
    MatrixXd Ad(6, 6);
    Ad = MatrixXd::Zero(6, 6);
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);
    Ad.block<3, 3>(0, 0) = R;
    Ad.block<3, 3>(3, 0) = VecToso3(p) * R;
    Ad.block<3, 3>(3, 3) = R;
    return Ad;
}

Matrix4d VecTose3(VectorXd V)
{
    Matrix4d se3 = MatrixXd::Zero(4, 4);
    Matrix3d so3 = VecToso3(V.block<3, 1>(0, 0));
    se3.block<3, 4>(0, 0) << so3, V.block<3, 1>(3, 0);
    return se3;
}


Matrix4d MatrixExp6(Matrix4d se3mat)
{
    Matrix4d T = MatrixXd::Identity(4, 4);
    Matrix3d  so3mat = se3mat.block<3, 3>(0, 0);
    Vector3d omgtheta = so3ToVec(so3mat);
    if (NearZear(omgtheta.norm()))
        T.block<3, 1>(0, 3) = se3mat.block<3, 1>(0, 3);
    else {
        Vector3d omghat; double theta;
        AxisAng3(omgtheta, omghat, theta);
        Matrix3d omgmat = so3mat / theta;
        T.block<3, 3>(0, 0) << MatrixExp3(so3mat);
        T.block<3, 1>(0, 3) << (MatrixXd::Identity(3, 3) * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * omgmat * omgmat) * se3mat.block<3, 1>(0, 3) / theta;
    }
    return T;
}

Matrix3d VecToso3(Vector3d omg)
{
    Matrix3d so3 = Matrix3d::Zero(3, 3);
    so3(0, 1) = -omg(2);   so3(0, 2) = omg(1);
    so3(1, 0) = omg(2);    so3(1, 2) = -omg(0);
    so3(2, 0) = -omg(1);   so3(2, 1) = omg(0);
    return so3;
}

void RpToTrans(Matrix3d R, Vector3d p, Matrix4d& T)
{
    T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) << R;
    T.block<3, 1>(0, 3) << p;
}

void TransToRp(Matrix4d T, Matrix3d& R, Vector3d& p)
{
    R << T.block<3, 3>(0, 0);
    p << T.block<3, 1>(0, 3);
}

Vector3d so3ToVec(Matrix3d  omg)
{
    Vector3d v(omg(2, 1), omg(0, 2), omg(1, 0));
    return v;
}

bool NearZear(double z)
{
    return abs(z) < 1e-6;
}

void AxisAng3(Vector3d expc3, Vector3d& unitV, double& theta)
{
    unitV = Normalize(expc3);
    theta = expc3.norm();
}

Vector3d Normalize(Vector3d  V)
{
    return V / V.norm();
}

Matrix3d MatrixExp3(Matrix3d so3mat)
{
    Vector3d omgtheta = so3ToVec(so3mat);
    if (NearZear(omgtheta.norm()))
        return Matrix3d::Identity();
    else {
        double theta = omgtheta.norm();
        Matrix3d omgmat = so3mat / theta;
        return  Matrix3d::Identity() + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
    }
}

MatrixXd SlistToBlist(MatrixXd Slist, Matrix4d M)
{
    int N = Slist.cols();
    MatrixXd Blist(6, N);
    MatrixXd Ad_M_inv = Adjoint(TransInv(M));
    for (int i = 0; i < N; i++)
    {
        Blist.col(i) = Ad_M_inv * Slist.col(i);
    }
    return Blist;
}

Matrix4d TransInv(Matrix4d T) {
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);
    Matrix3d Rt = R.transpose();
    Matrix4d Tinv = MatrixXd::Identity(4, 4);
    Tinv.block<3, 3>(0, 0) = Rt;
    Tinv.block<3, 1>(0, 3) = -Rt * p;
    return Tinv;
}

MatrixXd JacobianBody(MatrixXd Blist, VectorXd thetalist) {
    int num = thetalist.size();
    MatrixXd Jb = Blist; Matrix4d T = MatrixXd::Identity(4, 4);
    for (int i = num - 2; i >= 0; i--) {
        T *= MatrixExp6(VecTose3(Blist.col(i + 1) * (-thetalist(i + 1))));
        Jb.col(i) = Adjoint(T) * Blist.col(i);
    }
    return Jb;
}

Matrix4d FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist) {
    Matrix4d T = M;
    for (int i = thetalist.size() - 1; i >= 0; i--)
        T = MatrixExp6(VecTose3(Slist.col(i) * thetalist(i))) * T;
    return T;
}

Matrix4d FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist) {
    Matrix4d T = M;
    for (int i = 0; i < thetalist.size(); i++)
        T *= MatrixExp6(VecTose3(Blist.col(i) * thetalist(i)));
    return T;
}

Matrix4d MatrixLog6(Matrix4d T) {
    Matrix4d logT = Matrix4d::Zero();
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);
    
    Matrix3d omgmat = Matrix3d::Zero();
    
    // Check if R is identity
    if ((R - Matrix3d::Identity()).norm() < 1e-6) {
        logT.block<3, 1>(0, 3) = p;
        return logT;
    }
    
    // Compute rotation part
    double theta = acos((R.trace() - 1.0) / 2.0);
    omgmat = (R - R.transpose()) / (2.0 * sin(theta)) * theta;
    
    logT.block<3, 3>(0, 0) = omgmat;
    
    // Compute translation part
    Matrix3d G_inv = Matrix3d::Identity() / theta 
                    - omgmat / 2.0 
                    + (1.0 / theta - 1.0 / (2.0 * tan(theta / 2.0))) * omgmat * omgmat / theta;
    logT.block<3, 1>(0, 3) = G_inv * p;
    
    return logT;
}

VectorXd se3ToVec(Matrix4d se3mat) {
    VectorXd V(6);
    V.block<3, 1>(0, 0) = so3ToVec(se3mat.block<3, 3>(0, 0));
    V.block<3, 1>(3, 0) = se3mat.block<3, 1>(0, 3);
    return V;
}

MatrixXd ad(VectorXd V) {
    MatrixXd adV = MatrixXd::Zero(6, 6);
    Matrix3d omgmat = VecToso3(V.block<3, 1>(0, 0));
    Matrix3d vmat = VecToso3(V.block<3, 1>(3, 0));
    
    adV.block<3, 3>(0, 0) = omgmat;
    adV.block<3, 3>(3, 0) = vmat;
    adV.block<3, 3>(3, 3) = omgmat;
    
    return adV;
}

MatrixXd get_dexp_inv(VectorXd S) {
    Vector3d w = S.block<3, 1>(0, 0);
    Vector3d v = S.block<3, 1>(3, 0);
    double theta = w.norm();
    
    MatrixXd dexp_inv = MatrixXd::Identity(6, 6);
    
    if (theta < 1e-6) {
        // Small angle approximation
        dexp_inv.block<3, 3>(3, 0) = -0.5 * VecToso3(v);
        return dexp_inv;
    }
    
    Matrix3d w_hat = VecToso3(w);
    Matrix3d v_hat = VecToso3(v);
    
    double theta2 = theta * theta;
    double theta3 = theta2 * theta;
    double theta4 = theta3 * theta;
    double theta5 = theta4 * theta;
    
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    
    // Rotation part (top-left 3x3)
    dexp_inv.block<3, 3>(0, 0) = Matrix3d::Identity() 
                                 - 0.5 * w_hat 
                                 + (1.0 / theta2 - (1.0 + cos_theta) / (2.0 * theta * sin_theta)) * w_hat * w_hat;
    
    // Translation part (bottom-left 3x3)
    Matrix3d A = 0.5 * v_hat 
                + (theta - sin_theta) / theta3 * (w_hat * v_hat + v_hat * w_hat + w_hat * v_hat * w_hat)
                - (1.0 - theta2 / 2.0 - cos_theta) / theta4 * (w_hat * w_hat * v_hat + v_hat * w_hat * w_hat - 3.0 * w_hat * v_hat * w_hat)
                - 0.5 * ((1.0 - theta2 / 2.0 - cos_theta) / theta4 - 3.0 * (theta - sin_theta - theta3 / 6.0) / theta5) 
                  * (w_hat * v_hat * w_hat * w_hat + w_hat * w_hat * v_hat * w_hat);
    
    dexp_inv.block<3, 3>(3, 0) = -A;
    
    return dexp_inv;
}


cv::Mat kron(const cv::Mat& A, const cv::Mat& B)
{
    // 1. 统一转换为 CV_64F (double) 以保证精度和通用性
    cv::Mat A64, B64;
    if (A.type() != CV_64F) A.convertTo(A64, CV_64F);
    else A64 = A;

    if (B.type() != CV_64F) B.convertTo(B64, CV_64F);
    else B64 = B;

    int rA = A64.rows;
    int cA = A64.cols;
    int rB = B64.rows;
    int cB = B64.cols;

    // 2. 初始化目标矩阵 C，尺寸为 (rA * rB) x (cA * cB)
    cv::Mat C = cv::Mat::zeros(rA * rB, cA * cB, CV_64F);

    // 3. 遍历矩阵 A 的每个元素
    for (int i = 0; i < rA; ++i) {
        for (int j = 0; j < cA; ++j) {
            // 获取 A 中的标量 a_ij
            double a_val = A64.at<double>(i, j);

            // 4. 定义目标矩阵 C 中的 ROI 区域
            // cv::Rect 的参数顺序是 (x, y, width, height) 对应 (列, 行, 宽, 高)
            cv::Rect roi(j * cB, i * rB, cB, rB);

            // 5. 计算分块 a_ij * B，并直接拷贝进 C 的 ROI 区域
            cv::Mat block = a_val * B64;
            block.copyTo(C(roi));
        }
    }

    return C;
}


// 辅助函数：Eigen矩阵转OpenCV矩阵
cv::Mat cvMatFromEigen(const Eigen::MatrixXd& eigenMat)
{
    cv::Mat cvMat(eigenMat.rows(), eigenMat.cols(), CV_64FC1);
    for(int i = 0; i < eigenMat.rows(); i++) {
        for(int j = 0; j < eigenMat.cols(); j++) {
            cvMat.at<double>(i, j) = eigenMat(i, j);
        }
    }
    return cvMat;
}

// 辅助函数：OpenCV矩阵转Eigen矩阵
Eigen::MatrixXd eigenFromCvMat(const cv::Mat& cvMat)
{
    Eigen::MatrixXd eigenMat(cvMat.rows, cvMat.cols);
    for(int i = 0; i < cvMat.rows; i++) {
        for(int j = 0; j < cvMat.cols; j++) {
            eigenMat(i, j) = cvMat.at<double>(i, j);
        }
    }
    return eigenMat;
}

Eigen::MatrixXd eigenFromCvMat(const cv::Vec3d& vect)
{
    Eigen::MatrixXd eigenMat(3, 1);
    eigenMat << vect[0], vect[1], vect[2];
    return eigenMat;
}

