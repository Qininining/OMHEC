#ifndef MODERN_ROBOTIC_LIB_H
#define MODERN_ROBOTIC_LIB_H

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace Eigen;

    MatrixXd Adjoint(Matrix4d T);
    Matrix4d VecTose3(VectorXd V);
    Matrix4d MatrixExp6(Matrix4d se3mat);
    Matrix3d VecToso3(Vector3d omg);
    void RpToTrans(Matrix3d R, Vector3d p, Matrix4d& T);
    void TransToRp(Matrix4d T, Matrix3d& R, Vector3d& p);
    Vector3d so3ToVec(Matrix3d  omg);
    bool NearZear(double z);
    void AxisAng3(Vector3d expc3, Vector3d& unitV, double& theta);
    Vector3d  Normalize(Vector3d  V);
    Matrix3d MatrixExp3(Matrix3d so3mat);
    MatrixXd SlistToBlist(MatrixXd Slist, Matrix4d M);
    Matrix4d TransInv(Matrix4d T);
    MatrixXd JacobianBody(MatrixXd Blist, VectorXd thetalist);
    Matrix4d FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist);
    Matrix4d FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist);
    Matrix4d MatrixLog6(Matrix4d T);
    VectorXd se3ToVec(Matrix4d se3mat);
    MatrixXd ad(VectorXd V);
    MatrixXd get_dexp_inv(VectorXd S);

    cv::Mat kron(const cv::Mat& A, const cv::Mat& B);

    cv::Mat cvMatFromEigen(const Eigen::MatrixXd& eigenMat);
    Eigen::MatrixXd eigenFromCvMat(const cv::Mat& cvMat);
    Eigen::MatrixXd eigenFromCvMat(const cv::Vec3d& vect);


#endif // MODERN_ROBOTIC_LIB_H
