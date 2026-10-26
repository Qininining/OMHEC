#include <mhec/calibration.hpp>
#include <stdexcept>
#include <cmath>

namespace {
void matrix(const cv::Mat& m, int rows, int cols) {
    if(m.type()!=CV_64F || m.rows!=rows || m.cols!=cols || !cv::checkRange(m))
        throw std::invalid_argument("Expected a finite CV_64F matrix of the documented size");
}
void transform(const cv::Mat& t) {
    matrix(t,4,4);
    const auto R=t(cv::Rect(0,0,3,3));
    const cv::Mat bottom=(cv::Mat_<double>(1,4)<<0,0,0,1);
    if(cv::norm(R.t()*R-cv::Mat::eye(3,3,CV_64F))>1e-8 ||
       cv::determinant(R)<0 || cv::norm(t.row(3)-bottom)>1e-10)
        throw std::invalid_argument("Expected an SE(3) transform");
}
void covariance(const cv::Mat& p,int n,bool definite) {
    matrix(p,n,n); cv::Mat eigenvalues;
    if(cv::norm(p-p.t())>1e-10) throw std::invalid_argument("Covariance must be symmetric");
    cv::eigen(p,eigenvalues);
    if(eigenvalues.at<double>(n-1)<(definite?1e-15:-1e-12))
        throw std::invalid_argument("Invalid covariance eigenvalues");
}
void append(mhec::State& s,const cv::Mat& xi,const cv::Vec3d& a,const cv::Vec3d& b) {
    matrix(xi,6,1);
    for(int i=0;i<3;++i) if(!std::isfinite(a[i]) || !std::isfinite(b[i]))
        throw std::invalid_argument("Expected finite camera points");
    s.observations.all.Twist_c_list.push_back(xi.clone());
    s.observations.all.P_c_prev_list.push_back(a);
    s.observations.all.P_c_curr_list.push_back(b);
    s.initialized=s.observations.all.P_c_curr_list.size()>=static_cast<size_t>(s.config.initialization_motions);
}
}
namespace mhec {
State initialize(const cv::Mat& t,const cv::Mat& p,const cv::Mat& q,const cv::Mat& r,const Config& config) {
    transform(t); covariance(p,6,false); covariance(q,6,false); covariance(r,3,true);
    if(config.initialization_motions<1 || config.dt<=0 || config.decay_gain<0 ||
       config.jfgd_svd_relative_threshold<0) throw std::invalid_argument("Invalid calibration configuration");
    State s; s.Tsc=t.clone(); s.config=config;
    s.filter.all.KF.P=p.clone(); s.filter.all.KF.Q=q.clone(); s.filter.all.KF.R=r.clone();
    return s;
}
void update_iekf(State& s,const cv::Mat& xi,const cv::Vec3d& a,const cv::Vec3d& b) {
    append(s,xi,a,b);
    if(s.initialized) MHEC::updateIEKF(s.config,s.filter,s.observations,s.Tsc);
}
void update_fgd(State& s,const cv::Mat& xi,const cv::Vec3d& a,const cv::Vec3d& b,double length) {
    if(!std::isfinite(length) || length<=0) throw std::invalid_argument("Characteristic length must be positive");
    append(s,xi,a,b);
    if(s.initialized) MHEC::updateServoOnce(s.config,s.filter,s.observations,s.Tsc,length);
}
cv::Mat solve_position_only_ls(const std::vector<cv::Mat>& poses,const std::vector<cv::Vec3d>& points) {
    if(poses.size()!=points.size() || poses.size()<6)
        throw std::invalid_argument("PO-LS requires matching poses/points and at least six observations");
    cv::Mat A=cv::Mat::zeros(static_cast<int>(poses.size())*3,15,CV_64F);
    for(size_t i=0;i<poses.size();++i) {
        transform(poses[i]);
        for(double value:points[i].val) if(!std::isfinite(value)) throw std::invalid_argument("Nonfinite point");
        cv::Mat p(points[i],true),block,minus=-poses[i](cv::Rect(0,0,3,3));
        cv::hconcat(std::vector<cv::Mat>{kron(p.t(),cv::Mat::eye(3,3,CV_64F)),cv::Mat::eye(3,3,CV_64F),minus},block);
        block.copyTo(A.rowRange(static_cast<int>(i)*3,static_cast<int>(i)*3+3));
    }
    cv::SVD svd(A,cv::SVD::NO_UV);
    if(svd.w.at<double>(14)<=svd.w.at<double>(0)*1e-10)
        throw std::invalid_argument("PO-LS system is rank deficient");
    MHEC::CalibrationData data; data.all.T_se_list=poses; data.all.P_c_curr_list=points;
    MHEC::CalibrationState state; Config config;
    cv::Mat result=cv::Mat::eye(4,4,CV_64F);
    MHEC::updateYKronT(config,state,data,result); return result;
}
void residual_model(const cv::Vec3d& a,const cv::Vec3d& b,const cv::Mat& xi,cv::Mat& e,cv::Mat& H) {
    matrix(xi,6,1);
    const cv::Mat dT=cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(xi))));
    const cv::Mat ah=(cv::Mat_<double>(4,1)<<a[0],a[1],a[2],1.0);
    const cv::Mat pred=dT*ah;
    e=(cv::Mat_<double>(3,1)<<b[0]-pred.at<double>(0),b[1]-pred.at<double>(1),b[2]-pred.at<double>(2));
    const cv::Mat px=cvMatFromEigen(VecToso3(eigenFromCvMat(cv::Mat(a))));
    const cv::Mat wx=cvMatFromEigen(VecToso3(eigenFromCvMat(xi.rowRange(0,3))));
    const cv::Mat vx=cvMatFromEigen(VecToso3(eigenFromCvMat(xi.rowRange(3,6))));
    cv::Mat left=px*wx-vx,right=-wx; cv::hconcat(left,right,H);
}
}
