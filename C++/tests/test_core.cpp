#include <mhec/calibration.hpp>
#include "regression_scenario.hpp"
#include <stdexcept>
#include <iostream>

void require(bool pass,const char* message) { if(!pass) throw std::runtime_error(message); }
int main(int argc,char** argv) {
    try {
        require(argc==2,"Expected pre-extraction fixture");
        std::ifstream expected(argv[1]); require(bool(expected),"Missing fixture");
        regressionScenario([&](double actual) {
            double value; require(bool(expected>>value),"Truncated fixture");
            require(std::isfinite(actual) && std::abs(value-actual)<1e-9,"Pre-extraction trajectory or covariance changed");
        });
        double extra; require(!(expected>>extra),"Extra fixture values");
        // Public streaming adapters must preserve the extracted kernels.
        const auto data=generateDataset();
        for(int method=1;method<=2;++method) {
            mhec::Config cfg;cfg.innovation_gate_enabled=false;
            cv::Mat p=cv::Mat::eye(6,6,CV_64F)*.01;
            cv::Mat q=cv::Mat::eye(6,6,CV_64F)*1e-8;
            cv::Mat noise=cv::Mat::eye(3,3,CV_64F)*1e-6;
            auto stream=mhec::initialize(data.initial,p,q,noise,cfg);
            MHEC::CalibrationData log; MHEC::CalibrationState raw;
            raw.all.KF.P=p.clone(); raw.all.KF.Q=q.clone(); raw.all.KF.R=noise.clone();
            cv::Mat estimate=data.initial.clone();
            for(size_t k=0;k<data.stageTwists.size();++k) {
                const cv::Mat xi=cvMatFromEigen(Adjoint(eigenFromCvMat(estimate.inv()))*eigenFromCvMat(data.stageTwists[k]));
                log.all.Twist_c_list.push_back(xi);
                log.all.P_c_prev_list.push_back(data.points[k]);
                log.all.P_c_curr_list.push_back(data.points[k+1]);
                if(method==1) {
                    mhec::update_fgd(stream,xi,data.points[k],data.points[k+1],5.);
                    if(k>=5) MHEC::updateServoOnce(cfg,raw,log,estimate,5.);
                } else {
                    mhec::update_iekf(stream,xi,data.points[k],data.points[k+1]);
                    if(k>=5) MHEC::updateIEKF(cfg,raw,log,estimate);
                }
                require(cv::norm(stream.Tsc-estimate)<1e-12,"Streaming estimate changed");
                require(cv::norm(stream.filter.all.KF.P-raw.all.KF.P)<1e-12,"Streaming covariance changed");
            }
        }
        Eigen::VectorXd x(6); x<<.1,.2,-.1,.5,-.2,.1;
        const auto T=MatrixExp6(VecTose3(x));
        require((T*TransInv(T)-Eigen::Matrix4d::Identity()).norm()<1e-12,"SE(3) inverse");
        const cv::Mat eye=cv::Mat::eye(6,6,CV_64F),zero=cv::Mat::zeros(6,6,CV_64F),r=cv::Mat::eye(3,3,CV_64F);
        auto s=mhec::initialize(cvMatFromEigen(T),eye,zero,r);
        const cv::Mat initial=s.Tsc.clone();
        const cv::Mat xi=(cv::Mat_<double>(6,1)<<.01,.02,.03,.1,.2,.3);
        for(int k=0;k<5;++k) mhec::update_iekf(s,xi,{1,2,3},{1,2,3});
        require(!s.initialized && cv::norm(s.Tsc-initial)==0,"Initialization warmup");
        cv::Mat e,H; mhec::residual_model({1,2,3},{1,2,3},xi,e,H);
        const Eigen::Vector3d point(1,2,3);
        const Eigen::VectorXd body=eigenFromCvMat(xi);
        const Eigen::Vector3d base=VecToso3(body.head<3>())*point+body.tail<3>();
        Eigen::Matrix<double,3,6> numeric;
        for(int j=0;j<6;++j) {
            Eigen::VectorXd dx=Eigen::VectorXd::Zero(6);dx(j)=1e-4;
            Eigen::VectorXd moved=Adjoint(MatrixExp6(VecTose3(dx)))*body;
            Eigen::VectorXd opposite=Adjoint(MatrixExp6(VecTose3(-dx)))*body;
            numeric.col(j)=(VecToso3(moved.head<3>())*point+moved.tail<3>()
                -VecToso3(opposite.head<3>())*point-opposite.tail<3>())/2e-4;
        }
        require((eigenFromCvMat(H)-numeric).norm()<1e-6,"Residual Jacobian");
        bool rejected=false;
        try {mhec::solve_position_only_ls(std::vector<cv::Mat>(6,cv::Mat::eye(4,4,CV_64F)),std::vector<cv::Vec3d>(6));}
        catch(const std::invalid_argument&) {rejected=true;}
        require(rejected,"Degenerate PO-LS must be rejected");
        std::cout<<"PASS: frozen numerical baseline, SE(3), Jacobian, initialization and degeneracy\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
