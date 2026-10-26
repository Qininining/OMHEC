#pragma once
#include "../examples/simulation/dataset.hpp"
#include <fstream>
#include <iomanip>
#include <functional>

inline void regressionScenario(const std::function<void(double)>& emit) {
    const auto d=generateDataset();
    for(int method=0;method<3;++method) {
        MHEC::CalibrationCfg cfg; cfg.innovation_gate_enabled=false;
        MHEC::CalibrationData log; MHEC::CalibrationState state;
        state.all.KF.P=cv::Mat::eye(6,6,CV_64F)*.01;
        state.all.KF.Q=cv::Mat::eye(6,6,CV_64F)*1e-8;
        state.all.KF.R=cv::Mat::eye(3,3,CV_64F)*1e-6;
        cv::Mat T=d.initial.clone();
        for(size_t k=0;k<d.stageTwists.size();++k) {
            cv::Mat xi=cvMatFromEigen(Adjoint(eigenFromCvMat(T.inv()))*eigenFromCvMat(d.stageTwists[k]));
            log.all.Twist_c_list.push_back(xi);
            log.all.P_c_prev_list.push_back(d.points[k]);
            log.all.P_c_curr_list.push_back(d.points[k+1]);
            log.all.T_se_list.push_back(d.poses[k+1]);
            if(k+1>=6) {
                if(method==0) MHEC::updateYKronT(cfg,state,log,T);
                if(method==1) MHEC::updateServoOnce(cfg,state,log,T,.1/.02);
                if(method==2) MHEC::updateIEKF(cfg,state,log,T);
            }
            for(int i=0;i<16;++i) emit(T.at<double>(i));
            for(int i=0;i<36;++i) emit(state.all.KF.P.at<double>(i));
        }
    }
}
