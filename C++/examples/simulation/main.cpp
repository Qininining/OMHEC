#include <omhec/calibration.hpp>
#include "dataset.hpp"
#include <iostream>
#include <algorithm>
int main() {
    const auto data=generateDataset();
    omhec::Config cfg; cfg.innovation_gate_enabled=false;
    const cv::Mat P=cv::Mat::eye(6,6,CV_64F)*.01,Q=cv::Mat::zeros(6,6,CV_64F);
    const cv::Mat R=(cv::Mat_<double>(3,3)<<2e-8,0,0,0,2e-8,0,0,0,2e-6);
    for(int method=0;method<3;++method) {
        auto state=omhec::initialize(data.initial,P,Q,R,cfg);
        for(size_t k=0;k<data.stageTwists.size();++k) {
            if(method==0 && k+1>=6) {
                std::vector<cv::Mat> poses(data.poses.begin(),data.poses.begin()+k+2);
                std::vector<cv::Vec3d> points(data.points.begin(),data.points.begin()+k+2);
                state.Tsc=omhec::solve_position_only_ls(poses,points);
            } else if(method>0) {
                cv::Mat xi=cvMatFromEigen(Adjoint(eigenFromCvMat(state.Tsc.inv()))*eigenFromCvMat(data.stageTwists[k]));
                if(method==1) omhec::update_fgd(state,xi,data.points[k],data.points[k+1],.1/.02);
                else omhec::update_iekf(state,xi,data.points[k],data.points[k+1]);
            }
        }
        const cv::Mat r=state.Tsc(cv::Rect(0,0,3,3))*data.truth(cv::Rect(0,0,3,3)).t();
        double cosine=std::clamp((cv::trace(r)[0]-1)/2.,-1.,1.);
        std::cout << (method==0?"PO-LS":method==1?"PR-FGD":"PR-IEKF")
                  << ": rotation " << std::acos(cosine)*180/3.141592653589793
                  << " deg; translation " << cv::norm(state.Tsc(cv::Rect(3,0,1,3))-data.truth(cv::Rect(3,0,1,3))) << " mm\n"
                  << state.Tsc << '\n';
    }
}
