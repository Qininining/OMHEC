#ifndef MHECTASK_H
#define MHECTASK_H

#include <QObject>
#include <QStateMachine>

#include <string>
#include <vector>

#include "AgeMotionActor.h"
#include "BaslerCameraControl.h"
#include "ParallelPlatform.h"

#include "MHEC.h"

class MHECTask:  public QObject
{
    Q_OBJECT
public:
    MHECTask();
    ~MHECTask();

    void setup(ParallelPlatform* robot, AgeMotionActor* motionActor, BaslerCameraControl* cameraControl);
    void initWorkflow();
    void init10XWorkflow();

    void start();
    void stop();

public: signals:
    void MHECTaskFinished(bool success = true);
    void sigTargetCircle(cv::Vec3d Circle);

private slots:
    void onFinished();
    void stepFinished(bool success, const QString &errorMsg = QString());

private:
    void getPoint(cv::Vec3d& Point, double scale);
    void gotoPoint(cv::Vec3d Point, cv::Mat twist_c);

    void getScale(double& scale, cv::Mat& T_sc);
    void getInit(cv::Mat& T_sc, double scale);


    void runOLS(const MHEC::CalibrationCfg& cfg,
                 cv::Mat& T_sc_est, double scale);

    void runIEKF(const MHEC::CalibrationCfg& cfg,
                 MHEC::CalibrationState& sta,
                 cv::Mat& T_sc_est, double scale);

    void runIEKF_RT(const MHEC::CalibrationCfg& cfg,
                    MHEC::CalibrationState& sta,
                    cv::Mat& T_sc_est, double scale);

    void runEKF(const MHEC::CalibrationCfg& cfg,
                 MHEC::CalibrationState& sta,
                 cv::Mat& T_sc_est, double scale);

    void runRLS(const MHEC::CalibrationCfg& cfg,
                MHEC::CalibrationState& sta,
                cv::Mat& T_sc_est, double scale);

    void runServoOnce(const MHEC::CalibrationCfg& cfg,
                MHEC::CalibrationState& sta,
                cv::Mat& T_sc_est, double scale);

    void runServoTwoStage(const MHEC::CalibrationCfg& cfg,
                      MHEC::CalibrationState& sta,
                      cv::Mat& T_sc_est, double scale);

    void runYKronT(const MHEC::CalibrationCfg& cfg,
                   MHEC::CalibrationState& sta,
                   cv::Mat& T_sc_est, double scale);

    void runNoiseCal(const MHEC::CalibrationCfg& cfg,
                     MHEC::CalibrationState& sta,
                     cv::Mat& T_sc_est, double scale);

    void runDataAquire(const MHEC::CalibrationCfg& cfg,
                     cv::Mat& T_sc_est, double scale);

    double runTest(const std::string& name, const MHEC::CalibrationCfg& cfg,
                   cv::Mat& T_sc_est, double scale,
                   const cv::Mat& pointCovariance = cv::Mat());

    void runSharedValidation10X(const MHEC::CalibrationCfg& cfg,
                               const std::vector<cv::Mat>& candidateTsc,
                               const std::vector<std::string>& methodNames,
                               double scale,
                               const cv::Mat& pointCovariance,
                               const QString& roundTag);

    void runTransTest(cv::Mat& T_sc_est, double scale, double step);
    void runRotTest(cv::Mat& T_sc_est, double scale, double angle_deg);

    void getTargetPoint(cv::Vec3d& P_c_pred, cv::Vec3d P_c_prev, cv::Mat twist_c, double scale);
    cv::Mat runStaticNoise10X(int samples, double scale);
    void run10XExperiment();


    cv::Vec3d printPoseInformation(const cv::Mat& T);

private:
    QStateMachine* staMachine_;
    AgeMotionActor* motionActor_ = nullptr;
    BaslerCameraControl* cameraControl_ = nullptr;
    ParallelPlatform* robot_ = nullptr;

    int imgWidth_ = 0;
    int imgHigh_ = 0;
    double scale_ = 0;
    int t_testStep = 200000;
    int R_testStep = 5;
    cv::Mat T_cs_est_ = cv::Mat::eye(4, 4, CV_64F);
    QString runTag_;
};


#endif // MHECTASK_H
