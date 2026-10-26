#include "MHECTask.h"

#include <QFinalState>
#include <QDateTime>
#include <QRandomGenerator>

#include <algorithm>
#include <cmath>

#include "AsyncTaskBridge.hpp"
#include "modern_robotic_lib.h"

namespace {

bool isValidRigidTransform(const cv::Mat& transform)
{
    if (transform.empty() || transform.rows != 4 || transform.cols != 4 ||
        transform.type() != CV_64F || !cv::checkRange(transform)) {
        return false;
    }
    const cv::Mat rotation = transform(cv::Rect(0, 0, 3, 3));
    const cv::Mat orthogonalityError =
        rotation.t() * rotation - cv::Mat::eye(3, 3, CV_64F);
    return std::abs(cv::determinant(rotation) - 1.0) <= 1e-3 &&
           cv::norm(orthogonalityError, cv::NORM_INF) <= 1e-3 &&
           std::abs(transform.at<double>(3, 0)) <= 1e-9 &&
           std::abs(transform.at<double>(3, 1)) <= 1e-9 &&
           std::abs(transform.at<double>(3, 2)) <= 1e-9 &&
           std::abs(transform.at<double>(3, 3) - 1.0) <= 1e-9;
}

}

MHECTask::MHECTask()
{
    this->staMachine_ =  new QStateMachine();
    connect(this->staMachine_, &QStateMachine::finished, this, &MHECTask::onFinished);
    PASS_("MHECTask created");
}
MHECTask::~MHECTask()
{
    if(this->staMachine_ && this->staMachine_->isRunning()) {
        this->staMachine_->stop();
    }
    delete this->staMachine_;
}

void MHECTask::setup(ParallelPlatform* robot, AgeMotionActor* motionActor, BaslerCameraControl* cameraControl)
{
    if (!staMachine_) {
        ERROR_("State machine not initialized");
        throw MAR::RobotException(MAR::MARInternalError, "State machine not initialized");
    }
    if (!robot) {
        ERROR_("ParallelPlatform not set up");
        throw MAR::RobotException(MAR::MARRMDMotorNotInitialized, "ParallelPlatform not set up");
    }
    if (!motionActor) {
        ERROR_("AgeMotionActor not set up");
        throw MAR::RobotException(MAR::MARInternalError, "AgeMotionActor not set up");
    }
    if (!cameraControl) {
        ERROR_("BaslerCameraControl not set up");
        throw MAR::RobotException(MAR::MARInternalError, "BaslerCameraControl not set up");
    }
    this->robot_ = robot;
    this->motionActor_ = motionActor;
    this->cameraControl_ = cameraControl;

    init10XWorkflow();
}

void MHECTask::init10XWorkflow()
{
    // 10X experiment uses a dedicated single-state workflow and exits on completion.
    // 不再进入原工程中循环执行的历史实验状态。
    QState* experiment = new QState(this->staMachine_);
    QFinalState* finished = new QFinalState(this->staMachine_);
    AsyncTaskBridge* bridge = new AsyncTaskBridge(this);
    bridge->bindState(experiment, [this]() { run10XExperiment(); });
    experiment->addTransition(bridge, &AsyncTaskBridge::taskFinished, finished);
    connect(bridge, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    this->staMachine_->setInitialState(experiment);
}

void MHECTask::initWorkflow()
{
    if (!this->staMachine_) {
        ERROR_("State machine not initialized");
        throw MAR::RobotException(MAR::MARInternalError, "State machine not initialized");
    }

    // 定义状态
    QState* s1 = new QState(this->staMachine_);
    QState* s2 = new QState(this->staMachine_);
    QState* s3 = new QState(this->staMachine_);
    QState* s4 = new QState(this->staMachine_);
    QState* s5 = new QState(this->staMachine_);
    QState* s6 = new QState(this->staMachine_);
    QState* s7 = new QState(this->staMachine_);
    QState* s8 = new QState(this->staMachine_);
    QState* s9 = new QState(this->staMachine_);
    QState* s10 = new QState(this->staMachine_);
    QFinalState* stateFinal = new QFinalState(this->staMachine_);

    // 创建 AsyncTaskBridge 实例
    AsyncTaskBridge* bridge1 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge2 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge3 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge4 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge5 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge6 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge7 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge8 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge9 = new AsyncTaskBridge(this);
    AsyncTaskBridge* bridge10 = new AsyncTaskBridge(this);

    // ##################################################################
    // #################   初始化＋热启动   ###########################
    // ##################################################################
    bridge1->bindState(s1, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 1 at thread ID: %d", QThread::currentThreadId());
        // cv::Vec3d Point = cv::Vec3d(-1,-1,-1);
        // this->motionActor_->getPoint(Point);
        // qInfo() << QString("Point: (%1, %2, %3)").arg(Point[0]).arg(Point[1]).arg(Point[2]);

        // scale_ = 113.807;

        // cv::Mat T_cs_Wuliao = (cv::Mat_<double>(4, 4) <<
        //                            0.13300810,		0.99111068,		0.00290949,		619196.24447747,
        //                        0.99111415,		-0.13300369,		-0.00165923,		-65956.65058821,
        //                        -0.00125750,		0.00310433,		-0.99999439,		15472562.55769724,
        //                        0.00000000,		0.00000000,		0.00000000,		1.00000000);
        // cv::Mat R_cs_Wuliao = T_cs_Wuliao(cv::Rect(0, 0, 3, 3));
        // cv::Mat U_Wuliao, S_Wuliao, Vt_Wuliao;
        // cv::SVD::compute(R_cs_Wuliao, S_Wuliao, U_Wuliao, Vt_Wuliao);
        // R_cs_Wuliao = U_Wuliao * Vt_Wuliao;
        // R_cs_Wuliao.copyTo(T_cs_Wuliao(cv::Rect(0, 0, 3, 3)));

        // this->printPoseInformation(T_cs_Wuliao);

        // MHEC::CalibrationCfg test_cfg_Wuliao;
        // test_cfg_Wuliao.max_iter = 12;
        // runTest("Wuliao1_1", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao2", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao3", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao4", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao5", test_cfg_Wuliao, T_cs_Wuliao, scale_);


        // T_cs_Wuliao = (cv::Mat_<double>(4, 4) <<
        //                    0.12989846,		0.99152728,		-0.00022000,		762143.10351224,
        //                0.99152703,		-0.12989858,		-0.00070983,		286681.86861278,
        //                -0.00073239,		-0.00012593,		-0.99999972,		15507374.93441634,
        //                0.00000000,		0.00000000,		0.00000000,		1.00000000);
        // R_cs_Wuliao = T_cs_Wuliao(cv::Rect(0, 0, 3, 3));
        // cv::SVD::compute(R_cs_Wuliao, S_Wuliao, U_Wuliao, Vt_Wuliao);
        // R_cs_Wuliao = U_Wuliao * Vt_Wuliao;
        // R_cs_Wuliao.copyTo(T_cs_Wuliao(cv::Rect(0, 0, 3, 3)));

        // this->printPoseInformation(T_cs_Wuliao);

        // test_cfg_Wuliao.max_iter = 12;
        // runTest("Wuliao1_2", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao2", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao3", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao4", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao5", test_cfg_Wuliao, T_cs_Wuliao, scale_);



        // T_cs_Wuliao = (cv::Mat_<double>(4, 4) <<
        //                    0.13184932,		0.99126828,		0.00171604,		624661.88646120,
        //                0.99123382,		-0.13185908,		0.00828800,		-129600.31716208,
        //                0.00844191,		0.00060823,		-0.99996418,		15484964.05209081,
        //                0.00000000,		0.00000000,		0.00000000,		1.00000000);
        // R_cs_Wuliao = T_cs_Wuliao(cv::Rect(0, 0, 3, 3));
        // cv::SVD::compute(R_cs_Wuliao, S_Wuliao, U_Wuliao, Vt_Wuliao);
        // R_cs_Wuliao = U_Wuliao * Vt_Wuliao;
        // R_cs_Wuliao.copyTo(T_cs_Wuliao(cv::Rect(0, 0, 3, 3)));

        // this->printPoseInformation(T_cs_Wuliao);

        // test_cfg_Wuliao.max_iter = 12;
        // runTest("Wuliao1_3", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao2", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao3", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao4", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao5", test_cfg_Wuliao, T_cs_Wuliao, scale_);




        // T_cs_Wuliao = (cv::Mat_<double>(4, 4) <<
        //                    0.13173807,		0.99128328,		0.00159450,		617113.02871806,
        //                0.99127292,		-0.13174415,		0.00463460,		-74124.36723844,
        //                0.00480427,		0.00097003,		-0.99998799,		15473494.40596893,
        //                0.00000000,		0.00000000,		0.00000000,		1.00000000);
        // R_cs_Wuliao = T_cs_Wuliao(cv::Rect(0, 0, 3, 3));
        // cv::SVD::compute(R_cs_Wuliao, S_Wuliao, U_Wuliao, Vt_Wuliao);
        // R_cs_Wuliao = U_Wuliao * Vt_Wuliao;
        // R_cs_Wuliao.copyTo(T_cs_Wuliao(cv::Rect(0, 0, 3, 3)));

        // this->printPoseInformation(T_cs_Wuliao);

        // test_cfg_Wuliao.max_iter = 12;
        // runTest("Wuliao1_4", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao2", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao3", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao4", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao5", test_cfg_Wuliao, T_cs_Wuliao, scale_);






        // T_cs_Wuliao = (cv::Mat_<double>(4, 4) <<
        //                    0.13294166,		0.99112380,		0.00035764,		658581.12980487,
        //                0.99112122,		-0.13294046,		-0.00235630,		56215.53180153,
        //                -0.00228784,		0.00066771,		-0.99999716,		15479447.67702322,
        //                0.00000000,		0.00000000,		0.00000000,		1.00000000);
        // R_cs_Wuliao = T_cs_Wuliao(cv::Rect(0, 0, 3, 3));
        // cv::SVD::compute(R_cs_Wuliao, S_Wuliao, U_Wuliao, Vt_Wuliao);
        // R_cs_Wuliao = U_Wuliao * Vt_Wuliao;
        // R_cs_Wuliao.copyTo(T_cs_Wuliao(cv::Rect(0, 0, 3, 3)));

        // this->printPoseInformation(T_cs_Wuliao);

        // test_cfg_Wuliao.max_iter = 12;
        // runTest("Wuliao1_5", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao2", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao3", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao4", test_cfg_Wuliao, T_cs_Wuliao, scale_);
        // runTest("Wuliao5", test_cfg_Wuliao, T_cs_Wuliao, scale_);



        // 初始化
        cv::Mat T_cs_est = (cv::Mat_<double>(4, 4) <<
                            -0.03369230,		0.99943162,			-0.00112467,		1108699.52134736,
                            0.99942942,			0.03368951,			-0.00241781,		-3011437.36013615,
                            -0.00237855,		-0.00120549,		-0.99999644,		21656419.08868462,
                            0.00000000,			0.00000000,			0.00000000,			1.00000000);

        // 正交化初值
        cv::Mat R_cs_est = T_cs_est(cv::Rect(0, 0, 3, 3));
        cv::Mat U, S, Vt;
        cv::SVD::compute(R_cs_est, S, U, Vt);
        R_cs_est = U * Vt;
        R_cs_est.copyTo(T_cs_est(cv::Rect(0, 0, 3, 3)));

        scale_ = 230.207;
        T_cs_est_ = T_cs_est;

        // // Scale 标定
        // double scale = 0;
        // cv::Mat T_sc_init = cv::Mat::eye(4, 4, CV_64F);
        // this->getScale(scale, T_sc_init);
        // scale_ = scale;
        // qDebug() << "scale_ Details:" << scale_;
        // T_cs_est = T_sc_init.inv();
        // T_cs_est_ = T_cs_est;
        // this->printPoseInformation(T_cs_est);


        // // // 热启动
        cv::Mat T_sc_est = T_cs_est.inv();
        this->getInit(T_sc_est, scale_);
        T_cs_est = T_sc_est.inv();
        T_cs_est_ = T_cs_est;
        this->printPoseInformation(T_cs_est);

        MHEC::CalibrationCfg test_cfg;
        test_cfg.max_iter = 12;
        runTest("Init1", test_cfg, T_sc_est, scale_);
        runTest("Init2", test_cfg, T_sc_est, scale_);
        runTest("Init3", test_cfg, T_sc_est, scale_);
        runTest("Init4", test_cfg, T_sc_est, scale_);
        runTest("Init5", test_cfg, T_sc_est, scale_);

    });

    // ##################################################################
    // #################   方法1   ###########################
    // ##################################################################

    bridge2->bindState(s2, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 2 at thread ID: %d", QThread::currentThreadId());

        // // IEKF
        MHEC::CalibrationCfg cfg;
        MHEC::CalibrationState sta_iekf;
        // // cfg.max_iter = 300;

        double sigma_Tse = 0.03;
        double sigma_rot_step = sigma_Tse * cfg.step_len_R;
        double sigma_t_step   = sigma_Tse * cfg.step_len_t;

        sta_iekf.all.KF.R = (cv::Mat_<double>(3, 3) <<
                                 4e+6, 0,          0,           // X轴方差
                             0,          4e+6, 0,           // Y轴方差
                             0,          0,          1e+8  // Z轴方差 (噪声最大)
                             );
        sta_iekf.all.KF.Q = cv::Mat::eye(6, 6, CV_64F) * 1e+0;
        sta_iekf.all.KF.Q(cv::Range(0, 3), cv::Range(0, 3)) = cv::Mat::eye(3, 3, CV_64F) * (sigma_rot_step * sigma_rot_step);
        sta_iekf.all.KF.Q(cv::Range(3, 6), cv::Range(3, 6)) = cv::Mat::eye(3, 3, CV_64F) * (sigma_t_step * sigma_t_step);

        sta_iekf.all.KF.P = cv::Mat::eye(6, 6, CV_64F) * 1e+0;
        sta_iekf.all.KF.P(cv::Range(0, 3), cv::Range(0, 3)) = cv::Mat::eye(3, 3, CV_64F) * ((5.0 / 180.0 * M_PI) * (5.0 / 180.0 * M_PI));
        sta_iekf.all.KF.P(cv::Range(3, 6), cv::Range(3, 6)) = cv::Mat::eye(3, 3, CV_64F) * ((1e6) * (1e6));

        cv::Mat T_sc_est = T_cs_est_.inv();
        HIGHLIGHT_("[MHECTask] Starting Step 3 at thread ID: %d", QThread::currentThreadId());
        this->runIEKF(cfg, sta_iekf, T_sc_est, scale_);
        cv::Mat T_cs_est = T_sc_est.inv();
        this->printPoseInformation(T_cs_est);

        MHEC::CalibrationCfg test_cfg;
        test_cfg.max_iter = 12;
        runTest("IEKF1", test_cfg, T_sc_est, scale_);
        runTest("IEKF2", test_cfg, T_sc_est, scale_);
        runTest("IEKF3", test_cfg, T_sc_est, scale_);
        runTest("IEKF4", test_cfg, T_sc_est, scale_);
        runTest("IEKF5", test_cfg, T_sc_est, scale_);

    });

    // ##################################################################
    // #################      方法2       ###########################
    // ##################################################################

    bridge3->bindState(s3, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 3 at thread ID: %d", QThread::currentThreadId());

        // runServoOnce
        MHEC::CalibrationCfg cfg;
        MHEC::CalibrationState sta_servo_Once;

        sta_servo_Once.all.Lambda = 0.1;

        cv::Mat T_sc_est = T_cs_est_.inv();
        this->runServoOnce(cfg, sta_servo_Once, T_sc_est, scale_);
        cv::Mat T_cs_est = T_sc_est.inv();
        this->printPoseInformation(T_cs_est);

        MHEC::CalibrationCfg test_cfg;
        test_cfg.max_iter = 12;
        runTest("ServoOnce1", test_cfg, T_sc_est, scale_);
        runTest("ServoOnce2", test_cfg, T_sc_est, scale_);
        runTest("ServoOnce3", test_cfg, T_sc_est, scale_);
        runTest("ServoOnce4", test_cfg, T_sc_est, scale_);
        runTest("ServoOnce5", test_cfg, T_sc_est, scale_);
    });

    // ##################################################################
    // #################       方法3       ###########################
    // ##################################################################

    bridge4->bindState(s4, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 4 at thread ID: %d", QThread::currentThreadId());

        // // InEKF_RT
        MHEC::CalibrationCfg cfg;
        MHEC::CalibrationState sta_iekf_RT;

        double sigma_Tse = 0.03;
        double sigma_rot_step = sigma_Tse * cfg.step_len_R;
        double sigma_t_step   = sigma_Tse * cfg.step_len_t;

        sta_iekf_RT.rot.KF.R = (cv::Mat_<double>(3, 3) <<
                                    4e+6, 0,          0,           // X轴方差
                                0,          4e+6, 0,           // Y轴方差
                                0,          0,          1e+8  // Z轴方差 (噪声最大)
                                );
        sta_iekf_RT.rot.KF.Q = cv::Mat::eye(3, 3, CV_64F) * (sigma_rot_step * sigma_rot_step);
        sta_iekf_RT.rot.KF.P = cv::Mat::eye(3, 3, CV_64F) * ((5.0 / 180.0 * M_PI) * (5.0 / 180.0 * M_PI));

        sta_iekf_RT.t.KF.R = sta_iekf_RT.rot.KF.R;
        sta_iekf_RT.t.KF.Q = cv::Mat::eye(3, 3, CV_64F) * (sigma_t_step * sigma_t_step);
        sta_iekf_RT.t.KF.P = cv::Mat::eye(3, 3, CV_64F) * ((1e6) * (1e6));

        cv::Mat T_sc_est = T_cs_est_.inv();
        HIGHLIGHT_("[MHECTask] Starting Step 3 at thread ID: %d", QThread::currentThreadId());
        this->runIEKF_RT(cfg, sta_iekf_RT, T_sc_est, scale_);
        cv::Mat T_cs_est = T_sc_est.inv();
        this->printPoseInformation(T_cs_est);

        MHEC::CalibrationCfg test_cfg;
        test_cfg.max_iter = 12;
        runTest("InEKF_RT1", test_cfg, T_sc_est, scale_);
        runTest("InEKF_RT2", test_cfg, T_sc_est, scale_);
        runTest("InEKF_RT3", test_cfg, T_sc_est, scale_);
        runTest("InEKF_RT4", test_cfg, T_sc_est, scale_);
        runTest("InEKF_RT5", test_cfg, T_sc_est, scale_);

    });

    // ##################################################################
    // #################       方法4       ###########################
    // ##################################################################

    bridge5->bindState(s5, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 5 at thread ID: %d", QThread::currentThreadId());

        // runServoTwoStage
        MHEC::CalibrationCfg cfg;
        MHEC::CalibrationState sta_servo_TwoStage;

        sta_servo_TwoStage.rot.Lambda = 0.1;
        sta_servo_TwoStage.t.Lambda = 0.1;

        cv::Mat T_sc_est = T_cs_est_.inv();
        this->runServoTwoStage(cfg, sta_servo_TwoStage, T_sc_est, scale_);
        cv::Mat T_cs_est = T_sc_est.inv();
        this->printPoseInformation(T_cs_est);

        MHEC::CalibrationCfg test_cfg;
        test_cfg.max_iter = 12;
        runTest("ServoTwoStage1", test_cfg, T_sc_est, scale_);
        runTest("ServoTwoStage2", test_cfg, T_sc_est, scale_);
        runTest("ServoTwoStage3", test_cfg, T_sc_est, scale_);
        runTest("ServoTwoStage4", test_cfg, T_sc_est, scale_);
        runTest("ServoTwoStage5", test_cfg, T_sc_est, scale_);

    });

    bridge6->bindState(s6, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 6 at thread ID: %d", QThread::currentThreadId());

        // DataAquire
        MHEC::CalibrationCfg cfg;
        cv::Mat T_sc_est = T_cs_est_.inv();
        this->runDataAquire(cfg, T_sc_est, scale_);
        cv::Mat T_cs_est = T_sc_est.inv();
        this->printPoseInformation(T_cs_est);

    });

    bridge7->bindState(s7, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 7 at thread ID: %d", QThread::currentThreadId());

        // YKronT_RLS
        // MHEC::CalibrationCfg cfg;
        // MHEC::CalibrationState sta_ykront;

        // sta_ykront.all.KF.P = cv::Mat::eye(15, 15, CV_64F) * 1e+6;

        // cv::Mat T_sc_est = T_cs_est_.inv();
        // this->runYKronT(cfg, sta_ykront, T_sc_est, scale_);
        // cv::Mat T_cs_est = T_sc_est.inv();
        // this->printPoseInformation(T_cs_est);


    });

    bridge8->bindState(s8, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 8 at thread ID: %d", QThread::currentThreadId());
        // runNoiseCal
        // MHEC::CalibrationCfg cfg;
        // MHEC::CalibrationState sta_Noise;

        // cv::Mat T_sc_est = T_cs_est_.inv();
        // this->runNoiseCal(cfg, sta_Noise, T_sc_est, scale_);
        // cv::Mat T_cs_est = T_sc_est.inv();
        // this->printPoseInformation(T_cs_est);

    });

    bridge9->bindState(s9, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 9 at thread ID: %d", QThread::currentThreadId());
        //  OLS
        // MHEC::CalibrationCfg cfg;

        // cv::Mat T_sc_est = T_cs_est_.inv();
        // this->runOLS(cfg, T_sc_est, scale_);
        // cv::Mat T_cs_est = T_sc_est.inv();
        // this->printPoseInformation(T_cs_est);

        // MHEC::CalibrationCfg test_cfg;
        // test_cfg.max_iter = 12;
        // runTest("InEKF_RT", test_cfg, T_sc_est, scale_);
    });

    bridge10->bindState(s10, [this]() {
        HIGHLIGHT_("[MHECTask] Starting Step 10 at thread ID: %d", QThread::currentThreadId());
        // //  RLS
        // MHEC::CalibrationCfg cfg;
        // MHEC::CalibrationState sta_rls;

        // sta_rls.rot.KF.P = cv::Mat::eye(3, 3, CV_64F) * 1e+1;
        // sta_rls.t.KF.P = cv::Mat::eye(3, 3, CV_64F) * 1e+1;

        // cv::Mat T_sc_est = T_cs_est_.inv();
        // this->runRLS(cfg, sta_rls, T_sc_est, scale_);
        // cv::Mat T_cs_est = T_sc_est.inv();
        // this->printPoseInformation(T_cs_est);

        // MHEC::CalibrationCfg test_cfg;
        // test_cfg.max_iter = 12;
        // runTest("InEKF_RT", test_cfg, T_sc_est, scale_);
    });

    // 定义状态转换
    s1->addTransition(bridge1, &AsyncTaskBridge::taskFinished, s2);
    s2->addTransition(bridge2, &AsyncTaskBridge::taskFinished, s3);
    s3->addTransition(bridge3, &AsyncTaskBridge::taskFinished, s4);
    s4->addTransition(bridge4, &AsyncTaskBridge::taskFinished, s5);
    s5->addTransition(bridge5, &AsyncTaskBridge::taskFinished, s6);
    s6->addTransition(bridge6, &AsyncTaskBridge::taskFinished, s7);
    s7->addTransition(bridge7, &AsyncTaskBridge::taskFinished, s8);
    s8->addTransition(bridge8, &AsyncTaskBridge::taskFinished, s9);
    s9->addTransition(bridge9, &AsyncTaskBridge::taskFinished, s10);
    // s10->addTransition(bridge10, &AsyncTaskBridge::taskFinished, stateFinal);
    s10->addTransition(bridge10, &AsyncTaskBridge::taskFinished, s1);

    connect(bridge1, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge2, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge3, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge4, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge5, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge6, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge7, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge8, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge9, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);
    connect(bridge10, &AsyncTaskBridge::taskFinished, this, &MHECTask::stepFinished);

    this->staMachine_->setInitialState(s1);
}

void MHECTask::start()
{
    if (!this->staMachine_) {
        ERROR_("State machine not initialized");
        throw MAR::RobotException(MAR::MARInternalError, "State machine not initialized");
    }
    HIGHLIGHT_("[MHECTask] MHECTask task start at thread ID: %d", QThread::currentThreadId());
    this->staMachine_->start();
}

void MHECTask::stop()
{
    if (this->staMachine_) {
        if (!this->staMachine_->isRunning()) return;
        else ERROR_("[MHECTask] MHECTask task stoped at thread ID: %d", QThread::currentThreadId());
        this->staMachine_->stop();
        emit MHECTaskFinished(false);
    }
}

void MHECTask::onFinished()
{
    HIGHLIGHT_("[MHECTask] MHECTask task Finished at thread ID: %d", QThread::currentThreadId());
    emit MHECTaskFinished(true);
}

void MHECTask::stepFinished(bool success, const QString &errorMsg)
{
    if (!this->staMachine_->isRunning()) return;
    if (!success) {
        ERROR_("[MHECTask] Step failed: %s", errorMsg.toStdString().c_str());
        stop();
    }
}
void MHECTask::getScale(double& scale, cv::Mat& T_sc) {

    std::vector<Eigen::VectorXd> TargetPoses;
    // const double CALIB_STEP_SIZE = 200000;  // 5X
    const double CALIB_STEP_SIZE = 50000;  // 与5X2基线一致，单位nm
    Eigen::VectorXd start_pose;
    if(!this->robot_->getTaskPositions(start_pose)) {
        throw MAR::RobotException(MAR::MARInternalError, "11111111");
    }
    TargetPoses.clear();

    double start_offset = -1.0 * CALIB_STEP_SIZE; // centered 3x3 grid: -step, 0, +step

    for (int x_idx = 0; x_idx < 3; ++x_idx) {
        for (int y_idx = 0; y_idx < 3; ++y_idx) {
            Eigen::VectorXd pose = start_pose;
            pose(0) += start_offset + x_idx * CALIB_STEP_SIZE;
            pose(1) += start_offset + y_idx * CALIB_STEP_SIZE;
            TargetPoses.push_back(pose);
        }
    }

    MHEC::CalibrationData log;
    for (int i = 0; i < TargetPoses.size(); i++)
    {
        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->setTargetPose(TargetPoses[i]);
        QThread::msleep(1000);

        cv::Vec3d Point = cv::Vec3d(-1,-1,-1);
        // 预期10X比例只用于首次比例标定时选择自动对焦参数；
        // scale estimation itself still uses the original pixel coordinates.
        this->motionActor_->getPoint(Point, scale_ > 0.0 ? scale_ : 338.640);
        log.all.P_c_curr_list.push_back(Point);

        qInfo() << QString("[Scale] Pc %4: (%1, %2, %3)").arg(Point[0]).arg(Point[1]).arg(Point[2]).arg(i);

        Point[0] = TargetPoses[i][0];
        Point[1] = TargetPoses[i][1];
        Point[2] = TargetPoses[i][2];
        log.all.P_s_curr_list.push_back(Point);
    }

    this->robot_->setRobotMode(ROBOT_POSITION);
    this->robot_->setTargetPose(start_pose);
    QThread::msleep(1000);

    MHEC::saveCalbrationData(log, "ScaleData");

    MHEC::calculateCalibration(log.all.P_c_curr_list, log.all.P_s_curr_list, scale, T_sc);
}


void MHECTask::getInit(cv::Mat& T_sc_est, double scale) {
    double dt = 1.0;
    // double step_len_R = 2.0 / 180.0 * M_PI;  // 5X
    // double step_len_t = 200000.0;  // 5X
    double step_len_R = 0.25 / 180.0 * M_PI;  // 10X
    double step_len_t = 50000.0;  // 10X
    int N_rot = 8;
    int N_t = 8;

    MHEC::CalibrationData log;

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N_rot; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamTransMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[Init] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);


        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2000);
        T_se_curr = this->robot_->getTaskMat_cv();

        log.rot.Twist_c_list.push_back(twist_c);
        log.rot.Twist_s_list.push_back(twist_s);
        log.rot.P_c_prev_list.push_back(P_c_prev);
        log.rot.P_c_curr_list.push_back(P_c_curr);
        P_c_prev = P_c_Init;
    }

    for (int i = 0; i < N_t; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamRotMotion(T_se_curr, twist_c, twist_s, T_sc_est, P_c_prev, step_len_R, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[Init] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2000);
        T_se_curr = this->robot_->getTaskMat_cv();

        log.t.Twist_c_list.push_back(twist_c);
        log.t.Twist_s_list.push_back(twist_s);
        log.t.P_c_prev_list.push_back(P_c_prev);
        log.t.P_c_curr_list.push_back(P_c_curr);
        P_c_prev = P_c_Init;
    }

    MHEC::saveCalbrationData(log, "InitData");

    cv::Mat R_cs_est = MHEC::computeRotOLS(log, dt);
    cv::Mat t_cs_est = MHEC::computeTransOLS(log, R_cs_est, dt);

    cv::Mat T_cs_est = cv::Mat::eye(4, 4, CV_64F);
    R_cs_est.copyTo(T_cs_est(cv::Rect(0, 0, 3, 3)));
    t_cs_est.copyTo(T_cs_est(cv::Rect(3, 0, 1, 3)));
    T_sc_est = T_cs_est.inv();
}

void MHECTask::runOLS(const MHEC::CalibrationCfg& cfg,
            cv::Mat& T_sc_est, double scale) {
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N_rot = cfg.max_rot_iter;
    int N_t = cfg.max_p_iter;

    MHEC::CalibrationData log;

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N_rot; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamTransMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[OLS_rot] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);


        log.rot.Twist_c_list.push_back(twist_c);
        log.rot.Twist_s_list.push_back(twist_s);
        log.rot.P_c_prev_list.push_back(P_c_prev);
        log.rot.P_c_curr_list.push_back(P_c_curr);
        log.rot.T_se_list.push_back(T_se_curr);


        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2000);
        T_se_curr = this->robot_->getTaskMat_cv();

        this->getPoint(P_c_prev, scale);
    }

    for (int i = 0; i < N_t; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamRotMotion(T_se_curr, twist_c, twist_s, T_sc_est, P_c_prev, step_len_R, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[OLS_trans] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);

        log.t.Twist_c_list.push_back(twist_c);
        log.t.Twist_s_list.push_back(twist_s);
        log.t.P_c_prev_list.push_back(P_c_prev);
        log.t.P_c_curr_list.push_back(P_c_curr);
        log.t.T_se_list.push_back(T_se_curr);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2000);
        T_se_curr = this->robot_->getTaskMat_cv();

        this->getPoint(P_c_prev, scale);
    }

    // MHEC::saveCalbrationData(log, "OLSData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("OLSData_N%1_stepT%2_stepR%3")
                           .arg(N_rot + N_t)
                           .arg(stepT_safe, stepR_safe);
    MHEC::saveCalbrationData(log, fileName.toStdString());

    cv::Mat R_cs_est = MHEC::computeRotOLS(log, dt);
    cv::Mat t_cs_est = MHEC::computeTransOLS(log, R_cs_est, dt);

    cv::Mat T_cs_est = cv::Mat::eye(4, 4, CV_64F);
    R_cs_est.copyTo(T_cs_est(cv::Rect(0, 0, 3, 3)));
    t_cs_est.copyTo(T_cs_est(cv::Rect(3, 0, 1, 3)));
    T_sc_est = T_cs_est.inv();

}


void MHECTask::runIEKF(const MHEC::CalibrationCfg& cfg,
             MHEC::CalibrationState& sta,
             cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N = cfg.max_iter;
    if (N <= 0 || !std::isfinite(scale) || scale <= 0.0 ||
        !isValidRigidTransform(T_sc_est)) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid J-RIEKF experiment input");
    }

    MHEC::CalibrationData log;
    MHEC::resetCamMotionSequence();

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, step_len_R, dt);

        cv::Vec3d P_c_pred;this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(2500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                << QString("[IEKF] P_c %4/%5: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i).arg(N);

        log.all.Twist_c_list.push_back(twist_c);
        log.all.Twist_s_list.push_back(twist_s);
        log.all.P_c_prev_list.push_back(P_c_prev);
        log.all.P_c_curr_list.push_back(P_c_curr);
        // 使用规划目标位姿和规划旋量，不再额外读取编码器位姿重算运动。
        log.all.T_se_list.push_back(T_se_curr);

        HIGHLIGHT_("[MHECTask] Starting Step 3 at thread ID: %d", QThread::currentThreadId());
        // 前5次只采集；第6次记录堆叠雅可比秩并完成首次更新。
        if (static_cast<int>(log.all.P_c_curr_list.size()) == cfg.initialization_motions)
            qInfo() << "J-RIEKF initialization Jacobian rank ="
                    << MHEC::stackedJacobianRank(log, cfg.initialization_motions, cfg.state_length_scale);
        if (static_cast<int>(log.all.P_c_curr_list.size()) >= cfg.initialization_motions) {
            MHEC::updateIEKF(cfg, sta, log, T_sc_est);
            if (!isValidRigidTransform(T_sc_est)) {
                throw MAR::RobotException(MAR::MARInternalError,
                                          "J-RIEKF produced an invalid transform");
            }
        }

        cv::Mat T_cs_est = T_sc_est.inv();
        log.all.T_cs_list.push_back(T_cs_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition(-P_c_Init[2] / 1000.0);
        QThread::msleep(2500);
        T_se_curr = T_se_Init;
        this->getPoint(P_c_prev, scale);
    }

    // MHEC::saveCalbrationData(log, "IEKFData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("IEKFData_N%1_stepT%2_stepR%3")
                           .arg(N)
                           .arg(stepT_safe, stepR_safe);
    log.all.T_cs = T_sc_est.inv();
    MHEC::saveCalbrationData(log, (runTag_ + fileName).toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}



void MHECTask::runIEKF_RT(const MHEC::CalibrationCfg& cfg,
                       MHEC::CalibrationState& sta,
                       cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N_rot = cfg.max_rot_iter;
    int N_t = cfg.max_p_iter;

    MHEC::CalibrationData log;


    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N_rot; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamTransMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, dt);

        cv::Vec3d P_c_pred;this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(2500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                << QString("[IEKF_rot] P_c %4/%5: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i).arg(N_rot);

        log.rot.Twist_c_list.push_back(twist_c);
        log.rot.Twist_s_list.push_back(twist_s);
        log.rot.P_c_prev_list.push_back(P_c_prev);
        log.rot.P_c_curr_list.push_back(P_c_curr);
        log.rot.T_se_list.push_back(T_se_curr);

        MHEC::updateIEKFRot(cfg, sta, log, T_sc_est);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.rot.T_cs_list.push_back(T_cs_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition(-P_c_Init[2] / 1000.0);
        QThread::msleep(2500);
        T_se_curr = this->robot_->getTaskMat_cv();

        this->getPoint(P_c_prev, scale);
    }

    this->getPoint(P_c_prev, scale);
    P_c_Init = P_c_prev;

    for (int i = 0; i < N_t; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamRotMotion(T_se_curr, twist_c, twist_s, T_sc_est, P_c_prev, step_len_R, dt);

        cv::Vec3d P_c_pred;this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(2500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                << QString("[IEKF_t] P_c %4/%5: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i).arg(N_t);

        log.t.Twist_c_list.push_back(twist_c);
        log.t.Twist_s_list.push_back(twist_s);
        log.t.P_c_prev_list.push_back(P_c_prev);
        log.t.P_c_curr_list.push_back(P_c_curr);
        log.t.T_se_list.push_back(T_se_curr);

        MHEC::updateIEKFT(cfg, sta, log, T_sc_est);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.t.T_cs_list.push_back(T_cs_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2500);
        T_se_curr = this->robot_->getTaskMat_cv();

        this->getPoint(P_c_prev, scale);
    }

    // MHEC::saveCalbrationData(log, "ServoOnceData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("IEKF_RT%1_stepT%2_stepR%3")
                           .arg(N_rot + N_t)
                           .arg(stepT_safe, stepR_safe);
    MHEC::saveCalbrationData(log, fileName.toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}



void MHECTask::runEKF(const MHEC::CalibrationCfg& cfg,
                       MHEC::CalibrationState& sta,
                       cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N = cfg.max_iter;

    MHEC::CalibrationData log;

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, step_len_R, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[EKF] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.all.Twist_c_list.push_back(twist_c);
        log.all.Twist_s_list.push_back(twist_s);
        log.all.P_c_prev_list.push_back(P_c_prev);
        log.all.P_c_curr_list.push_back(P_c_curr);
        log.all.T_cs_list.push_back(T_cs_est);

        cv::Mat T_sc_temp = T_sc_est.clone();
        MHEC::updateEKF(cfg, sta, log, T_sc_temp);

        // 4. 将结果写回原始变量
        T_sc_est = T_sc_temp.clone();

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2000);
        T_se_curr = T_se_Init;
        P_c_prev = P_c_Init;
    }

    // MHEC::saveCalbrationData(log, "EKFData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("EKFData_N%1_stepT%2_stepR%3")
                           .arg(N)
                           .arg(stepT_safe, stepR_safe);
    MHEC::saveCalbrationData(log, fileName.toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}




void MHECTask::runServoTwoStage(const MHEC::CalibrationCfg& cfg,
                            MHEC::CalibrationState& sta,
                            cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N_rot = cfg.max_rot_iter;
    int N_t = cfg.max_p_iter;

    MHEC::CalibrationData log;

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N_rot; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamTransMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, dt);

        cv::Vec3d P_c_pred;this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(2500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                << QString("[ServoTwoStage_rot] P_c %4/%5: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i).arg(N_rot);

        log.rot.Twist_c_list.push_back(twist_c);
        log.rot.Twist_s_list.push_back(twist_s);
        log.rot.P_c_prev_list.push_back(P_c_prev);
        log.rot.P_c_curr_list.push_back(P_c_curr);
        log.rot.T_se_list.push_back(T_se_curr);

        cv::Mat T_sc_temp = T_sc_est.clone();
        cv::Mat T_cs_est = T_sc_temp.inv();
        MHEC::updateServoRot(cfg, sta, log, T_cs_est);
        T_sc_temp = T_cs_est.inv();
        T_sc_est = T_sc_temp.clone();
        // this->printPoseInformation(T_cs_est);

        log.rot.T_cs_list.push_back(T_cs_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2500);

        T_se_curr = this->robot_->getTaskMat_cv();

        this->getPoint(P_c_prev, scale);
    }

    this->getPoint(P_c_prev, scale);
    P_c_Init = P_c_prev;

    for (int i = 0; i < N_t; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamRotMotion(T_se_curr, twist_c, twist_s, T_sc_est, P_c_prev, step_len_R, dt);

        cv::Vec3d P_c_pred;this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(2500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                << QString("[ServoTwoStage_t] P_c %4/%5: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i).arg(N_t);

        log.t.Twist_c_list.push_back(twist_c);
        log.t.Twist_s_list.push_back(twist_s);
        log.t.P_c_prev_list.push_back(P_c_prev);
        log.t.P_c_curr_list.push_back(P_c_curr);
        log.t.T_se_list.push_back(T_se_curr);

        cv::Mat T_sc_temp = T_sc_est.clone();
        cv::Mat T_cs_est = T_sc_temp.inv();
        MHEC::updateServoT(cfg, sta, log, T_cs_est);
        T_sc_temp = T_cs_est.inv();
        T_sc_est = T_sc_temp.clone();
        // this->printPoseInformation(T_cs_est);

        log.t.T_cs_list.push_back(T_cs_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2500);
        T_se_curr = this->robot_->getTaskMat_cv();

        this->getPoint(P_c_prev, scale);
    }

    // MHEC::saveCalbrationData(log, "ServoOnceData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("ServoTwoStage%1_stepT%2_stepR%3")
                           .arg(N_rot + N_t)
                           .arg(stepT_safe, stepR_safe);
    MHEC::saveCalbrationData(log, fileName.toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}



void MHECTask::runServoOnce(const MHEC::CalibrationCfg& cfg,
                      MHEC::CalibrationState& sta,
                      cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N = cfg.max_iter;

    MHEC::CalibrationData log;
    MHEC::resetCamMotionSequence();

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, step_len_R, dt);

        cv::Vec3d P_c_pred;this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(2500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
               << QString("[Servo_once] P_c %4/%5: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i).arg(N);

        log.all.Twist_c_list.push_back(twist_c);
        log.all.Twist_s_list.push_back(twist_s);
        log.all.P_c_prev_list.push_back(P_c_prev);
        log.all.P_c_curr_list.push_back(P_c_curr);

        // 与论文模型一致，记录规划目标位姿及其对应的规划运动。
        log.all.T_se_list.push_back(T_se_curr);

        cv::Mat T_sc_temp = T_sc_est.clone();
        double scale_factor = cfg.state_length_scale;
        // double scale_factor = (std::abs(cfg.step_len_R) > 1e-12) ? (cfg.step_len_t / cfg.step_len_R) : 1.0;
        // 这里只记录可观测性诊断，不因数值阈值直接终止整轮硬件实验。
        if (static_cast<int>(log.all.P_c_curr_list.size()) == cfg.initialization_motions)
            qInfo() << "J-FGD initialization Jacobian rank ="
                    << MHEC::stackedJacobianRank(log, cfg.initialization_motions, cfg.state_length_scale);
        if (static_cast<int>(log.all.P_c_curr_list.size()) >= cfg.initialization_motions) {
            MHEC::updateServoOnce(cfg, sta, log, T_sc_temp, scale_factor);
            if (!isValidRigidTransform(T_sc_temp)) {
                throw MAR::RobotException(MAR::MARInternalError,
                                          "J-FGD produced an invalid transform");
            }
        }
        T_sc_est = T_sc_temp.clone();

        cv::Mat T_cs_est = T_sc_est.inv();
        log.all.T_cs_list.push_back(T_cs_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition(-P_c_Init[2] / 1000.0);
        QThread::msleep(2500);
        T_se_curr = T_se_Init;
        this->getPoint(P_c_prev, scale);
    }

    // MHEC::saveCalbrationData(log, "ServoOnceData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("ServoOnceData_N%1_stepT%2_stepR%3")
                           .arg(N)
                           .arg(stepT_safe, stepR_safe);
    log.all.T_cs = T_sc_est.inv();
    MHEC::saveCalbrationData(log, (runTag_ + fileName).toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}



void MHECTask::runDataAquire(const MHEC::CalibrationCfg& cfg,
                   cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N = cfg.max_iter;

    MHEC::CalibrationData log;

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, step_len_R, dt);

        cv::Vec3d P_c_pred;this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(1500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                << QString("[DataAquire] P_c %4/%5: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i).arg(N);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.all.Twist_c_list.push_back(twist_c);
        log.all.Twist_s_list.push_back(twist_s);
        log.all.P_c_prev_list.push_back(P_c_prev);
        log.all.P_c_curr_list.push_back(P_c_curr);
        log.all.T_cs_list.push_back(T_cs_est);
        log.all.T_se_list.push_back(T_se_curr);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition(-P_c_Init[2] / 1000.0);
        QThread::msleep(2000);
        T_se_curr = T_se_Init;
        this->getPoint(P_c_prev, scale);
    }

    // MHEC::saveCalbrationData(log, "ServoOnceData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("DataAquire_N%1_stepT%2_stepR%3")
                           .arg(N)
                           .arg(stepT_safe, stepR_safe);
    MHEC::saveCalbrationData(log, fileName.toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}

double MHECTask::runTest(const std::string& name, const MHEC::CalibrationCfg& cfg,
             cv::Mat& T_sc_est, double scale, const cv::Mat& pointCovariance)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N = cfg.max_iter;
    if (N <= 0 || !std::isfinite(scale) || scale <= 0.0 ||
        !isValidRigidTransform(T_sc_est)) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid calibration validation input");
    }

    MHEC::CalibrationData log;
    MHEC::resetCamMotionSequence(cfg.excitation_sequence_offset);

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N; i++) {
        cv::Mat twist_c_command, twist_s;
        // 平台目标始终由共同初值规划，保证所有候选标定结果经历相同的运动协议。
        MHEC::planCamMotion(T_se_curr, twist_c_command, twist_s,
                            T_cs_est_.inv(), step_len_t, step_len_R, dt);

        // 验证必须使用待评估的T_sc_est解释同一个平台旋量；否则预测点只与
        // 共同规划初值有关，无法区分三种方法的最终标定结果。
        cv::Mat twist_c_validation =
            cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est.inv()))) * twist_s;

        cv::Vec3d P_c_pred;
        this->getTargetPoint(P_c_pred, P_c_prev, twist_c_validation, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        // 对焦预定位跟随实际下发的规划运动，不使用候选标定结果改变硬件轨迹。
        this->gotoPoint(P_c_prev, twist_c_command);
        QThread::msleep(1500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        cv::Vec3d P_c_e = P_c_curr - P_c_pred;
        qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                << QString("[runTest(e)] P_c %4/%5: (%1, %2, %3)").arg(P_c_e[0]).arg(P_c_e[1]).arg(P_c_e[2]).arg(i).arg(N);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.all.Twist_c_list.push_back(twist_c_validation);
        log.all.Twist_s_list.push_back(twist_s);
        log.all.P_c_prev_list.push_back(P_c_prev);
        log.all.P_c_curr_list.push_back(P_c_curr);
        log.all.T_cs_list.push_back(T_cs_est);
        // 验证同样使用规划运动，不在运动后重新读取平台位姿。
        log.all.T_se_list.push_back(T_se_curr);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition(-P_c_Init[2] / 1000.0);
        QThread::msleep(2000);
        T_se_curr = T_se_Init;
        this->getPoint(P_c_prev, scale);
    }

    // MHEC::saveCalbrationData(log, "ServoOnceData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("test_N%1_stepT%2_stepR%3")
                           .arg(N)
                           .arg(stepT_safe, stepR_safe);
    log.all.T_cs = T_sc_est.inv();
    MHEC::saveCalbrationData(log, name + fileName.toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();



    // ================== 计算重投影误差 (3D坐标预测误差) ==================
    double total_error = 0.0;
    double max_error = 0.0;
    double sq_error_sum = 0.0;
    cv::Vec3d axisSquaredSum(0.0, 0.0, 0.0);
    double normalizedSquaredSum = 0.0;
    const bool reportNormalizedError =
        pointCovariance.rows == 3 && pointCovariance.cols == 3 &&
        pointCovariance.type() == CV_64F && cv::checkRange(pointCovariance);
    std::vector<double> error_list;

    for (int i = 0; i < N; i++) {
        cv::Vec3d P_curr = log.all.P_c_curr_list[i];
        cv::Vec3d P_prev = log.all.P_c_prev_list[i];
        cv::Mat twist_c  = log.all.Twist_c_list[i];

        // 根据前一时刻的点和相机运动，重新计算预测的当前点
        cv::Vec3d P_pred;
        this->getTargetPoint(P_pred, P_prev, twist_c, scale);

        // 计算 3D 欧氏距离 (L2 Norm)
        double err = cv::norm(cv::Vec3d(P_curr) - P_pred);
        const cv::Vec3d residual = cv::Vec3d(P_curr) - P_pred;
        error_list.push_back(err);

        total_error += err;
        sq_error_sum += (err * err);
        for (int axis = 0; axis < 3; ++axis)
            axisSquaredSum[axis] += residual[axis] * residual[axis];

        if (reportNormalizedError) {
            const cv::Mat deltaTransform =
                cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));
            const cv::Mat deltaRotation = deltaTransform(cv::Rect(0, 0, 3, 3));
            cv::Mat residualCovariance = pointCovariance +
                deltaRotation * pointCovariance * deltaRotation.t();
            residualCovariance = 0.5 *
                (residualCovariance + residualCovariance.t());
            residualCovariance += cv::Mat::eye(3, 3, CV_64F);
            cv::Mat inverseCovariance;
            cv::invert(residualCovariance, inverseCovariance, cv::DECOMP_SVD);
            const cv::Mat residualMat = (cv::Mat_<double>(3, 1) <<
                residual[0], residual[1], residual[2]);
            const cv::Mat normalizedValue =
                residualMat.t() * inverseCovariance * residualMat;
            normalizedSquaredSum += normalizedValue.at<double>(0, 0);
        }

        if (err > max_error) {
            max_error = err;
        }
    }

    // 统计数据计算
    double mean_error = total_error / N;
    double rmse = std::sqrt(sq_error_sum / N); // Root Mean Square Error

    // 打印误差报告
    qInfo() << "================ Calibration Test Error Report ================";
    qInfo() << QString("Total Test Samples : %1").arg(N);
    qInfo() << QString("Mean Error (MAE)   : %1").arg(mean_error);
    qInfo() << QString("Max Error          : %1").arg(max_error);
    qInfo() << QString("Root Mean Sq (RMSE): %1").arg(rmse);
    qInfo() << QString("Axis RMSE X/Y/Z   : %1, %2, %3")
                   .arg(std::sqrt(axisSquaredSum[0] / N))
                   .arg(std::sqrt(axisSquaredSum[1] / N))
                   .arg(std::sqrt(axisSquaredSum[2] / N));
    if (reportNormalizedError) {
        qInfo() << QString("Cov-normalized RMSE: %1")
                       .arg(std::sqrt(normalizedSquaredSum / (3.0 * N)));
    }
    qInfo() << "===============================================================";
    return rmse;
}


void MHECTask::runSharedValidation10X(
    const MHEC::CalibrationCfg& cfg,
    const std::vector<cv::Mat>& candidateTsc,
    const std::vector<std::string>& methodNames,
    double scale,
    const cv::Mat& pointCovariance,
    const QString& roundTag)
{
    const int methodCount = static_cast<int>(candidateTsc.size());
    const int sampleCount = cfg.max_iter;
    if (methodCount <= 0 || methodNames.size() != candidateTsc.size() ||
        sampleCount <= 0 || !std::isfinite(scale) || scale <= 0.0) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid shared validation input");
    }
    for (const cv::Mat& transform : candidateTsc) {
        if (!isValidRigidTransform(transform)) {
            throw MAR::RobotException(MAR::MARInternalError,
                                      "Invalid candidate in shared validation");
        }
    }

    const bool useNormalizedMetric =
        pointCovariance.rows == 3 && pointCovariance.cols == 3 &&
        pointCovariance.type() == CV_64F && cv::checkRange(pointCovariance);
    std::vector<double> squaredErrorSum(methodCount, 0.0);
    std::vector<double> normalizedSquaredSum(methodCount, 0.0);
    std::vector<cv::Vec3d> axisSquaredSum(methodCount, cv::Vec3d(0.0, 0.0, 0.0));
    std::vector<MHEC::CalibrationData> candidateLogs(methodCount);

    MHEC::resetCamMotionSequence(cfg.excitation_sequence_offset);
    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    const cv::Mat T_se_initial = T_se_curr.clone();
    cv::Vec3d pointPrevious;
    this->getPoint(pointPrevious, scale);
    const cv::Vec3d pointInitial = pointPrevious;

    for (int sample = 0; sample < sampleCount; ++sample) {
        cv::Mat twistCameraCommand, twistBase;
        // A fixed common matrix generates the physical validation trajectory.
        // Candidate matrices only reinterpret this same base-frame motion.
        MHEC::planCamMotion(T_se_curr, twistCameraCommand, twistBase,
                            T_cs_est_.inv(), cfg.step_len_t,
                            cfg.step_len_R, cfg.dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(pointPrevious, twistCameraCommand);
        QThread::msleep(1500);

        cv::Vec3d pointCurrent;
        this->getPoint(pointCurrent, scale);

        for (int method = 0; method < methodCount; ++method) {
            const cv::Mat candidateTcs = candidateTsc[method].inv();
            const cv::Mat twistCamera =
                cvMatFromEigen(Adjoint(eigenFromCvMat(candidateTcs))) * twistBase;
            const cv::Mat deltaTransform =
                cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twistCamera))));
            const cv::Mat pointPreviousHomogeneous =
                (cv::Mat_<double>(4, 1) << pointPrevious[0], pointPrevious[1],
                 pointPrevious[2], 1.0);
            const cv::Mat prediction = deltaTransform * pointPreviousHomogeneous;
            const cv::Vec3d residual(
                pointCurrent[0] - prediction.at<double>(0),
                pointCurrent[1] - prediction.at<double>(1),
                pointCurrent[2] - prediction.at<double>(2));

            squaredErrorSum[method] += residual.dot(residual);
            for (int axis = 0; axis < 3; ++axis)
                axisSquaredSum[method][axis] += residual[axis] * residual[axis];

            if (useNormalizedMetric) {
                const cv::Mat deltaRotation = deltaTransform(cv::Rect(0, 0, 3, 3));
                cv::Mat residualCovariance = pointCovariance +
                    deltaRotation * pointCovariance * deltaRotation.t();
                residualCovariance = 0.5 *
                    (residualCovariance + residualCovariance.t());
                residualCovariance += cv::Mat::eye(3, 3, CV_64F);
                cv::Mat inverseCovariance;
                cv::invert(residualCovariance, inverseCovariance, cv::DECOMP_SVD);
                const cv::Mat residualMatrix =
                    (cv::Mat_<double>(3, 1) << residual[0], residual[1], residual[2]);
                const cv::Mat normalizedValue =
                    residualMatrix.t() * inverseCovariance * residualMatrix;
                normalizedSquaredSum[method] += normalizedValue.at<double>(0, 0);
            }

            MHEC::CalibrationData& methodLog = candidateLogs[method];
            methodLog.all.Twist_c_list.push_back(twistCamera);
            methodLog.all.Twist_s_list.push_back(twistBase);
            methodLog.all.P_c_prev_list.push_back(pointPrevious);
            methodLog.all.P_c_curr_list.push_back(pointCurrent);
            methodLog.all.T_se_list.push_back(T_se_curr);
            methodLog.all.T_cs_list.push_back(candidateTcs);

            qInfo() << QDateTime::currentDateTime().toString("[HH:mm:ss.zzz]")
                    << QString("[SharedValidation %1] %2/%3 residual: (%4, %5, %6)")
                           .arg(QString::fromStdString(methodNames[method]))
                           .arg(sample).arg(sampleCount)
                           .arg(residual[0]).arg(residual[1]).arg(residual[2]);
        }

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_initial));
        motionActor_->setTargetPosition(-pointInitial[2] / 1000.0);
        QThread::msleep(2000);
        T_se_curr = T_se_initial;
        this->getPoint(pointPrevious, scale);
    }

    const QString stepT = QString::number(cfg.step_len_t).replace('.', '_');
    const QString stepR = QString::number(cfg.step_len_R).replace('.', '_');
    for (int method = 0; method < methodCount; ++method) {
        const double rmse = std::sqrt(squaredErrorSum[method] / sampleCount);
        qInfo() << "================ Shared Validation Report ================";
        qInfo() << "Method:" << QString::fromStdString(methodNames[method]);
        qInfo() << "3D RMSE (nm):" << rmse;
        qInfo() << QString("Axis RMSE X/Y/Z (nm): %1, %2, %3")
                       .arg(std::sqrt(axisSquaredSum[method][0] / sampleCount))
                       .arg(std::sqrt(axisSquaredSum[method][1] / sampleCount))
                       .arg(std::sqrt(axisSquaredSum[method][2] / sampleCount));
        if (useNormalizedMetric) {
            qInfo() << "Cov-normalized RMSE:"
                    << std::sqrt(normalizedSquaredSum[method] /
                                 (3.0 * sampleCount));
        }
        qInfo() << "==========================================================";

        candidateLogs[method].all.T_cs = candidateTsc[method].inv();
        const QString filename = QString("%1_%2_SharedValidation_N%3_stepT%4_stepR%5")
            .arg(roundTag)
            .arg(QString::fromStdString(methodNames[method]))
            .arg(sampleCount)
            .arg(stepT)
            .arg(stepR);
        MHEC::saveCalbrationData(candidateLogs[method], filename.toStdString());
    }
}


void MHECTask::runTransTest(cv::Mat& T_sc_est, double scale, double step)
{
    if (T_sc_est.empty() || T_sc_est.rows != 4 || T_sc_est.cols != 4) {
        qCritical() << "Error: T_sc_est is not a valid 4x4 transformation matrix.";
        return;
    }

    // 1. 获取初始状态
    cv::Mat T_se_initial = this->robot_->getTaskMat_cv(); // 当前末端相对于基座的位姿
    cv::Vec3d P_c_initial;
    this->getPoint(P_c_initial, scale);

    // 将初始观测点转为齐次坐标 (相机系下)
    cv::Mat P_c_init_homo = (cv::Mat_<double>(4, 1) << P_c_initial[0], P_c_initial[1], P_c_initial[2], 1.0);

    // 【眼在手外核心】：计算目标点在“机器人末端坐标系(e)”下的固定位置
    // P_e = T_se_initial.inv() * T_sc_est * P_c_initial
    cv::Mat t_eo = T_se_initial.inv() * T_sc_est * P_c_init_homo;

    // this->printPoseInformation(T_se_initial);

    qInfo() << QString("t_eo: T_x=%2, T_y=%3, T_z=%4")
                   .arg(t_eo.at<double>(0, 0))
                   .arg(t_eo.at<double>(1, 0)).arg(t_eo.at<double>(2, 0));

    std::vector<cv::Vec3d> translations = {
        // {0.0, 0.0, 0.0},
        {step, 0.0, 0.0}, {0.0, step, 0.0}, {0.0, 0.0, step},
        {-step, 0.0, 0.0}, {0.0, -step, 0.0}, {0.0, 0.0, -step}
    };

    double total_error = 0.0;
    int valid_count = 0;

    for (size_t i = 0; i < translations.size(); ++i) {
        cv::Vec3d trans = translations[i];

        // 2. 构建目标位姿 (在基座系下移动末端)
        cv::Mat T_se_target = T_se_initial.clone();
        T_se_target.at<double>(0, 3) += trans[0];
        T_se_target.at<double>(1, 3) += trans[1];
        T_se_target.at<double>(2, 3) += trans[2];

        // qInfo() << QString("Moving to Pose %1: T_x=%2, T_y=%3, T_z=%4")
        //                .arg(i+1).arg(T_se_target.at<double>(0, 3))
        //                .arg(T_se_target.at<double>(1, 3)).arg(T_se_target.at<double>(2, 3));

        // 3. 执行移动
        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_target));
        QThread::msleep(1500);

        // 4. 获取实际相机观测值
        cv::Vec3d P_c_raw;
        this->getPoint(P_c_raw, scale);

        // 5. 计算理论预测值 (眼在手外逻辑)
        // 预测观测点 P_c_pred = T_sc_est.inv() * T_se_target * t_eo
        cv::Mat P_c_pred_homo = T_sc_est.inv() * T_se_target * t_eo;

        cv::Point3f p_pred(
            P_c_pred_homo.at<double>(0),
            P_c_pred_homo.at<double>(1),
            P_c_pred_homo.at<double>(2)
            );

        cv::Vec3d circle(P_c_pred_homo.at<double>(0)/scale + 2448/2,
                         P_c_pred_homo.at<double>(1)/scale + 2048/2,
                         50000/scale);

        emit sigTargetCircle(circle);


        // 6. 误差计算 (欧氏距离)
        double err = cv::norm(cv::Vec3d(p_pred.x, p_pred.y, p_pred.z) - P_c_raw);
        total_error += err;
        valid_count++;

        qInfo() << QString("Pose %1 | Err: %2 nm | Obs X: %3 | Pred X: %4 | Obs Y: %5 | Pred Y: %6 | Obs Z: %7 | Pred Z: %8")
                       .arg(i).arg(err, 0, 'f', 2)
                       .arg(P_c_raw[0]).arg(p_pred.x)
                       .arg(P_c_raw[1]).arg(p_pred.y)
                       .arg(P_c_raw[2]).arg(p_pred.z);
    }

    // 7. 统计输出
    if (valid_count > 0) {
        qInfo() << "------------------------------------------";
        qInfo() << "Eye-to-Hand Translation Test Results:";
        qInfo() << "Valid Points:" << valid_count;
        qInfo() << "Mean Error:" << (total_error / valid_count) << "nm";
    }

    this->robot_->setRobotMode(ROBOT_POSITION);
    this->robot_->moveAbsolute(eigenFromCvMat(T_se_initial));
    QThread::msleep(1500);
}


void MHECTask::runRotTest(cv::Mat& T_sc_est, double scale, double angle_deg)
{
    if (T_sc_est.empty() || T_sc_est.rows != 4 || T_sc_est.cols != 4) {
        qCritical() << "Error: T_sc_est is invalid.";
        return;
    }

    // 1. 获取初始状态
    cv::Mat T_se_initial = this->robot_->getTaskMat_cv();
    cv::Vec3d P_c_initial;
    this->getPoint(P_c_initial, scale);

    // 计算特征点在末端系(e)下的坐标 t_eo (Homogeneous)
    cv::Mat P_c_init_homo = (cv::Mat_<double>(4, 1) << P_c_initial[0], P_c_initial[1], P_c_initial[2], 1.0);
    // t_eo 是特征点在机器人手抓坐标系下的固定位置
    cv::Mat t_eo_homo = T_se_initial.inv() * T_sc_est * P_c_init_homo;
    cv::Mat t_eo = t_eo_homo(cv::Rect(0, 0, 1, 3)).clone(); // 3x1 向量

    // 计算特征点在基座系(s)下的固定位置 P_s (作为旋转中心)
    cv::Mat P_s_homo = T_sc_est * P_c_init_homo;
    cv::Mat P_s = P_s_homo(cv::Rect(0, 0, 1, 3)).clone(); // 3x1 向量

    // 2. 定义旋转测试序列
    double rad = angle_deg * CV_PI / 180.0;
    std::vector<cv::Vec3d> rotations = {
        // {0.0, 0.0, 0.0},
        {rad, 0.0, 0.0}, {-rad, 0.0, 0.0},
        {0.0, rad, 0.0}, {0.0, -rad, 0.0},
        {0.0, 0.0, rad}, {0.0, 0.0, -rad}
    };

    double total_error = 0.0;
    int valid_count = 0;

    for (size_t i = 0; i < rotations.size(); ++i) {
        cv::Vec3d rot_vec = rotations[i];

        // 3. 构建目标位姿：绕特征点 P_s 旋转
        cv::Mat R_delta;
        cv::Rodrigues(rot_vec, R_delta);

        cv::Mat R_initial = T_se_initial(cv::Rect(0, 0, 3, 3));
        cv::Mat R_target = R_initial * R_delta; // 得到目标旋转矩阵

        // 计算目标平移 t_target，使得 T_target * t_eo = P_s
        // t_target = P_s - R_target * t_eo
        cv::Mat t_target = P_s - R_target * t_eo;

        // 组装完整的 4x4 T_se_target
        cv::Mat T_se_target = cv::Mat::eye(4, 4, CV_64F);
        R_target.copyTo(T_se_target(cv::Rect(0, 0, 3, 3)));
        t_target.copyTo(T_se_target(cv::Rect(3, 0, 1, 3)));

        // 4. 执行移动
        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_target));
        QThread::msleep(2500); // 包含平移和旋转，稍微延长等待时间

        // 5. 获取实际观测
        cv::Vec3d P_c_current;
        this->getPoint(P_c_current, scale);

        // 6. 误差计算
        // 理论上如果标定完美且绕该点旋转，P_c_current 应该等于 P_c_initial
        double err = cv::norm(P_c_current - P_c_initial);

        total_error += err;
        valid_count++;

        qInfo() << QString("RotTest %1 | Drift Err: %2 | Current P_c: [%3,%4,%5]")
                       .arg(i).arg(err, 0, 'f', 4)
                       .arg(P_c_current[0]).arg(P_c_current[1]).arg(P_c_current[2]);
    }

    if (valid_count > 0) {
        qInfo() << "==========================================";
        qInfo() << "Remote Center Motion (RCM) Test Results:";
        qInfo() << "Average Point Drift (Calibration Error):" << (total_error / valid_count) << "units";
        qInfo() << "==========================================";
    }
    this->robot_->setRobotMode(ROBOT_POSITION);
    this->robot_->moveAbsolute(eigenFromCvMat(T_se_initial));
    QThread::msleep(1500);
}

// void MHECTask::runRotTest(cv::Mat& T_sc_est, double scale, double angle_deg)
// {
//     if (T_sc_est.empty() || T_sc_est.rows != 4 || T_sc_est.cols != 4) {
//         qCritical() << "Error: T_sc_est is invalid.";
//         return;
//     }

//     // 1. 获取初始状态
//     cv::Mat T_se_initial = this->robot_->getTaskMat_cv();
//     cv::Vec3d P_c_initial;
//     this->getPoint(P_c_initial, scale);

//     // 计算特征点在末端系(e)下的坐标 t_eo
//     cv::Mat P_c_init_homo = (cv::Mat_<double>(4, 1) << P_c_initial[0], P_c_initial[1], P_c_initial[2], 1.0);
//     cv::Mat t_eo = T_se_initial.inv() * T_sc_est * P_c_init_homo;

//     // 2. 定义旋转测试序列 (分别绕末端坐标系的 X, Y, Z 轴旋转)
//     // 格式：{Roll, Pitch, Yaw} 单位：弧度
//     double rad = angle_deg * CV_PI / 180.0;
//     std::vector<cv::Vec3d> rotations = {
//         {0.0, 0.0, 0.0},       // 基准位姿
//         {rad, 0.0, 0.0}, {-rad, 0.0, 0.0}, // 绕X旋转
//         {0.0, rad, 0.0}, {0.0, -rad, 0.0}, // 绕Y旋转
//         {0.0, 0.0, rad}, {0.0, 0.0, -rad}  // 绕Z旋转
//     };

//     double total_error = 0.0;
//     int valid_count = 0;

//     for (size_t i = 0; i < rotations.size(); ++i) {
//         cv::Vec3d rot_vec = rotations[i];

//         // 3. 构建目标位姿
//         // 注意：旋转通常是以末端 TCP 为中心旋转，保持 T_se_target 的平移部分不变
//         // 但为了严格测试旋转矩阵 R，我们通过欧拉角增量合成新的变换矩阵
//         cv::Mat R_delta;
//         // 使用 Rodrigues 或由欧拉角转旋转矩阵
//         cv::Vec3d rvec = rot_vec;
//         cv::Rodrigues(rvec, R_delta);

//         cv::Mat T_se_target = T_se_initial.clone();
//         cv::Mat R_initial = T_se_initial(cv::Rect(0, 0, 3, 3));
//         cv::Mat R_target = R_initial * R_delta; // 在当前末端系基础上旋转
//         R_target.copyTo(T_se_target(cv::Rect(0, 0, 3, 3)));

//         // 4. 执行移动
//         this->robot_->setRobotMode(ROBOT_POSITION);
//         this->robot_->moveAbsolute(eigenFromCvMat(T_se_target));
//         QThread::msleep(2000); // 旋转运动可能较慢，建议增加等待时间

//         // 5. 获取实际观测
//         cv::Vec3d P_c_raw;
//         this->getPoint(P_c_raw, scale);

//         // 6. 计算理论预测值
//         // P_c_pred = T_sc_est.inv() * T_se_target * t_eo
//         cv::Mat P_c_pred_homo = T_sc_est.inv() * T_se_target * t_eo;

//         double p_pred_x = P_c_pred_homo.at<double>(0);
//         double p_pred_y = P_c_pred_homo.at<double>(1);
//         double p_pred_z = P_c_pred_homo.at<double>(2);

//         // 7. 误差计算
//         double err = cv::norm(cv::Vec3d(p_pred_x, p_pred_y, p_pred_z) - P_c_raw);
//         total_error += err;
//         valid_count++;

//         qInfo() << QString("RotPose %1 | Err: %2 | Obs: [%3,%4,%5] | Pred: [%6,%7,%8]")
//                        .arg(i).arg(err, 0, 'f', 4)
//                        .arg(P_c_raw[0]).arg(P_c_raw[1]).arg(P_c_raw[2])
//                        .arg(p_pred_x).arg(p_pred_y).arg(p_pred_z);
//     }

//     if (valid_count > 0) {
//         qInfo() << "==========================================";
//         qInfo() << "Eye-to-Hand Rotation Test Results:";
//         qInfo() << "Mean Reprojection Error:" << (total_error / valid_count) << "units";
//         qInfo() << "==========================================";
//     }
// }

void MHECTask::runNoiseCal(const MHEC::CalibrationCfg& cfg,
                 MHEC::CalibrationState& sta,
                 cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N = cfg.max_iter;

    MHEC::CalibrationData log;

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, step_len_R, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(3500);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        QThread::msleep(3500);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[Noise] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.all.Twist_c_list.push_back(twist_c);
        log.all.Twist_s_list.push_back(twist_s);
        log.all.P_c_prev_list.push_back(P_c_prev);
        log.all.P_c_curr_list.push_back(P_c_curr);
        log.all.T_cs_list.push_back(T_cs_est);
        log.all.T_se_list.push_back(T_se_curr);

        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2000);
        T_se_curr = T_se_Init;
        P_c_prev = P_c_Init;
    }

    MHEC::saveCalbrationData(log, "ServoOnceData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("Noise%1_stepT%2_stepR%3")
                           .arg(N)
                           .arg(stepT_safe, stepR_safe);
    MHEC::saveCalbrationData(log, fileName.toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}


void MHECTask::runYKronT(const MHEC::CalibrationCfg& cfg,
               MHEC::CalibrationState& sta,
               cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N = cfg.max_iter;

    cv::Mat T_sc_Init = T_sc_est.clone();

    MHEC::CalibrationData log;
    MHEC::resetCamMotionSequence();

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr;
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;
    log.all.P_c_curr_list.push_back(P_c_Init);
    log.all.P_c_prev_list.push_back(P_c_Init);
    log.all.T_se_list.push_back(T_se_Init);
    log.all.T_cs_list.push_back(T_sc_est.inv());
    log.all.Twist_c_list.push_back(cv::Mat::zeros(6, 1, CV_64F));
    log.all.Twist_s_list.push_back(cv::Mat::zeros(6, 1, CV_64F));

    for (int i = 0; i < N; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamMotion(T_se_curr, twist_c, twist_s, T_sc_Init, step_len_t, step_len_R, dt);

        // 与J-FGD和J-RIEKF保持一致：先根据规划的相机运动预测特征点，
        // 再将自动对焦轴预移到预计焦面。尤其对±Z激励，若缺少该步骤，
        // 自动对焦会始终从初始位置开始并可能超出局部搜索范围。
        cv::Vec3d P_c_pred;
        this->getTargetPoint(P_c_pred, P_c_prev, twist_c, scale);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        this->gotoPoint(P_c_prev, twist_c);
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[YKronT] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.all.Twist_c_list.push_back(twist_c);
        log.all.Twist_s_list.push_back(twist_s);
        log.all.P_c_prev_list.push_back(P_c_prev);
        log.all.P_c_curr_list.push_back(P_c_curr);
        log.all.T_cs_list.push_back(T_cs_est);
        // PO-LS保存规划目标位姿，不在运动后重新读取平台编码器。
        log.all.T_se_list.push_back(T_se_curr);

        cv::Mat T_sc_temp = T_sc_est.clone();
        // PO-LS仍由内部线性系统秩检查保护；此处只输出共同J的诊断结果。
        if (i + 1 == cfg.initialization_motions)
            qInfo() << "PO-LS initialization Jacobian rank ="
                    << MHEC::stackedJacobianRank(log, cfg.initialization_motions, cfg.state_length_scale);
        if (i + 1 >= cfg.initialization_motions) {
            MHEC::updateYKronT(cfg, sta, log, T_sc_temp);
            if (!isValidRigidTransform(T_sc_temp)) {
                throw MAR::RobotException(MAR::MARInternalError,
                                          "PO-LS produced an invalid transform");
            }
        }

        // 4. 将结果写回原始变量
        T_sc_est = T_sc_temp.clone();
        log.all.T_cs_list.back() = T_sc_est.inv();

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition(-P_c_Init[2] / 1000.0);
        QThread::msleep(2000);
        T_se_curr = T_se_Init;
        P_c_prev = P_c_Init;
    }

    // MHEC::saveCalbrationData(log, "ServoOnceData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("YKronTData_N%1_stepT%2_stepR%3")
                           .arg(N)
                           .arg(stepT_safe, stepR_safe);
    log.all.T_cs = T_sc_est.inv();
    MHEC::saveCalbrationData(log, (runTag_ + fileName).toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}


void MHECTask::runRLS(const MHEC::CalibrationCfg& cfg,
                            MHEC::CalibrationState& sta,
                            cv::Mat& T_sc_est, double scale)
{
    double dt = cfg.dt;
    double step_len_R = cfg.step_len_R;
    double step_len_t = cfg.step_len_t;
    int N_rot = cfg.max_rot_iter;
    int N_t = cfg.max_p_iter;

    MHEC::CalibrationData log;

    cv::Mat T_se_curr = this->robot_->getTaskMat_cv();
    cv::Mat T_se_Init = T_se_curr.clone();
    cv::Vec3d P_c_prev = cv::Vec3d(-1, -1, -1);
    this->getPoint(P_c_prev, scale);
    cv::Vec3d P_c_Init = P_c_prev;

    for (int i = 0; i < N_rot; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamTransMotion(T_se_curr, twist_c, twist_s, T_sc_est, step_len_t, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        // qInfo() << QString("[RLS_rot] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.rot.Twist_c_list.push_back(twist_c);
        log.rot.Twist_s_list.push_back(twist_s);
        log.rot.P_c_prev_list.push_back(P_c_prev);
        log.rot.P_c_curr_list.push_back(P_c_curr);
        log.rot.T_cs_list.push_back(T_cs_est);

        MHEC::updateRotRLS(cfg, sta, log, T_sc_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition(-P_c_Init[2] / 1000.0);
        QThread::msleep(2000);
        T_se_curr = T_se_Init;
        P_c_prev = P_c_Init;
    }

    for (int i = 0; i < N_t; i++) {
        cv::Mat twist_c, twist_s;
        MHEC::planCamRotMotion(T_se_curr, twist_c, twist_s, T_sc_est, P_c_prev, step_len_R, dt);

        this->robot_->setRobotMode(ROBOT_POSITION);
        this->robot_->moveAbsolute(eigenFromCvMat(T_se_curr));
        QThread::msleep(2000);

        cv::Vec3d P_c_curr;
        this->getPoint(P_c_curr, scale);
        qInfo() << QString("[RLS_trans] P_c %4: (%1, %2, %3)").arg(P_c_curr[0]).arg(P_c_curr[1]).arg(P_c_curr[2]).arg(i);

        cv::Mat T_cs_est = T_sc_est.inv();
        log.t.Twist_c_list.push_back(twist_c);
        log.t.Twist_s_list.push_back(twist_s);
        log.t.P_c_prev_list.push_back(P_c_prev);
        log.t.P_c_curr_list.push_back(P_c_curr);
        log.t.T_cs_list.push_back(T_cs_est);

        MHEC::updateTransRLS(cfg, sta, log, T_sc_est);

        this->robot_->moveAbsolute(eigenFromCvMat(T_se_Init));
        motionActor_->setTargetPosition( -P_c_Init[2]/1000);
        QThread::msleep(2000);
        T_se_curr = T_se_Init;
        P_c_prev = P_c_Init;
    }

    // MHEC::saveCalbrationData(log, "RLSData");
    QString stepT_safe = QString::number(cfg.step_len_t).replace('.', '_');
    QString stepR_safe = QString::number(cfg.step_len_R).replace('.', '_');
    QString fileName = QString("RLSData_N%1_stepT%2_stepR%3")
                           .arg(N_rot + N_t)
                           .arg(stepT_safe, stepR_safe);
    MHEC::saveCalbrationData(log, fileName.toStdString());

    cv::Mat T_cs_est = T_sc_est.inv();
}

cv::Mat MHECTask::runStaticNoise10X(int samples, double scale)
{
    // PI平台保持静止，每次独立执行自动对焦并读取三维特征点。
    // 统计量覆盖图像定位和自动对焦组成的完整三维观测过程。
    if (samples < 5)
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Static-noise calibration needs at least five samples");

    // 每次自动对焦都从相同轴位置启动；该位置单位为um，getPoint输出单位为nm。
    double focusStartUm = 0.0;
    if (!motionActor_->getCommandedPosition(focusStartUm))
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Failed to read autofocus start position");

    MHEC::CalibrationData log;
    cv::Mat values(samples, 3, CV_64F);
    for (int i = 0; i < samples; ++i) {
        if (!motionActor_->setTargetPosition(focusStartUm))
            throw MAR::RobotException(MAR::MARInternalError,
                                      "Failed to restore autofocus start position");
        QThread::msleep(1000);
        cv::Vec3d p;
        getPoint(p, scale); // includes an independent autofocus cycle
        log.all.P_c_curr_list.push_back(p);
        for (int j = 0; j < 3; ++j) values.at<double>(i, j) = p[j];
        qInfo() << QString("[10X noise] %1/%2: (%3, %4, %5)")
                       .arg(i + 1).arg(samples).arg(p[0]).arg(p[1]).arg(p[2]);
    }
    cv::Mat mean, rawCovariance;
    cv::calcCovarMatrix(values, rawCovariance, mean,
                        cv::COVAR_NORMAL | cv::COVAR_ROWS | cv::COVAR_SCALE,
                        CV_64F);

    // 对相邻样本作差。若单次观测相互独立，则
    // Cov(p_i-p_{i-1})=2*Sigma_p，故差分样本协方差除以2得到Sigma_p。
    cv::Mat differences(samples - 1, 3, CV_64F);
    for (int i = 1; i < samples; ++i) {
        const cv::Mat difference = values.row(i) - values.row(i - 1);
        difference.copyTo(differences.row(i - 1));
    }
    cv::Mat differenceMean;
    cv::reduce(differences, differenceMean, 0, cv::REDUCE_AVG, CV_64F);
    const cv::Mat centeredDifferences =
        differences - cv::repeat(differenceMean, differences.rows, 1);
    const cv::Mat differenceCovariance =
        centeredDifferences.t() * centeredDifferences /
        static_cast<double>(differences.rows - 1);

    // 与论文相同，只保留三个坐标轴的方差，轴间协方差置零。
    cv::Mat covariance = cv::Mat::zeros(3, 3, CV_64F);
    for (int axis = 0; axis < 3; ++axis)
        covariance.at<double>(axis, axis) =
            0.5 * differenceCovariance.at<double>(axis, axis);

    MHEC::saveCalbrationData(log, "10X_StaticNoise");
    qInfo() << QString("10X raw covariance diagonal, including drift (nm^2): [%1, %2, %3]")
                   .arg(rawCovariance.at<double>(0, 0))
                   .arg(rawCovariance.at<double>(1, 1))
                   .arg(rawCovariance.at<double>(2, 2));
    qInfo() << QString("10X adjacent-difference observation covariance Sigma_p (nm^2): [%1, %2, %3]")
                   .arg(covariance.at<double>(0, 0))
                   .arg(covariance.at<double>(1, 1))
                   .arg(covariance.at<double>(2, 2));
    return covariance;
}

void MHECTask::run10XExperiment()
{
    // 10X正式实验：先标定图像比例，再采集50个静态点估计观测协方差；
    // 随后完成8轮比较，每种方法每轮使用20个标定运动和同一组10个验证运动。
    // 三种方法均在前5次只收集数据，第6次进行首次完整更新。
    MHEC::CalibrationCfg cfg;
    cfg.max_iter = 20;
    cfg.validation_motions = 10;
    cfg.repetitions = 8;
    // 与论文10X实验一致：标定激励为150 um和1 deg。
    cfg.step_len_R = 1.0 * M_PI / 180.0;
    cfg.step_len_t = 150000.0;
    cfg.decay_gain = 0.03;
    cfg.state_length_scale = cfg.step_len_t / cfg.step_len_R;
    // 11.345只是三维高斯创新的理论起点。首次无异常值试运行时可将门控关闭，
    // 程序仍会输出每一步NIS；根据实测分布确认后再用于正式实验。
    // 本次短测试先关闭硬拒绝，但updateIEKF仍会逐步输出NIS。
    // 这样可以判断递推本身是否收敛，并据实测创新分布重新确定门控模型。
    cfg.innovation_gate_enabled = false;
    // Physical nominal-data experiment: retain the NIS log but do not reject
    // measurements using the static-noise-only chi-square model.
    cfg.innovation_gate_enabled = false;
    cfg.innovation_gate_threshold = 11.345;
    // 默认采用论文中的三轴独立假设；若静态实验表明轴间相关性不可忽略，再切换为true。
    cfg.physical_use_full_measurement_covariance = false;
    // 相对残差中的前一帧点噪声会被本次旋转传播，不能在各向异性条件下
    // 简化成固定的2*Sigma。
    cfg.iekf_propagate_point_covariance = true;

    // 历史标定结果仅作为三种方法共同的控制与估计初值；
    // 三种方法不得使用彼此的最终结果进行热启动。
    const cv::Mat T_cs_initial = (cv::Mat_<double>(4, 4) <<
        0.14244521,  0.98970088,  0.01419637, -9333534.28189423,
        0.98971872, -0.14223166, -0.01506680,  6975481.85648128,
       -0.01289245,  0.01619661, -0.99978570, 15358892.01894797,
        0.0,         0.0,         0.0,                1.0);

    // 更换10X物镜后重新标定比例，不复用历史结果。
    double measuredScale = 0.0;
    cv::Mat unusedScalePose = cv::Mat::eye(4, 4, CV_64F);
    getScale(measuredScale, unusedScalePose);
    const double expected10XScale = 338.640; // 仅作量级检查，后续使用实测比例
    if (measuredScale <= 0.0 || !std::isfinite(measuredScale) ||
        measuredScale < 0.6 * expected10XScale ||
        measuredScale > 1.5 * expected10XScale)
        throw MAR::RobotException(MAR::MARInternalError, "Invalid 10X scale calibration");
    scale_ = measuredScale;
    T_cs_est_ = T_cs_initial.clone();
    qInfo() << "10X calibrated scale (nm/pixel):" << scale_;

    // 正式标定前测量静止特征点的三轴观测协方差；每个样本都包含一次完整自动对焦。
    const cv::Mat pointCov = runStaticNoise10X(50, scale_);
    if (pointCov.rows != 3 || pointCov.cols != 3 || pointCov.type() != CV_64F ||
        !cv::checkRange(pointCov) || pointCov.at<double>(0, 0) <= 0.0 ||
        pointCov.at<double>(1, 1) <= 0.0 || pointCov.at<double>(2, 2) <= 0.0) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid 10X static observation covariance");
    }
    const char* methodNames[3] = {"PO-LS", "J-FGD", "J-RIEKF"};

    // ---------- 正式8轮对比实验的共同初值 ----------
    // 先单独运行一次完整的PO-LS粗标定。这里仍从历史矩阵出发规划运动，
    // 但粗标定结果只用于生成共同初值，不计入后续任何一种方法的统计结果。
    runTag_ = "10X_CommonInitial_PO-LS_";
    cv::Mat T_sc_common_initial = T_cs_initial.inv();
    MHEC::CalibrationState coarseState;
    MHEC::CalibrationCfg coarseCfg = cfg;
    coarseCfg.max_iter = 20;
    runYKronT(coarseCfg, coarseState, T_sc_common_initial, scale_);

    // 在使用粗标定结果之前做基本合法性检查，避免无效矩阵进入全部正式实验。
    if (!isValidRigidTransform(T_sc_common_initial))
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid PO-LS common initial calibration");

    // 使用与粗标定数据不重合的20个运动验证共同初值，并单独保存验证数据。
    // 验证只评估该矩阵，不进行任何状态更新，也不会改变共同初值。
    MHEC::CalibrationCfg initialValidation = cfg;
    initialValidation.max_iter = initialValidation.validation_motions;
    initialValidation.excitation_sequence_offset = coarseCfg.max_iter;
    // cv::Mat::inv() 返回 cv::MatExpr；先求值为 Mat，再保存独立副本。
    const cv::Mat T_cs_common_initial = T_sc_common_initial.inv();
    T_cs_est_ = T_cs_common_initial.clone();
    const double commonInitialRmse =
        runTest("10X_CommonInitial_Validation_", initialValidation,
                T_sc_common_initial, scale_, pointCov);

    // ---------- 在已验证的粗标定结果上加入统一的已知初始误差 ----------
    // 误差方向固定且不与单一坐标轴重合，避免每轮随机方向造成额外实验变量。
    // 旋转和平移分别按向量模长定义，因此实际加入的误差恰为5 deg和2 mm，
    // 而不是三个坐标分量各自加入5 deg或2 mm。
    cv::Mat T_cs_experiment_initial = T_sc_common_initial.inv();
    cv::Vec3d rotationAxis(1.0, 1.0, 1.0);
    rotationAxis /= cv::norm(rotationAxis);
    cv::Vec3d translationDirection(1.0, -1.0, 1.0);
    translationDirection /= cv::norm(translationDirection);

    cv::Mat rotationVector = (cv::Mat_<double>(3, 1) <<
        rotationAxis[0] * 5.0 * M_PI / 180.0,
        rotationAxis[1] * 5.0 * M_PI / 180.0,
        rotationAxis[2] * 5.0 * M_PI / 180.0);
    cv::Mat R_error;
    cv::Rodrigues(rotationVector, R_error);
    cv::Mat R_cs_perturbed = R_error * T_cs_experiment_initial(cv::Rect(0, 0, 3, 3));
    R_cs_perturbed.copyTo(T_cs_experiment_initial(cv::Rect(0, 0, 3, 3)));

    const double translationErrorNm = 2.0e6; // 2 mm，程序内部位移单位为nm
    for (int axis = 0; axis < 3; ++axis)
        T_cs_experiment_initial.at<double>(axis, 3) +=
            translationDirection[axis] * translationErrorNm;

    // 后续运动规划与三种估计方法都使用同一个扰动后初值。
    T_cs_est_ = T_cs_experiment_initial.clone();
    cv::Mat T_sc_perturbed_initial = T_cs_experiment_initial.inv();
    if (!isValidRigidTransform(T_sc_perturbed_initial))
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid perturbed common initial calibration");
    qInfo() << "Common initial perturbation: rotation = 5 deg, translation = 2 mm";

    // 对扰动后的共同初值再次进行独立验证。这里复用与扰动前完全相同的
    // 20个验证方向，但重新采集观测，因此可以直接比较扰动前后的误差变化。
    // runTest只计算并保存验证误差，不会更新T_sc_perturbed_initial。
    const double perturbedInitialRmse =
        runTest("10X_PerturbedInitial_Validation_", initialValidation,
                T_sc_perturbed_initial, scale_, pointCov);
    qInfo() << "Initial validation RMSE before/after perturbation (nm):"
            << commonInitialRmse << perturbedInitialRmse;
    if (!std::isfinite(commonInitialRmse) || !std::isfinite(perturbedInitialRmse) ||
        perturbedInitialRmse <= commonInitialRmse) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Initial validation did not respond to perturbation");
    }

    // 后续8轮中的24次标定均从该扰动矩阵的完全相同副本开始。
    const cv::Mat T_sc_experiment_initial = T_sc_perturbed_initial.clone();

    // 每轮分别完成三种20步标定，随后执行一次10步公共验证。
    for (int round = 0; round < cfg.repetitions; ++round) {
        std::vector<cv::Mat> roundCandidates(3);
        for (int slot = 0; slot < 3; ++slot) {
            // 初值验证完成后首先运行J-RIEKF，以便尽早检查实物NIS和门控拒绝情况；
            // 后续仍按轮次循环改变方法顺序，减弱设备随时间漂移造成的顺序偏差。
            // 前三轮顺序分别为：RIEKF/PO/FGD、PO/FGD/RIEKF、FGD/RIEKF/PO。
            const int method = (round + slot + 2) % 3;
            runTag_ = QString("10X_R%1_%2_").arg(round + 1, 2, 10, QChar('0')).arg(methodNames[method]);
            // 每种方法都从“已验证PO-LS结果+统一5 deg/2 mm扰动”开始，
            // 避免前一种方法的最终结果泄漏到后一种方法。
            cv::Mat T_sc = T_sc_experiment_initial.clone();

            if (method == 0) {
                // PO-LS：保留本轮全部绝对位置观测并累计求解。
                MHEC::CalibrationState state;
                runYKronT(cfg, state, T_sc, scale_);
            } else if (method == 1) {
                // J-FGD：第6次采用堆叠初始化，之后固定增益递推。
                MHEC::CalibrationState state;
                state.all.Lambda = cfg.decay_gain;
                runServoOnce(cfg, state, T_sc, scale_);
            } else {
                // J-RIEKF：残差由前后两次近似独立的点观测之差构成，
                // 因而按论文模型使用V=2*Sigma。若实际观测存在时间相关性，
                // 后续应直接用静态差分序列标定V，而不能机械沿用2倍关系。
                MHEC::CalibrationState state;
                cv::Mat measurementCovariance = cv::Mat::zeros(3, 3, CV_64F);
                if (cfg.physical_use_full_measurement_covariance) {
                    // 完整协方差保留自动对焦与图像定位引起的轴间相关项。
                    measurementCovariance = 0.5 * (pointCov + pointCov.t());
                } else {
                    // 与论文当前的独立三轴噪声假设保持一致，只采用实测方差。
                    for (int axis = 0; axis < 3; ++axis)
                        measurementCovariance.at<double>(axis, axis) = pointCov.at<double>(axis, axis);
                }
                // 1 nm^2仅用于防止完全零方差导致数值奇异，相对实测噪声可忽略。
                // 保存单次点观测协方差Sigma；updateIEKF逐运动构造
                // V_k = Sigma + DeltaR_k*Sigma*DeltaR_k^T。
                state.all.KF.R = measurementCovariance.clone();
                // 单轮实验中手眼矩阵为常量。Q描述状态自身的漂移，
                // 不是PI平台的运动误差，因此实物标定先采用零过程噪声。
                state.all.KF.Q = cv::Mat::zeros(6, 6, CV_64F);

                // P0表示对共同PO-LS初值的不确定性，而不是某一次验证残差本身。
                // 当前采用显式且偏保守的每轴先验标准差，不再使用仿真的1/3换算。
                // 若以后重复执行多次粗标定，可再用其六维重复性统计替换这两个先验值。
                const double initialRotationStd = cfg.physical_initial_rotation_std;
                const double initialTranslationStd = cfg.physical_initial_translation_std;
                state.all.KF.P = cv::Mat::zeros(6, 6, CV_64F);
                state.all.KF.P(cv::Range(0, 3), cv::Range(0, 3)) = cv::Mat::eye(3, 3, CV_64F) * initialRotationStd * initialRotationStd;
                state.all.KF.P(cv::Range(3, 6), cv::Range(3, 6)) = cv::Mat::eye(3, 3, CV_64F) * initialTranslationStd * initialTranslationStd;
                runIEKF(cfg, state, T_sc, scale_);
            }

            // 方法内部若出现NaN、镜像旋转或严重失去正交性，立即停止，
            // 避免继续执行20次无意义验证和剩余长时间实验。
            if (!isValidRigidTransform(T_sc))
                throw MAR::RobotException(MAR::MARInternalError,
                                          "Calibration method returned invalid transform");

            roundCandidates[method] = T_sc.clone();

        }

        MHEC::CalibrationCfg sharedValidation = cfg;
        sharedValidation.max_iter = cfg.validation_motions;
        sharedValidation.excitation_sequence_offset = cfg.max_iter;
        // 与论文10X实验一致：验证激励为225 um和1.5 deg。
        sharedValidation.step_len_R = 1.5 * M_PI / 180.0;
        sharedValidation.step_len_t = 225000.0;
        sharedValidation.state_length_scale =
            sharedValidation.step_len_t / sharedValidation.step_len_R;
        const std::vector<std::string> sharedMethodNames = {
            methodNames[0], methodNames[1], methodNames[2]};
        const QString sharedRoundTag =
            QString("10X_R%1").arg(round + 1, 2, 10, QChar('0'));
        runSharedValidation10X(sharedValidation, roundCandidates,
                             sharedMethodNames, scale_, pointCov,
                             sharedRoundTag);
    }
    runTag_.clear();
}

void MHECTask::getPoint(cv::Vec3d& Point, double scale) {

    if (imgWidth_ == 0 || imgHigh_ == 0) {
        this->cameraControl_->getImgSize(imgWidth_, imgHigh_);
        qDebug() << "imgWidth_" << imgWidth_ << "imgHigh_" << imgHigh_;
    }
    if (imgWidth_ <= 0 || imgHigh_ <= 0 || !std::isfinite(scale) || scale <= 0.0) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid image size or image scale");
    }
    this->motionActor_->getPoint(Point, scale);
    Point[0] = (Point[0] - imgWidth_/2) * scale;
    Point[1] = (Point[1] - imgHigh_/2) * scale;
    if (!std::isfinite(Point[0]) || !std::isfinite(Point[1]) ||
        !std::isfinite(Point[2])) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Invalid autofocus observation");
    }
}

void MHECTask::gotoPoint(cv::Vec3d Point, cv::Mat twist_c) {
    if (imgWidth_ == 0 || imgHigh_ == 0) {
        this->cameraControl_->getImgSize(imgWidth_, imgHigh_);
        qDebug() << "imgWidth_" << imgWidth_ << "imgHigh_" << imgHigh_;
    }

    double X_c = Point[0];
    double Y_c = Point[1];
    double distance = 0.0;

    if (twist_c.rows == 6 && twist_c.cols == 1) {
        double w_x = twist_c.at<double>(0, 0);
        double w_y = twist_c.at<double>(1, 0);
        double v_z = twist_c.at<double>(5, 0);
        distance = (- w_x * Y_c + w_y * X_c - v_z) / 1000;
    } else {
        qWarning() << "Invalid twist_c dimensions for relative distance calculation!";
    }

    if (!this->motionActor_->setRelativePosition(distance)) {
        throw MAR::RobotException(MAR::MARInternalError,
                                  "Autofocus-axis relative command failed");
    }
}

void MHECTask::getTargetPoint(cv::Vec3d& P_c_pred, cv::Vec3d P_c_prev, cv::Mat twist_c, double scale)
{
    cv::Vec3d w(twist_c.at<double>(0), twist_c.at<double>(1), twist_c.at<double>(2));
    cv::Vec3d v(twist_c.at<double>(3), twist_c.at<double>(4), twist_c.at<double>(5));

    cv::Mat dT = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(twist_c))));

    cv::Mat P_c_prev_homo = (cv::Mat_<double>(4, 1) << P_c_prev[0], P_c_prev[1], P_c_prev[2], 1.0);
    cv::Mat P_c_pred_homo = dT * P_c_prev_homo;

    cv::Vec3d P_c_pred_temp(P_c_pred_homo.at<double>(0),
                            P_c_pred_homo.at<double>(1),
                            P_c_pred_homo.at<double>(2));

    P_c_pred = P_c_pred_temp;

    cv::Vec3d circle(P_c_pred_homo.at<double>(0)/scale + imgWidth_/2.0,
                     P_c_pred_homo.at<double>(1)/scale + imgHigh_/2.0,
                     50000/scale);

    emit sigTargetCircle(circle);
}


cv::Vec3d MHECTask::printPoseInformation(const cv::Mat& T) {
    if (T.empty() || T.rows != 4 || T.cols != 4) {
        qDebug() << "Error: Invalid Transformation Matrix!";
        return cv::Vec3d(0, 0, 0);
    }

    QString matrixStr = "\n[";
    for (int i = 0; i < T.rows; ++i) {
        for (int j = 0; j < T.cols; ++j) {
            // 使用 QString::number 控制精度，并用 \t 制表符对齐
            matrixStr += QString::number(T.at<double>(i, j), 'f', 8) + ",\t\t\t";
        }
        matrixStr += "\n";
    }
    matrixStr += "]";
    qInfo() << "T_cs_est Details:" << qPrintable(matrixStr);

    // --- 1. 提取平移部分 ---
    double tx = T.at<double>(0, 3);
    double ty = T.at<double>(1, 3);
    double tz = T.at<double>(2, 3);

    // --- 2. 提取旋转矩阵并计算欧拉角 (ZYX 顺规) ---
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));

    double roll, pitch, yaw;
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                          R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // 检查万向节锁

    if (!singular) {
        roll  = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        yaw   = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        roll  = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        pitch = std::atan2(-R.at<double>(2, 0), sy);
        yaw   = 0;
    }

    // 弧度转角度
    roll  *= (180.0 / CV_PI);
    pitch *= (180.0 / CV_PI);
    yaw   *= (180.0 / CV_PI);

    // --- 3. 打印输出 ---
    QString line = "---------------------------------------------";
    qDebug() << qPrintable(line);
    qDebug() << "POSE ESTIMATION REPORT:";
    qDebug() << "Position (mm): "
             << "X:" << QString::number(tx, 'f', 2)
             << "Y:" << QString::number(ty, 'f', 2)
             << "Z:" << QString::number(tz, 'f', 2);
    qDebug() << "Orientation (deg):"
             << "Roll:"  << QString::number(roll, 'f', 2)
             << "Pitch:" << QString::number(pitch, 'f', 2)
             << "Yaw:"   << QString::number(yaw, 'f', 2);
    qDebug() << qPrintable(line);

    return cv::Vec3d(roll, pitch, yaw);
}
