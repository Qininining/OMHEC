#ifndef PARALLELPLATFORM_H
#define PARALLELPLATFORM_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include <QString>
#include <Eigen/Dense>
#include <QTimer>
#include <QThread>
#include <QMutexLocker>
#include <QRecursiveMutex>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include <windows.h>
#include "PI_GCS2_DLL_PF.h"
#include <opencv2/opencv.hpp>

#define ROBOT_VELOCITY              0
#define ROBOT_POSITION              1
#define ROBOT_STOP                  2
#define ROBOT_MOVE_TO_ZERO          3

class ParallelPlatform:  public QObject
{
    Q_OBJECT // 必须包含这个宏

public:
    ParallelPlatform(double hz);
    ~ParallelPlatform();

public:
    /// 初始化函数
    bool connect();
    bool disconnect();
    bool startupdate();
    bool stopupdate();
    bool findReferance();   // 使得各个关节轴回到标志位
    // 标准 Pivot 准备流程：等待停止 -> 六轴回零 -> 等待停止 -> 写入并回读 Pivot。
    bool preparePivotForMotion(int timeoutMs = 30000);

    /// 功能性函数
    bool stop();
    // 单位nm
    bool moveAbsolute(signed int x,signed int y,signed int z,signed int rx,signed int ry,signed int rz);
    /// 自己通过运算写的函数
    //  输入矩阵T移动,自动转化为需求的向量
    bool moveAbsolute(Eigen::Matrix4d T);
    // 给定Δt和速度方向,通过位置控制移动到指定位置
    bool moveVelocity(Eigen::VectorXd VelocityList_b,double dt);

    /// 状态读取函数
    // bool getSpeed();
    bool getJointPositions(Eigen::VectorXd& positions);
    bool getTaskPositions(Eigen::VectorXd& positions);
    bool getTargetVelocity(Eigen::VectorXd& targetVelocity);
    bool getTaskMat(Eigen::Matrix4d& T);
    cv::Mat getTaskMat_cv();
    bool checkresult();// 用于判断函数释放顺利执行

    // 检查是否能够到达
    bool checkPoseReachable(int x,int y,int z,int rx,int ry,int rz);
    bool setPivot(signed int x, signed int y, signed int z);
    Eigen::Matrix4d getT(Eigen::VectorXd Tlist);
    Eigen::VectorXd getList(Eigen::Matrix4d T);
    bool setTargetVelocity(Eigen::VectorXd V);
    bool setTargetPose(Eigen::VectorXd P);
    void setRobotMode(int mode);

signals:
    void sig_updateRobotData(const  Eigen::VectorXd current_positions, const Eigen::VectorXd target_velocity, const Eigen::VectorXd current_pose);

private slots:
    void controller();
    void update();  // 上传x，y，z，rx，ry，rz的位置

private:
    // MHEC 任务线程与状态定时器可能同时访问控制器，PI GCS2 命令必须串行执行。
    mutable QRecursiveMutex piMutex_;

    // PI GCS2 dynamic loading follows PI's DynamicLoadSample and uses
    // the official function-pointer declarations from PI_GCS2_DLL_PF.h.
    using PI_BOOL = BOOL;

    bool loadPiDll();
    void unloadPiDll();
    bool resolvePiFunction(FARPROC& target, const char* name);
    bool openPiConnection();
    bool refreshPiPose();
    bool queryPiIdentification();
    bool verifyPiConfiguration();
    bool queryAllAxesReferenced(bool& allReferenced);
    bool queryPiMoving(bool& anyMoving);
    bool waitForPiMotionDone(int timeoutMs = 30000);
    bool setPiServo(bool enabled);
    bool updatePiError(bool ok, const char* commandName);
    QString describePiError() const;
    void syncCachedPoseFromCurrentVector();

    HMODULE piDll_ = nullptr;
    int piId_ = -1;
    bool piConnected_ = false;
    int piLastError_ = 0;
    QString piDllPath_;
    QString piControllerType_ = "C-887";
    QString piHost_ = "127.0.0.1";
    int piPort_ = 50000;
    bool piUseTcp_ = false;
    bool piUseUsb_ = false;
    // MHEC 的 Pivot 编译期开关：
    // true = preparePivotForMotion() 写入下方标定 Pivot；
    // false = preparePivotForMotion() 明确写入 0,0,0，避免沿用控制器残留 Pivot。
    // connect()/findReferance() 只建立连接和确认参考状态，不会直接写 Pivot。
    // bool piApplyPivotOnConnect_ = true;
    bool piApplyPivotOnConnect_ = false;

    bool piConfigurationVerified_ = false;
    bool piMotionArmed_ = false;
    bool piPivotPrepared_ = false;
    bool piPivotPreparationInProgress_ = false;
    bool piAllowRetargetWhileMoving_ = false;
    bool piDisableServoOnDisconnect_ = false;
    // C-887.52 的 qIDN 仅返回系列名 "C-887"，不包含铭牌后缀 ".52"。
    QString piExpectedController_ = "C-887";
    QString piExpectedStage_ = "H-811.S2IHP";
    QString piControllerIdentification_;
    QString piStageConfiguration_;

    // Pivot 坐标单位为 nm。开启 Pivot 前，在这里填写标定得到的 P_oe。
    double piDefaultPivotXNm_ = -19003941.9223;
    double piDefaultPivotYNm_ = 2127019.2161 ;
    double piDefaultPivotZNm_ = 14092489.8335;

    PFPI_InterfaceSetupDlg pi_InterfaceSetupDlg_ = nullptr;
    PFPI_ConnectTCPIP pi_ConnectTCPIP_ = nullptr;
    PFPI_EnumerateUSB pi_EnumerateUSB_ = nullptr;
    PFPI_ConnectUSB pi_ConnectUSB_ = nullptr;
    PFPI_CloseConnection pi_CloseConnection_ = nullptr;
    PFPI_qIDN pi_qIDN_ = nullptr;
    PFPI_qCST pi_qCST_ = nullptr;
    PFPI_GetError pi_GetError_ = nullptr;
    PFPI_TranslateError pi_TranslateError_ = nullptr;
    PFPI_MOV pi_MOV_ = nullptr;
    PFPI_qPOS pi_qPOS_ = nullptr;
    PFPI_IsMoving pi_IsMoving_ = nullptr;
    PFPI_STP pi_STP_ = nullptr;
    PFPI_StopAll pi_StopAll_ = nullptr;
    PFPI_SVO pi_SVO_ = nullptr;
    PFPI_qSVO pi_qSVO_ = nullptr;
    PFPI_FRF pi_FRF_ = nullptr;
    PFPI_qFRF pi_qFRF_ = nullptr;
    PFPI_IsControllerReady pi_IsControllerReady_ = nullptr;
    PFPI_qTMN pi_qTMN_ = nullptr;
    PFPI_qTMX pi_qTMX_ = nullptr;
    PFPI_qTRA pi_qTRA_ = nullptr;
    PFPI_SPI pi_SPI_ = nullptr;
    PFPI_qSPI pi_qSPI_ = nullptr;

    bool result_ = false;
    Eigen::Matrix4d b_T_e_;                      //  矩阵 b_T_e 当前位姿
    Eigen::VectorXd current_end_effector_pose_; //  向量 [x;y;z;rx;ry;rz]
    // 以下变量储存轴的
    int positionx_;
    int positiony_;
    int positionz_;
    int rotationx_;
    int rotationy_;
    int rotationz_;
    Eigen::VectorXd currentPose_Vec_; //  向量 [x;y;z;rx;ry;rz],记录当前的指令的目标位置,注意：实际是编码器读数
    Eigen::Matrix4d currentPose_Mat_; //  矩阵 T,记录当前的指令的目标位置
    Eigen::VectorXd targetVelocity_;
    Eigen::VectorXd targetPose_; //  向量 [x;y;z;rx;ry;rz]
    double epsilon_w_;
    double epsilon_v_;
    double dt_;
    QTimer *m_preciseTimer_;
    QElapsedTimer cycleTimer;
    int controlMode_; // 0: velocity; 1: position; 2: stop; 3: moveToZero

};

#endif // ParallelPlatform_H
