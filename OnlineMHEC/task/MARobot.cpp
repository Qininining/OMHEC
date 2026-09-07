#include "MARobot.h"
#include "RobotException.hpp"
MARobot::MARobot()
{
    robot_ = new ParallelPlatform(30.0);
    robotThread_ = new QThread();
    robot_->moveToThread(robotThread_);
    robotThread_->start();

    camera_ = new BaslerCameraControl(30); // 实际 release 下仅有 21 HZ
    cameraThread_ = new QThread();
    camera_->moveToThread(cameraThread_);
    cameraThread_->start();

    motionActor_ = new AgeMotionActor(30);

    mhecTask_ = new MHECTask();
    mhecTask_->setup(robot_, motionActor_, camera_);

    initializeSystem();
}

MARobot::~MARobot()
{
    delete motionActor_;

    if (robotThread_) {
        robotThread_->quit();
        robotThread_->wait();
        delete robotThread_;
        robotThread_ = nullptr;
    }
    if (robot_) {
        delete robot_;
        robot_ = nullptr;
    }

    if (cameraThread_) {
        cameraThread_->quit();
        cameraThread_->wait();
        delete cameraThread_;
        cameraThread_ = nullptr;
    }
    if (camera_) {
        delete camera_;
        camera_ = nullptr;
    }
}

bool MARobot::initializeSystem()
{
    // 初始化相机
    if(!this->camera_->init()) {
        ERROR_("相机打开失败");
        return false;
    }
    // 连接机器人
    if(!this->robot_->connect()) {
        ERROR_("机器人打开失败");
        return false;
    }
    if (!this->motionActor_->connect()) {
        ERROR_("显微镜滑台打开失败");
        return false;
    }
    this->motionActor_->setup(this->camera_);
    HIGHLIGHT_("MARobot 初始化成功");
    return true;
}

void MARobot::startMHECTask()
{
    if (mhecTask_) {
        mhecTask_->start();
    }
}
void MARobot::stopMHECTask()
{
    if (mhecTask_) {
        mhecTask_->stop();
    }
}
