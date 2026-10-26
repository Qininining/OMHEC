#include "CoarseAlignTask.h"

#include <QFinalState>
#include "AsyncTaskBridge.hpp"

CoarseAlignTask::CoarseAlignTask()
{
    staMachine_ =  new QStateMachine();
    connect(staMachine_, &QStateMachine::finished, this, &CoarseAlignTask::onFinished);
    PASS_("CoarseAlignTask created");
}
CoarseAlignTask::~CoarseAlignTask()
{
    if(staMachine_ && staMachine_->isRunning()) {
        staMachine_->stop();
    }

    delete staMachine_;
    PASS_("CoarseAlignTask destroyed");
}

void CoarseAlignTask::setup(ManipulatorActor* manipulatorActor)
{
    manipulatorActor_ = manipulatorActor;
    initWorkflow();
}

void CoarseAlignTask::onFinished()
{
    HIGHLIGHT_("[CoarseAlignTask] Coarse alignment task Finished at thread ID: %d", QThread::currentThreadId());
    emit sigCoarseAlignFinished(true);
}

void CoarseAlignTask::stepFinished(bool success, const QString &errorMsg)
{
    if (!staMachine_->isRunning()) return;

    if (!success) {
        ERROR_("[CoarseAlignTask] Step failed: %s", errorMsg.toStdString().c_str());
        stop();
    }
}

void CoarseAlignTask::initWorkflow()
{
    if (!staMachine_) {
        ERROR_("State machine not initialized");
        throw MAR::RobotException(MAR::MARInternalError, "State machine not initialized");
    }
    if (!manipulatorActor_) {
        ERROR_("ManipulatorActor not set up");
        throw MAR::RobotException(MAR::MARNatorNotInitialized, "ManipulatorActor not set up");
    }
    // 1. 初始化状态机
    QState *s1 = new QState(staMachine_);
    QState *s2 = new QState(staMachine_);
    QState *s3 = new QState(staMachine_);
    QFinalState *finalState = new QFinalState(staMachine_);

    // 2. 创建异步任务承载桥
    AsyncTaskBridge *bridge1 = new AsyncTaskBridge(this);
    AsyncTaskBridge *bridge2 = new AsyncTaskBridge(this);
    AsyncTaskBridge *bridge3 = new AsyncTaskBridge(this);

    // 3. 绑定逻辑与状态
    bridge1->bindState(s1, [this]() {
        // qDebug() << "执行任务 1..." << "当前线程ID:" << QThread::currentThreadId();
        HIGHLIGHT_("[CoarseAlignTask] Task 1 executing at thread ID: %d", QThread::currentThreadId());
        this->manipulatorActor_->place(Eigen::Vector3d(0, 0, 50));
        QThread::msleep(1000); // 模拟耗时操作
    });

    bridge2->bindState(s2, [this]() {
        // qDebug() << "执行任务 2..." << "当前线程ID:" << QThread::currentThreadId();
        HIGHLIGHT_("[CoarseAlignTask] Task 2 executing at thread ID: %d", QThread::currentThreadId());
        this->manipulatorActor_->moveTo(Eigen::Vector3d(100, 0, 50));
        QThread::msleep(800);
    });

    bridge3->bindState(s3, [this]() {
        // qDebug() << "执行任务 3..." << "当前线程ID:" << QThread::currentThreadId();
        HIGHLIGHT_("[CoarseAlignTask] Task 3 executing at thread ID: %d", QThread::currentThreadId());
        this->manipulatorActor_->pick(Eigen::Vector3d(100, 0, 50), 30);
        QThread::msleep(1200);
    });

    // 4. 配置状态机跳转：任务结束信号驱动跳转
    s1->addTransition(bridge1, &AsyncTaskBridge::taskFinished, s2);
    s2->addTransition(bridge2, &AsyncTaskBridge::taskFinished, s3);
    s3->addTransition(bridge3, &AsyncTaskBridge::taskFinished, finalState);

    connect(bridge1, &AsyncTaskBridge::taskFinished, this, &CoarseAlignTask::stepFinished);
    connect(bridge2, &AsyncTaskBridge::taskFinished, this, &CoarseAlignTask::stepFinished);
    connect(bridge3, &AsyncTaskBridge::taskFinished, this, &CoarseAlignTask::stepFinished);

    // 5. 设置初始状态
    staMachine_->setInitialState(s1);

}

void CoarseAlignTask::startCoarseAlignment()
{
    // qDebug() << "开始粗略对齐任务..." << "当前线程ID:" << QThread::currentThreadId();
    HIGHLIGHT_("[CoarseAlignTask] Coarse alignment task Started at thread ID: %d", QThread::currentThreadId());
    if (!staMachine_) {
        ERROR_("State machine not initialized");
        throw MAR::RobotException(MAR::MARInternalError, "State machine not initialized");
    }
    if (!manipulatorActor_) {
        ERROR_("ManipulatorActor not set up");
        throw MAR::RobotException(MAR::MARNatorNotInitialized, "ManipulatorActor not set up");
    }

    staMachine_->start();
}

void CoarseAlignTask::stop()
{
    if (staMachine_ && staMachine_->isRunning()) {
        HIGHLIGHT_("[CoarseAlignTask] Stopping task manually...");
        staMachine_->stop();
        emit sigCoarseAlignFinished(false);
    }
}



