#pragma once

#ifndef ASYNCTASKBRIDGE_H
#define ASYNCTASKBRIDGE_H

#include <QObject>
#include <QRunnable>
#include <QThreadPool>
#include <QThread>
#include <functional>
#include <atomic>
#include <QStateMachine>
#include "RobotException.hpp"

class AsyncTaskBridge : public QObject
{
    Q_OBJECT
    std::atomic<int> m_activeCount{0};
public:
    explicit AsyncTaskBridge(QObject *parent = nullptr) : QObject(parent) {}
    ~AsyncTaskBridge() {
        while (m_activeCount.load() > 0) {
            QThread::msleep(10);
        }
    }

    // 核心方法：绑定一个状态，并定义该状态下要执行的任务
    void bindState(QState *state, std::function<void()> taskLogic) {
        if (!state) return;

        // 当进入该状态时，触发执行
        connect(state, &QState::entered, this, [this, taskLogic]() {
            execute(taskLogic);
        });
    }

private:
    void execute(std::function<void()> taskLogic) {
        m_activeCount++;
        // 使用 QRunnable::create 快速创建任务，无需手动新建类
        QRunnable *runnable = QRunnable::create([this, taskLogic]() {
            bool isSuccess = true;
            QString errorMsg;
            if (taskLogic) {
                try {
                    taskLogic();
                } catch (const std::exception &e) {
                    isSuccess = false;
                    errorMsg = QString::fromStdString(e.what());
                    ERROR_("AsyncTaskBridge caught exception: %s", e.what());
                }
            }
            // 任务执行完后，回到主线程发射完成信号（确保状态机跳转在主线程安全执行）
            emit taskFinished(isSuccess, errorMsg);
            PASS_("[finished] on thread ID: %d", QThread::currentThreadId());
            m_activeCount--;
        });

        QThreadPool::globalInstance()->start(runnable);
    }

signals:
    // 任务完成信号，用于状态机的 Transition
    void taskFinished(bool isSuccess, const QString &errorMsg = QString());
};

#endif // ASYNCTASKBRIDGE_H
