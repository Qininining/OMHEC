#ifndef COARSEALIGNTASK_H
#define COARSEALIGNTASK_H

#include <QObject>
#include <QStateMachine>

#include "ManipulatorActor.h"

class CoarseAlignTask: public QObject
{
    Q_OBJECT

public:
    CoarseAlignTask();
    ~CoarseAlignTask();

    void setup(ManipulatorActor* manipulatorActor);

    void startCoarseAlignment();
    void stop();

public: signals:
    void sigCoarseAlignFinished(bool success = true);

private:
    void initWorkflow();

private slots:
    void onFinished();
    void stepFinished(bool success, const QString &errorMsg = QString());

private:
    QStateMachine* staMachine_;
    ManipulatorActor* manipulatorActor_ = nullptr;
};


#endif // COARSEALIGNTASK_H
