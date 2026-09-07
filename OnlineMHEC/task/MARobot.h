#ifndef MAROBOT_H
#define MAROBOT_H

#include <QThread>

#include "ParallelPlatform.h"
#include "BaslerCameraControl.h"
#include "AgeMotionActor.h"

#include "MHECTask.h"

class MARobot
{
public:
    MARobot();
    ~MARobot();

    bool initializeSystem();

    void startMHECTask();
    void stopMHECTask();

public:
    ParallelPlatform* robot_;
    QThread* robotThread_;

    BaslerCameraControl* camera_;
    QThread* cameraThread_;

    AgeMotionActor* motionActor_;
    QThread* motionActorThread_;

    MHECTask* mhecTask_;

};


#endif // MAROBOT_H
