#ifndef MANIPULATORACTOR_H
#define MANIPULATORACTOR_H

#include "Manipulator.h"

class ManipulatorActor
{
public:
    ManipulatorActor();
    ~ManipulatorActor();

    void initialize(const std::string& userId);
    void shutdown();
    
    // Recovery
    void reset();

    //  ********* task *********
    void pick(Eigen::Vector3d position, double width);//宽度
    void place(Eigen::Vector3d position);

    // ********* base functions *********
    void findReference();
    void waitForMoveComplete();
    void moveTo(const Eigen::Vector3d position);
    void open(double width);
    void grip();
    void release();

private:
    // ************* status functions **************
    void setInitialized(bool initialized);
    bool getInitialized() const;

    void setBusy(bool busy);
    bool getBusy() const;

    bool checkStatus() const;

    void checkAutomaticStatus() const;

    class BusyGuard {
    public:
        BusyGuard(ManipulatorActor* actor);
        ~BusyGuard();
        // Prevent copying
        BusyGuard(const BusyGuard&) = delete;
        BusyGuard& operator=(const BusyGuard&) = delete;
    private:
        ManipulatorActor* m_actor;
    };


private:
    Manipulator * manipulator;

    bool isInitialized = false;
    bool isBusy = false;

};

#endif // MANIPULATORACTOR_H
