#include "ManipulatorActor.h"

ManipulatorActor::ManipulatorActor() {
    manipulator = new Manipulator();
    PASS_("ManipulatorActor created");
}

ManipulatorActor::~ManipulatorActor() {
    delete manipulator;
    PASS_("ManipulatorActor destroyed");
}

void ManipulatorActor::initialize(const std::string& userId) {
    manipulator->initialize(userId);
    this->setInitialized(true);
    this->setBusy(false);
    PASS_("ManipulatorActor initialized");
}

void ManipulatorActor::shutdown() {
    manipulator->shutdown();
    this->setInitialized(false);
    PASS_("ManipulatorActor shutdown");
}

void ManipulatorActor::pick(Eigen::Vector3d position, double width) {
    PASS_("ManipulatorActor pick started");
    checkAutomaticStatus();
    BusyGuard guard(this);
    manipulator->open(width);
    manipulator->waitForMoveComplete();
    manipulator->moveTo(position);
    manipulator->waitForMoveComplete();
    manipulator->grip();
    manipulator->waitForMoveComplete();
    PASS_("ManipulatorActor pick completed");
}

void ManipulatorActor::place(Eigen::Vector3d position) {
    PASS_("ManipulatorActor place started");
    checkAutomaticStatus();
    BusyGuard guard(this);
    manipulator->moveTo(position);
    manipulator->waitForMoveComplete();
    manipulator->release();
    manipulator->waitForMoveComplete();
    PASS_("ManipulatorActor place completed");
}

// ************* status functions **************
void ManipulatorActor::setInitialized(bool initialized) {
    isInitialized = initialized;
}
bool ManipulatorActor::getInitialized() const {
    return isInitialized;
}
void ManipulatorActor::setBusy(bool busy) {
    isBusy = busy;
}
bool ManipulatorActor::getBusy() const {
    return isBusy;
}
void ManipulatorActor::reset() {
    setBusy(false);
    PASS_("ManipulatorActor reset");
}
bool ManipulatorActor::checkStatus() const {
    return isInitialized && !isBusy;
}

void ManipulatorActor::checkAutomaticStatus() const {
    if (!isInitialized) {
        ERROR_("ManipulatorActor not initialized");
        throw MAR::RobotException(MAR::MARNatorNotInitialized, "Manipulator not initialized");
    }
    if (isBusy) {
        ERROR_("ManipulatorActor is busy");
        throw MAR::RobotException(MAR::MARBusy, "Manipulator is busy");
    }
}

ManipulatorActor::BusyGuard::BusyGuard(ManipulatorActor* actor) : m_actor(actor) {
    if (m_actor) {
        m_actor->setBusy(true);
    }
}

ManipulatorActor::BusyGuard::~BusyGuard() {
    if (m_actor) {
        m_actor->setBusy(false);
    }
}



// ************* base functions **************

void ManipulatorActor::findReference() {
    checkAutomaticStatus();
    PASS_("ManipulatorActor findReference started");
    manipulator->findReference();
}
void ManipulatorActor::waitForMoveComplete() {
    checkAutomaticStatus();
    PASS_("ManipulatorActor waitForMoveComplete started");
    manipulator->waitForMoveComplete();
}
void ManipulatorActor::moveTo(const Eigen::Vector3d position) {
    checkAutomaticStatus();
    PASS_("ManipulatorActor moveTo started");
    manipulator->moveTo(position);
}
void ManipulatorActor::open(double width) {
    checkAutomaticStatus();
    PASS_("ManipulatorActor open started");
    manipulator->open(width);
}
void ManipulatorActor::grip() {
    checkAutomaticStatus();
    PASS_("ManipulatorActor grip started");
    manipulator->grip();
}
void ManipulatorActor::release() {
    checkAutomaticStatus();
    PASS_("ManipulatorActor release started");
    manipulator->release();
}












