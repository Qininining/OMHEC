#ifndef AGEMOTIONACTOR_H
#define AGEMOTIONACTOR_H

#include <QObject>
#include <atomic>
#include <mutex>
#include <QFuture>

#include "AgeMotionDriver.h"

#include "BaslerCameraControl.h"

class AgeMotionActor:  public QObject
{
    Q_OBJECT
public:
    AgeMotionActor(double hz);
    ~AgeMotionActor();

    void setup(BaslerCameraControl* camera);
    void setFocusScale(double umPerPixel);

    bool connect();

    bool findReference(bool toHigh);
    bool setCurrPositionToZero();

    bool setTargetPosition(double positionUm);
    bool setRelativePosition(double deltaUm);
    bool getCommandedPosition(double& positionUm) const;

    void AutoFocusOnce(double umPerPixel = -1.0, bool debugTrace = false);

    void getPoint(cv::Vec3d& Point, double umPerPixel = -1.0);
    void enableGetCircle(bool enable);

signals:
    void sigFocusValue(double value);
    void sigAgeMotionValue(double value);
    void sigCircle(cv::Vec3d Circle);

public slots:
    void FocusCalculate();
    void PointDetect();

public:
    AgeMotionDriver* MotonDriver_ = nullptr;

private:
    std::atomic<bool> m_isLoopRunning_fous_{false};
    QFuture<void> m_future_fous_;

    std::atomic<bool> m_isLoopRunning_Target_{false};
    QFuture<void> m_future_Target_;

    // cv::Mat img_ = cv::Mat();
    // cv::Mat img_queen[2];
    // std::atomic<int> m_readIndex{0};

    BaslerCameraControl* camera_ = nullptr;
    std::atomic<double> AgeMotionValue_{0.0};
    // The controller's real-position register changes in about 0.5 um steps
    // even though commands are accepted on the 0.01 um pulse grid. Keep the
    // high-resolution commanded coordinate independently of that feedback.
    std::atomic<double> commandedPositionUm_{0.0};
    std::atomic<bool> commandedPositionValid_{false};
    mutable std::mutex motionCommandMutex_;
    std::atomic<double> FocusValue_{0.0};
    mutable std::mutex circleMutex_;
    cv::Vec3d preciseCircle_ = cv::Vec3d(-1,-1,-1);
    // Last valid target location used only to keep autofocus on one fixed ROI.
    // preciseCircle_ remains invalid when the current observation fails.
    cv::Vec3d focusRoiCircle_ = cv::Vec3d(-1,-1,-1);

    std::atomic<double> focusScaleUmPerPixel_{0.0};
    // 最近可靠主焦峰的锐度基准，仅用于识别明显偏低的次峰，不限制焦点位置。
    double reliableFocusScore_ = -1.0;
    bool enableGetCircle_ = true;
};

#endif // AGEMOTIONACTOR_H
