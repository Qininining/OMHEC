#include "AgeMotionActor.h"
#include "FocusAnalyzer.h"
#include "TargetDectect.h"
#include "RobotException.hpp"
#include <QCoreApplication>
#include <QtConcurrent>
#include <QTime>
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <QElapsedTimer>
#include <QStringList>
#include <algorithm>
#include <limits>
#include <cmath>
#include <vector>

namespace {
constexpr double kAutofocusCommandResolutionUm = 0.01;

double snapToAutofocusGrid(double positionUm)
{
    return std::round(positionUm / kAutofocusCommandResolutionUm) *
        kAutofocusCommandResolutionUm;
}

struct FocusProfile {
    double initialStepUm;
    double settleMs;
    int sampleCount;
    int sampleDelayMs;
    int maxBracketExpand;
    double bracketExpandFactor;
    int goldenIterations;
    double minBracketWidthUm;
    double refineStepUm;
    int roiSize;
    bool precisionMode;
};

FocusProfile buildFocusProfile(double umPerPixel)
{
    if (!std::isfinite(umPerPixel) || umPerPixel <= 0.0) {
        return {15.0, 180.0, 7, 25, 4, 1.40, 8, 0.5, 3.0, 500, false};
    }

    // 10X：与论文实验设置一致。精细模式使用新相机帧计算清晰度，
    // 并在多个亮度阈值下细化白色圆斑中心。
    if (umPerPixel >= 250.0 && umPerPixel < 500.0) {
        return {15.0, 180.0, 7, 25, 4, 1.40, 8, 0.5, 3.0, 420, true};
    }

    if (umPerPixel >= 500.0) {
        // 5X：小步长先定位主焦峰，必要时再扩展，避免50 um初始跨步落入次峰。
        return {15.0, 180.0, 7, 25, 4, 1.40, 8, 0.5, 3.0, 500, false};
    }

    // 20X：景深和视野更小，使用更窄的搜索与细化步长。
    return {4.0, 260.0, 7, 20, 7, 1.30, 10, 0.15, 0.75, 280, true};
}

double robustFocusScore(const std::atomic<double>& focusValue,
                        int sampleCount,
                        int sampleDelayMs,
                        double* robustSigma = nullptr)
{
    std::vector<double> scores;
    scores.reserve(sampleCount);

    for (int i = 0; i < sampleCount; ++i) {
        const double score = focusValue.load(std::memory_order_acquire);
        if (std::isfinite(score) && score >= 0.0)
            scores.push_back(score);

        if (i + 1 < sampleCount) {
            QThread::msleep(sampleDelayMs);
        }
    }

    if (scores.empty()) {
        if (robustSigma) {
            *robustSigma = std::numeric_limits<double>::infinity();
        }
        return -1.0;
    }

    // 中位数不会被单帧振动、曝光变化或错误圆斑ROI产生的锐度尖峰主导。
    std::sort(scores.begin(), scores.end());
    const size_t middle = scores.size() / 2;
    const double result = scores.size() % 2 == 0
        ? 0.5 * (scores[middle - 1] + scores[middle])
        : scores[middle];
    if (robustSigma) {
        std::vector<double> deviations;
        deviations.reserve(scores.size());
        for (double score : scores) {
            deviations.push_back(std::abs(score - result));
        }
        std::sort(deviations.begin(), deviations.end());
        const double mad = deviations.size() % 2 == 0
            ? 0.5 * (deviations[middle - 1] + deviations[middle])
            : deviations[middle];
        *robustSigma = 1.4826 * mad;
    }
    return result;
}

double robustFocusScoreFromNewFrames(BaslerCameraControl* camera,
                                     const cv::Rect& fixedRoi,
                                     int sampleCount,
                                     double* robustSigma = nullptr,
                                     bool debugTrace = false)
{
    if (!camera || fixedRoi.area() <= 0 || sampleCount <= 0) {
        if (robustSigma) {
            *robustSigma = std::numeric_limits<double>::infinity();
        }
        return -1.0;
    }

    cv::Mat frame;
    quint64 lastSequence = 0;
    camera->getLatestFrameWithSequence(frame, lastSequence);

    std::vector<double> scores;
    scores.reserve(sampleCount);
    quint64 firstUsedSequence = 0;
    quint64 lastUsedSequence = 0;
    while (static_cast<int>(scores.size()) < sampleCount) {
        QElapsedTimer frameTimeout;
        frameTimeout.start();
        bool receivedNewFrame = false;
        while (frameTimeout.elapsed() < 1000) {
            quint64 sequence = lastSequence;
            if (camera->getLatestFrameWithSequence(frame, sequence) &&
                sequence > lastSequence) {
                lastSequence = sequence;
                receivedNewFrame = true;
                break;
            }
            QThread::msleep(5);
        }
        if (!receivedNewFrame) {
            if (debugTrace) {
                qWarning() << "[AutoFocus][frames] timed out waiting for a new frame after sequence"
                           << lastSequence << "; collected"
                           << scores.size() << "of" << sampleCount;
            }
            if (robustSigma) {
                *robustSigma = std::numeric_limits<double>::infinity();
            }
            return -1.0;
        }

        const auto metrics =
            MicroscopeVision::FocusAnalyzer::computeGradientEnergy(frame, fixedRoi);
        if (metrics.isValid && std::isfinite(metrics.sharpnessScore) &&
            metrics.sharpnessScore >= 0.0) {
            if (scores.empty()) {
                firstUsedSequence = lastSequence;
            }
            lastUsedSequence = lastSequence;
            scores.push_back(metrics.sharpnessScore);
        }
    }

    std::sort(scores.begin(), scores.end());
    const size_t middle = scores.size() / 2;
    const double result = scores.size() % 2 == 0
        ? 0.5 * (scores[middle - 1] + scores[middle])
        : scores[middle];
    double calculatedSigma = 0.0;
    if (robustSigma || debugTrace) {
        std::vector<double> deviations;
        deviations.reserve(scores.size());
        for (double score : scores) {
            deviations.push_back(std::abs(score - result));
        }
        std::sort(deviations.begin(), deviations.end());
        const double mad = deviations.size() % 2 == 0
            ? 0.5 * (deviations[middle - 1] + deviations[middle])
            : deviations[middle];
        calculatedSigma = 1.4826 * mad;
        if (robustSigma) {
            *robustSigma = calculatedSigma;
        }
    }
    if (debugTrace) {
        QStringList scoreStrings;
        for (double score : scores) {
            scoreStrings << QString::number(score, 'g', 10);
        }
        qInfo().noquote()
            << QString("[AutoFocus][frames] sequence %1..%2, scores=[%3], median=%4, MAD-sigma=%5")
                   .arg(firstUsedSequence)
                   .arg(lastUsedSequence)
                   .arg(scoreStrings.join(", "))
                   .arg(result, 0, 'g', 10)
                   .arg(calculatedSigma, 0, 'g', 10);
    }
    return result;
}

} // namespace

AgeMotionActor::AgeMotionActor(double hz) {
    MotonDriver_ = new AgeMotionDriver(true);

    // 限制 OpenCV 内部每个算子只用单线程，避免过载
    cv::setNumThreads(1);

    m_isLoopRunning_fous_ = true;
    m_future_fous_ = QtConcurrent::run([this]() {
        while (m_isLoopRunning_fous_) {
            this->FocusCalculate();
            QThread::msleep(10);
        }
    });

    m_isLoopRunning_Target_ = true;
    m_future_Target_ = QtConcurrent::run([this]() {
        while (m_isLoopRunning_Target_) {
            this->PointDetect();
            QThread::msleep(10);
        }
    });
}

AgeMotionActor::~AgeMotionActor() {
    m_isLoopRunning_fous_ = false;
    m_isLoopRunning_Target_ = false;

    m_future_fous_.waitForFinished();
    m_future_Target_.waitForFinished();

    delete MotonDriver_;
}

bool AgeMotionActor::connect() {
    if (!MotonDriver_->connectDevice()) {
        return false;
    }

    double initialPositionUm = 0.0;
    if (MotonDriver_->getPosition(initialPositionUm)) {
        initialPositionUm = snapToAutofocusGrid(initialPositionUm);
        commandedPositionUm_.store(initialPositionUm, std::memory_order_release);
        commandedPositionValid_.store(true, std::memory_order_release);
        AgeMotionValue_.store(initialPositionUm, std::memory_order_release);
        emit sigAgeMotionValue(initialPositionUm);
    }
    return true;
}

void AgeMotionActor::setup(BaslerCameraControl* camera) {
    camera_ = camera;
}

void AgeMotionActor::setFocusScale(double umPerPixel) {
    focusScaleUmPerPixel_.store(umPerPixel, std::memory_order_release);
}

void AgeMotionActor::FocusCalculate() {
    double feedbackPositionUm = 0.0;
    if (MotonDriver_->getPosition(feedbackPositionUm)) {
        if (!commandedPositionValid_.load(std::memory_order_acquire)) {
            const double initialPositionUm =
                snapToAutofocusGrid(feedbackPositionUm);
            commandedPositionUm_.store(initialPositionUm,
                                       std::memory_order_release);
            commandedPositionValid_.store(true, std::memory_order_release);
        }
        const double displayPositionUm =
            commandedPositionUm_.load(std::memory_order_acquire);
        AgeMotionValue_.store(displayPositionUm, std::memory_order_release);
        emit sigAgeMotionValue(displayPositionUm);
    } else {

    }

    if (camera_ == nullptr){
        FocusValue_ = -1;
        return;
    }
    if (!camera_->isOpen() || !camera_->isAcquire()){
        FocusValue_ = -1;
        return;
    }
    cv::Mat img = camera_->getLatestFrame();
    if (img.empty()){
        FocusValue_ = -1;
        return;
        // throw MAR::RobotException(MAR::MARInternalError, "FocusCalculate:: Empty frame from camera");
    }

    cv::Rect roi;
    const FocusProfile profile = buildFocusProfile(focusScaleUmPerPixel_.load(std::memory_order_acquire));
    cv::Vec3d focusCircle;
    {
        std::lock_guard<std::mutex> lock(circleMutex_);
        focusCircle = focusRoiCircle_;
    }
    if (focusCircle[0] >= 0 && focusCircle[1] >= 0) {
        const int ROI_SIZE = profile.roiSize;
        const int cx = cvRound(focusCircle[0]);
        const int cy = cvRound(focusCircle[1]);
        const cv::Rect targetRect(cx - ROI_SIZE / 2, cy - ROI_SIZE / 2, ROI_SIZE, ROI_SIZE);
        const cv::Rect imgBounds(0, 0, img.cols, img.rows);
        roi = targetRect & imgBounds;
    }

    // Never switch from the target ROI to the full image when circle
    // detection is temporarily lost during defocus.
    if (roi.area() <= 0) {
        FocusValue_ = -1;
        return;
    }

    auto focusMetrics = MicroscopeVision::FocusAnalyzer::computeGradientEnergy(img, roi);
    FocusValue_ = focusMetrics.sharpnessScore;
    emit sigFocusValue(FocusValue_);
}
void AgeMotionActor::PointDetect() {

    if (this->enableGetCircle_) {
        if (camera_ == nullptr){
            std::lock_guard<std::mutex> lock(circleMutex_);
            preciseCircle_ = cv::Vec3d(-1,-1,-1);
            return;
        }
        if (!camera_->isOpen() || !camera_->isAcquire()){
            std::lock_guard<std::mutex> lock(circleMutex_);
            preciseCircle_ = cv::Vec3d(-1,-1,-1);
            return;
        }
        cv::Mat img = camera_->getLatestFrame();
        if (img.empty()){
            std::lock_guard<std::mutex> lock(circleMutex_);
            preciseCircle_ = cv::Vec3d(-1,-1,-1);
            return;
            // throw MAR::RobotException(MAR::MARInternalError, "FocusCalculate:: Empty frame from camera");
        }

        cv::Mat gray;
        if (img.channels() == 1)
            gray = img;
        else if (img.channels() == 3)
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else
            cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);

        cv::Mat src_8u;
        if (gray.depth() == CV_8U)
            src_8u = gray;
        else
            cv::normalize(gray, src_8u, 0, 255, cv::NORM_MINMAX, CV_8U);

        cv::Vec3d coarseCircle = TargetDectect::checkBestCircle(src_8u);
        const FocusProfile profile =
            buildFocusProfile(focusScaleUmPerPixel_.load(std::memory_order_acquire));
        cv::Vec3d preciseCircle = profile.precisionMode
            ? TargetDectect::findPreciseCenterHighMagnification(src_8u, coarseCircle)
            : TargetDectect::findPreciseCenter(src_8u, coarseCircle);

        const bool validCircle = preciseCircle[0] >= 0.0 && preciseCircle[1] >= 0.0 && preciseCircle[2] > 0.0;
        // 标定需要每一帧的独立位置观测，不能用上一位姿的圆心低通或在
        // 检测失败时复用旧值，否则会人为改变相邻点位移。
        {
            std::lock_guard<std::mutex> lock(circleMutex_);
            preciseCircle_ = validCircle
                ? preciseCircle : cv::Vec3d(-1, -1, -1);
            if (validCircle) {
                focusRoiCircle_ = preciseCircle;
            }
        }

        emit sigCircle(validCircle ? preciseCircle : cv::Vec3d(-1, -1, -1));
    }
}


void AgeMotionActor::AutoFocusOnce(double umPerPixel, bool debugTrace) {
    if (umPerPixel > 0.0) {
        setFocusScale(umPerPixel);
    } else {
        umPerPixel = focusScaleUmPerPixel_.load(std::memory_order_acquire);
    }

    const FocusProfile profile = buildFocusProfile(umPerPixel);

    if (camera_ == nullptr || !camera_->isOpen() || !camera_->isAcquire()) {
        qWarning() << "[AutoFocus] Camera is not acquiring.";
        return;
    }

    cv::Mat initialFrame;
    quint64 initialSequence = 0;
    if (!camera_->getLatestFrameWithSequence(initialFrame, initialSequence)) {
        qWarning() << "[AutoFocus] No camera frame is available.";
        return;
    }
    cv::Vec3d fixedFocusCircle;
    {
        std::lock_guard<std::mutex> lock(circleMutex_);
        fixedFocusCircle = focusRoiCircle_;
    }
    if (fixedFocusCircle[0] < 0.0 || fixedFocusCircle[1] < 0.0) {
        qWarning() << "[AutoFocus] No valid target ROI is available.";
        return;
    }
    const int focusRoiSize = profile.roiSize;
    const cv::Rect requestedFocusRoi(
        cvRound(fixedFocusCircle[0]) - focusRoiSize / 2,
        cvRound(fixedFocusCircle[1]) - focusRoiSize / 2,
        focusRoiSize, focusRoiSize);
    const cv::Rect fixedFocusRoi = requestedFocusRoi &
        cv::Rect(0, 0, initialFrame.cols, initialFrame.rows);
    if (fixedFocusRoi.area() < focusRoiSize * focusRoiSize / 2) {
        qWarning() << "[AutoFocus] Target ROI is outside the usable image area.";
        return;
    }
    if (debugTrace) {
        double feedbackPositionUm = 0.0;
        double hardwareTargetUm = 0.0;
        const bool feedbackOk = MotonDriver_->getPosition(feedbackPositionUm);
        const bool targetOk = MotonDriver_->getTargetPosition(hardwareTargetUm);
        qInfo().noquote()
            << QString("[AutoFocus][debug] BEGIN scale=%1 nm/pixel, precisionMode=%2, initialStep=%3 um, refineStep=%4 um, fineSamples=%5, coarseSamples=%6")
                   .arg(umPerPixel, 0, 'g', 10)
                   .arg(profile.precisionMode)
                   .arg(profile.initialStepUm)
                   .arg(profile.refineStepUm)
                   .arg(profile.sampleCount)
                   .arg(profile.precisionMode ? 3 : profile.sampleCount);
        qInfo() << "[AutoFocus][debug] fixed ROI x/y/w/h:"
                << fixedFocusRoi.x << fixedFocusRoi.y
                << fixedFocusRoi.width << fixedFocusRoi.height
                << "initial frame sequence:" << initialSequence;
        qInfo() << "[AutoFocus][debug] hardware feedback/target (um):"
                << (feedbackOk ? feedbackPositionUm
                               : std::numeric_limits<double>::quiet_NaN())
                << (targetOk ? hardwareTargetUm
                             : std::numeric_limits<double>::quiet_NaN());
    }

    // 定义等待函数，保持事件循环运行以便 FocusCalculate 能够更新 FocusValue_
    auto waitAndProcess = [](int durationMs) {
        QTime deadline = QTime::currentTime().addMSecs(durationMs);
        while (QTime::currentTime() < deadline) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            QThread::msleep(5);
        }
    };

    auto waitForMotionComplete = [&](int timeoutMs) {
        bool isMotionDone = false;
        QTime motionTimeout = QTime::currentTime().addMSecs(timeoutMs);
        while (QTime::currentTime() < motionTimeout) {
            if (!MotonDriver_->isMotionComplete(isMotionDone)) {
                return false;
            }
            if (isMotionDone) {
                return true;
            }
            waitAndProcess(20);
        }
        return false;
    };

    double lastReachedPositionUm = std::numeric_limits<double>::quiet_NaN();
    int debugMoveIndex = 0;
    auto moveToPosition = [&](double targetPositionUm, int settleMs) {
        const int moveIndex = ++debugMoveIndex;
        double currentPositionUm = 0.0;
        if (!getCommandedPosition(currentPositionUm)) {
            qWarning() << "[AutoFocus][move] logical position unavailable.";
            return false;
        }

        double feedbackBeforeUm = 0.0;
        double hardwareTargetBeforeUm = 0.0;
        const bool feedbackBeforeOk =
            MotonDriver_->getPosition(feedbackBeforeUm);
        const bool targetBeforeOk =
            MotonDriver_->getTargetPosition(hardwareTargetBeforeUm);

        const double deltaUm = targetPositionUm - currentPositionUm;
        if (debugTrace) {
            qInfo() << "[AutoFocus][move] #" << moveIndex
                    << "request/current/delta (um):"
                    << targetPositionUm << currentPositionUm << deltaUm
                    << "feedback/target-register before (um):"
                    << (feedbackBeforeOk ? feedbackBeforeUm
                                         : std::numeric_limits<double>::quiet_NaN())
                    << (targetBeforeOk ? hardwareTargetBeforeUm
                                       : std::numeric_limits<double>::quiet_NaN());
        }

        QElapsedTimer motionElapsed;
        qint64 completionElapsedMs = 0;
        if (std::abs(deltaUm) > 1e-6) {
            if (!setTargetPosition(targetPositionUm)) {
                qWarning() << "[AutoFocus][move] #" << moveIndex
                           << "failed to write target" << targetPositionUm;
                return false;
            }
            motionElapsed.start();
            // Wait longer than one 8 FPS camera period before the first
            // completion check.  This gives the controller enough time to
            // consume the target and start even a very small 0.01 um move,
            // whose coarse position feedback may otherwise look complete.
            waitAndProcess(150);
            if (!waitForMotionComplete(5000)) {
                double failedFeedbackUm = 0.0;
                double failedTargetUm = 0.0;
                double failedVelocityUmPerSec = 0.0;
                MotonDriver_->getPosition(failedFeedbackUm);
                MotonDriver_->getTargetPosition(failedTargetUm);
                const bool failedVelocityOk =
                    MotonDriver_->getVelocity(failedVelocityUmPerSec);
                qWarning() << "[AutoFocus][move] #" << moveIndex
                           << "timeout/failure after" << motionElapsed.elapsed()
                           << "ms; feedback/target-register (um):"
                           << failedFeedbackUm << failedTargetUm
                           << "velocity (um/s):"
                           << (failedVelocityOk
                                   ? failedVelocityUmPerSec
                                   : std::numeric_limits<double>::quiet_NaN());
                return false;
            }
            completionElapsedMs = motionElapsed.elapsed();
        }

        waitAndProcess(settleMs);
        // The hardware feedback register is quantized at about 0.5 um. Focus
        // samples must therefore be associated with the 0.01 um command grid.
        const bool logicalPositionOk =
            getCommandedPosition(lastReachedPositionUm);
        if (debugTrace) {
            double feedbackAfterUm = 0.0;
            double hardwareTargetAfterUm = 0.0;
            double velocityAfterUmPerSec = 0.0;
            const bool feedbackAfterOk =
                MotonDriver_->getPosition(feedbackAfterUm);
            const bool targetAfterOk =
                MotonDriver_->getTargetPosition(hardwareTargetAfterUm);
            const bool velocityAfterOk =
                MotonDriver_->getVelocity(velocityAfterUmPerSec);
            qInfo() << "[AutoFocus][move] #" << moveIndex
                    << "done; completion/settle (ms):"
                    << completionElapsedMs << settleMs
                    << "logical/feedback/target-register after (um):"
                    << (logicalPositionOk
                            ? lastReachedPositionUm
                            : std::numeric_limits<double>::quiet_NaN())
                    << (feedbackAfterOk
                            ? feedbackAfterUm
                            : std::numeric_limits<double>::quiet_NaN())
                    << (targetAfterOk
                            ? hardwareTargetAfterUm
                            : std::numeric_limits<double>::quiet_NaN())
                    << "velocity (um/s):"
                    << (velocityAfterOk
                            ? velocityAfterUmPerSec
                            : std::numeric_limits<double>::quiet_NaN());
        }
        return logicalPositionOk;
    };

    auto measureAtCurrentPosition = [&](double* robustSigma = nullptr,
                                        int sampleCountOverride = 0) -> double {
        const int sampleCount = sampleCountOverride > 0
            ? sampleCountOverride : profile.sampleCount;
        if (profile.precisionMode) {
            return robustFocusScoreFromNewFrames(
                camera_, fixedFocusRoi, sampleCount, robustSigma,
                debugTrace);
        }
        return robustFocusScore(FocusValue_, sampleCount,
                                profile.sampleDelayMs, robustSigma);
    };

    double startPositionUm = 0.0;
    if (!getCommandedPosition(startPositionUm)) {
        return;
    }
    lastReachedPositionUm = startPositionUm;

    double bestObservedPositionUm = startPositionUm;
    double bestObservedScore = -1.0;

    auto recordBestSample = [&](double positionUm, double score) {
        if (std::isfinite(score) && score >= 0.0 && score > bestObservedScore) {
            bestObservedScore = score;
            bestObservedPositionUm = positionUm;
        }
    };

    auto measureAtPosition = [&](double targetPositionUm,
                                 double* robustSigma = nullptr,
                                 int sampleCountOverride = 0) -> double {
        if (!moveToPosition(targetPositionUm, static_cast<int>(profile.settleMs))) {
            return -1.0;
        }
        const double score = measureAtCurrentPosition(
            robustSigma, sampleCountOverride);
        recordBestSample(lastReachedPositionUm, score);
        if (debugTrace) {
            qInfo() << "[AutoFocus][sample] requested/used position (um), score:"
                    << targetPositionUm << lastReachedPositionUm << score;
        }
        return score;
    };

    // 精细模式下，最终三个焦点评价位置都从同一方向逼近，以减小轴向回差。
    constexpr double finalApproachPreloadUm = 0.50;
    auto measureAtPositionOneDirection = [&](double targetPositionUm,
                                             double* robustSigma = nullptr) {
        if (profile.precisionMode) {
            const int preloadSettleMs =
                std::max(80, static_cast<int>(profile.settleMs * 0.5));
            if (!moveToPosition(targetPositionUm - finalApproachPreloadUm,
                                preloadSettleMs)) {
                return -1.0;
            }
        }
        return measureAtPosition(targetPositionUm, robustSigma);
    };

    // 初始等待，确保获取稳定的初始对焦值
    waitAndProcess(static_cast<int>(profile.settleMs * 0.75));
    double currentSigma = 0.0;
    const int coarseSampleCount = profile.precisionMode ? 3 : profile.sampleCount;
    double currentScore = measureAtCurrentPosition(
        &currentSigma, coarseSampleCount);
    if (currentScore < 0) {
        qWarning() << "[AutoFocus] Failed to measure the initial focus score.";
        return;
    }
    if (debugTrace) {
        qInfo() << "[AutoFocus][sample] initial position (um), score:"
                << startPositionUm << currentScore;
    }
    bestObservedScore = currentScore;
    recordBestSample(startPositionUm, currentScore);

    const double initialStepUm = profile.initialStepUm;
    double leftPosUm = 0.0;
    double midPosUm = startPositionUm;
    double rightPosUm = 0.0;
    double leftSigma = 0.0;
    double rightSigma = 0.0;
    double leftScore = -1.0;
    double midScore = currentScore;
    double rightScore = -1.0;
    bool initialBracketFlat = false;
    const bool initialWidePrescanUsed =
        profile.precisionMode && reliableFocusScore_ <= 0.0;

    if (initialWidePrescanUsed) {
        constexpr double captureStepUm = 12.0;
        constexpr int captureHalfGridCount = 5;
        std::vector<double> capturePositions;
        std::vector<double> captureScores;
        capturePositions.reserve(2 * captureHalfGridCount + 1);
        captureScores.reserve(2 * captureHalfGridCount + 1);
        int captureBestIndex = -1;
        double captureBestScore = -1.0;

        for (int gridIndex = -captureHalfGridCount;
             gridIndex <= captureHalfGridCount; ++gridIndex) {
            const double capturePositionUm =
                startPositionUm + gridIndex * captureStepUm;
            const double captureScore = gridIndex == 0
                ? currentScore
                : measureAtPosition(capturePositionUm, nullptr, 1);
            if (captureScore < 0.0) {
                moveToPosition(startPositionUm,
                               static_cast<int>(profile.settleMs));
                return;
            }
            capturePositions.push_back(
                gridIndex == 0 ? startPositionUm : lastReachedPositionUm);
            captureScores.push_back(captureScore);
            if (captureScore > captureBestScore) {
                captureBestScore = captureScore;
                captureBestIndex =
                    static_cast<int>(captureScores.size()) - 1;
            }
        }

        midPosUm = capturePositions[captureBestIndex];
        midScore = captureScores[captureBestIndex];
        if (captureBestIndex == 0) {
            rightPosUm = capturePositions[1];
            rightScore = captureScores[1];
            leftPosUm = midPosUm - captureStepUm;
            leftScore = measureAtPosition(leftPosUm, nullptr, 1);
            leftPosUm = lastReachedPositionUm;
        } else if (captureBestIndex + 1 ==
                   static_cast<int>(capturePositions.size())) {
            leftPosUm = capturePositions[captureBestIndex - 1];
            leftScore = captureScores[captureBestIndex - 1];
            rightPosUm = midPosUm + captureStepUm;
            rightScore = measureAtPosition(rightPosUm, nullptr, 1);
            rightPosUm = lastReachedPositionUm;
        } else {
            leftPosUm = capturePositions[captureBestIndex - 1];
            leftScore = captureScores[captureBestIndex - 1];
            rightPosUm = capturePositions[captureBestIndex + 1];
            rightScore = captureScores[captureBestIndex + 1];
        }
        if (leftScore < 0.0 || rightScore < 0.0) {
            moveToPosition(startPositionUm,
                           static_cast<int>(profile.settleMs));
            return;
        }
        if (debugTrace) {
            qInfo() << "[AutoFocus][capture] fast +/-60 um best position/score:"
                    << midPosUm << midScore
                    << "bracket:" << leftPosUm << rightPosUm;
        }
    } else {
        leftPosUm = startPositionUm - initialStepUm;
        leftScore = measureAtPosition(
            leftPosUm, &leftSigma, coarseSampleCount);
        if (leftScore < 0.0) {
            return;
        }
        leftPosUm = lastReachedPositionUm;

        rightPosUm = startPositionUm + initialStepUm;
        rightScore = measureAtPosition(
            rightPosUm, &rightSigma, coarseSampleCount);
        if (rightScore < 0.0) {
            return;
        }
        rightPosUm = lastReachedPositionUm;

        const double initialScoreMin =
            std::min(leftScore, std::min(midScore, rightScore));
        const double initialScoreMax =
            std::max(leftScore, std::max(midScore, rightScore));
        const double initialNoiseSigma =
            std::max(leftSigma, std::max(currentSigma, rightSigma));
        initialBracketFlat =
            profile.precisionMode &&
            (initialScoreMax - initialScoreMin <= 3.0 * initialNoiseSigma);
    }

    if (debugTrace) {
        qInfo() << "[AutoFocus][bracket] initial L/C/R positions (um):"
                << leftPosUm << midPosUm << rightPosUm
                << "scores:" << leftScore << midScore << rightScore
                << "MAD-sigma:" << leftSigma << currentSigma << rightSigma;
    }

    const int maxBracketExpand = profile.maxBracketExpand;
    for (int expandIndex = 0; expandIndex < maxBracketExpand; ++expandIndex) {
        if (midScore >= leftScore && midScore >= rightScore) {
            break;
        }

        if (rightScore > midScore) {
            leftPosUm = midPosUm;
            leftScore = midScore;
            midPosUm = rightPosUm;
            midScore = rightScore;
            rightPosUm = midPosUm + initialStepUm * std::pow(profile.bracketExpandFactor, expandIndex + 1);
            rightScore = measureAtPosition(
                rightPosUm, nullptr, coarseSampleCount);
            if (rightScore < 0) {
                return;
            }
            rightPosUm = lastReachedPositionUm;
        } else if (leftScore > midScore) {
            rightPosUm = midPosUm;
            rightScore = midScore;
            midPosUm = leftPosUm;
            midScore = leftScore;
            leftPosUm = midPosUm - initialStepUm * std::pow(profile.bracketExpandFactor, expandIndex + 1);
            leftScore = measureAtPosition(
                leftPosUm, nullptr, coarseSampleCount);
            if (leftScore < 0) {
                return;
            }
            leftPosUm = lastReachedPositionUm;
        } else {
            break;
        }
    }
    if (debugTrace) {
        qInfo() << "[AutoFocus][bracket] expanded L/C/R positions (um):"
                << leftPosUm << midPosUm << rightPosUm
                << "scores:" << leftScore << midScore << rightScore;
    }

    const double expandedNeighbourScore = std::max(leftScore, rightScore);
    const bool hasConfidentExpandedPeak =
        profile.precisionMode && midScore > 0.0 &&
        midScore >= leftScore && midScore >= rightScore &&
        midScore >= 1.25 * expandedNeighbourScore;

    const double goldenRatio = 0.6180339887498949;
    double bracketLeft = leftPosUm;
    double bracketRight = rightPosUm;

    double x1 = bracketRight - goldenRatio * (bracketRight - bracketLeft);
    double x2 = bracketLeft + goldenRatio * (bracketRight - bracketLeft);
    double f1 = measureAtPosition(x1, nullptr, coarseSampleCount);
    if (f1 < 0) {
        return;
    }
    x1 = lastReachedPositionUm;
    double f2 = measureAtPosition(x2, nullptr, coarseSampleCount);
    if (f2 < 0) {
        return;
    }
    x2 = lastReachedPositionUm;

    const int goldenIterations = profile.goldenIterations;
    const double minBracketWidth = profile.minBracketWidthUm;
    for (int iter = 0; iter < goldenIterations && (bracketRight - bracketLeft) > minBracketWidth; ++iter) {
        if (f1 < f2) {
            bracketLeft = x1;
            x1 = x2;
            f1 = f2;
            x2 = bracketLeft + goldenRatio * (bracketRight - bracketLeft);
            f2 = measureAtPosition(x2, nullptr, coarseSampleCount);
            if (f2 < 0) {
                return;
            }
            x2 = lastReachedPositionUm;
        } else {
            bracketRight = x2;
            x2 = x1;
            f2 = f1;
            x1 = bracketRight - goldenRatio * (bracketRight - bracketLeft);
            f1 = measureAtPosition(x1, nullptr, coarseSampleCount);
            if (f1 < 0) {
                return;
            }
            x1 = lastReachedPositionUm;
        }
    }

    const double coarseBestPositionUm = (f1 > f2) ? x1 : x2;
    const double coarseBestScore = std::max(f1, f2);
    recordBestSample(coarseBestPositionUm, coarseBestScore);
    if (debugTrace) {
        qInfo() << "[AutoFocus][golden] final bracket/x1/x2 (um):"
                << bracketLeft << bracketRight << x1 << x2
                << "scores:" << f1 << f2
                << "best:" << coarseBestPositionUm << coarseBestScore;
    }

    const double refineStepUm = profile.precisionMode
        ? profile.refineStepUm
        : std::max(profile.refineStepUm, initialStepUm * 0.25);
    for (int refineIndex = 0; refineIndex < 2; ++refineIndex) {
        const double centerPositionUm = bestObservedPositionUm;
        const double centerScore = measureAtPosition(centerPositionUm);
        if (centerScore < 0) {
            return;
        }
        const double reachedCenterPositionUm = lastReachedPositionUm;

        const double leftRefineScore = measureAtPosition(centerPositionUm - refineStepUm);
        if (leftRefineScore < 0) {
            return;
        }
        const double reachedLeftPositionUm = lastReachedPositionUm;

        const double rightRefineScore = measureAtPosition(centerPositionUm + refineStepUm);
        if (rightRefineScore < 0) {
            return;
        }
        const double reachedRightPositionUm = lastReachedPositionUm;

        if (centerScore >= leftRefineScore && centerScore >= rightRefineScore) {
            break;
        }

        if (leftRefineScore > centerScore && leftRefineScore >= rightRefineScore) {
            bestObservedPositionUm = reachedLeftPositionUm;
            bestObservedScore = leftRefineScore;
        } else if (rightRefineScore > centerScore && rightRefineScore >= leftRefineScore) {
            bestObservedPositionUm = reachedRightPositionUm;
            bestObservedScore = rightRefineScore;
        } else {
            bestObservedPositionUm = reachedCenterPositionUm;
            bestObservedScore = centerScore;
        }
    }

    // 对最终候选及其左右邻点重新独立测量，以局部三点峰形复核搜索结果。
    // 正式运动后的真实焦面可能远离起点，因此不能按离起点的距离拒绝候选。
    const double candidatePositionUm = bestObservedPositionUm;
    const double verifyStepUm = profile.precisionMode
        ? profile.refineStepUm
        : std::max(0.5, profile.refineStepUm);
    double leftVerifySigma = 0.0;
    double candidateSigma = 0.0;
    double rightVerifySigma = 0.0;
    const double leftVerifyScore = measureAtPositionOneDirection(
        candidatePositionUm - verifyStepUm, &leftVerifySigma);
    const double leftVerifyPositionUm = lastReachedPositionUm;
    const double candidateScore = measureAtPositionOneDirection(
        candidatePositionUm, &candidateSigma);
    const double candidateReachedPositionUm = lastReachedPositionUm;
    const double rightVerifyScore = measureAtPositionOneDirection(
        candidatePositionUm + verifyStepUm, &rightVerifySigma);
    const double rightVerifyPositionUm = lastReachedPositionUm;
    if (leftVerifyScore < 0.0 || candidateScore < 0.0 || rightVerifyScore < 0.0) {
        moveToPosition(startPositionUm, static_cast<int>(profile.settleMs));
        return;
    }

    double acceptedPositionUm = candidateReachedPositionUm;
    double acceptedScore = candidateScore;
    if (leftVerifyScore > acceptedScore) {
        acceptedPositionUm = leftVerifyPositionUm;
        acceptedScore = leftVerifyScore;
    }
    if (rightVerifyScore > acceptedScore) {
        acceptedPositionUm = rightVerifyPositionUm;
        acceptedScore = rightVerifyScore;
    }

    if (profile.precisionMode) {
        // 对局部左、中、右三个清晰度值作抛物线插值，并用真实图像复测顶点，
        // 避免焦点结果被局部搜索步长量化。
        const double curvature =
            leftVerifyScore - 2.0 * candidateScore + rightVerifyScore;
        const double scoreScale = std::max(
            1.0, std::max(leftVerifyScore,
                          std::max(candidateScore, rightVerifyScore)));
        const double focusNoise = std::max(
            leftVerifySigma, std::max(candidateSigma, rightVerifySigma));
        const double requiredCurvature = std::max(
            3.0 * focusNoise, 1e-4 * scoreScale);
        if (curvature < -requiredCurvature) {
            double vertexOffsetUm = 0.5 * verifyStepUm *
                (leftVerifyScore - rightVerifyScore) / curvature;
            vertexOffsetUm = std::max(-verifyStepUm,
                                      std::min(verifyStepUm, vertexOffsetUm));
            const double vertexPositionUm = candidatePositionUm + vertexOffsetUm;
            const double vertexScore =
                measureAtPositionOneDirection(vertexPositionUm);
            if (vertexScore >= 0.0 && vertexScore >= acceptedScore) {
                acceptedPositionUm = lastReachedPositionUm;
                acceptedScore = vertexScore;
            }
        }
    }

    // 局部三点只能验证附近峰形，无法发现几十um外更高的主峰。若本次峰值
    // 明显低于历史可靠锐度，则围绕运动模型给出的预定位位置执行宽范围恢复
    // 扫描。只比较锐度，不用历史Z位置约束当前真实焦面。
    bool recoveryScanUsed = false;
    const bool needsInitialWideScan =
        profile.precisionMode && reliableFocusScore_ <= 0.0 &&
        !initialWidePrescanUsed && !hasConfidentExpandedPeak;
    const bool scoreLostMainPeak =
        reliableFocusScore_ > 0.0 &&
        acceptedScore < 0.90 * reliableFocusScore_;
    if (debugTrace) {
        qInfo() << "[AutoFocus][recovery] confident-peak/initial-wide/flat/lost-main-peak/baseline/current:"
                << hasConfidentExpandedPeak << needsInitialWideScan
                << initialBracketFlat << scoreLostMainPeak
                << reliableFocusScore_ << acceptedScore;
    }
    if (needsInitialWideScan ||
        (initialBracketFlat && !initialWidePrescanUsed) ||
        scoreLostMainPeak) {
        recoveryScanUsed = true;
        double recoveryPositionUm = acceptedPositionUm;
        double recoveryScore = acceptedScore;
        constexpr int halfGridCount = 5; // 12 um网格覆盖起点两侧各60 um。
        constexpr double recoveryGridStepUm = 12.0;
        std::vector<double> recoveryGridPositions;
        std::vector<double> recoveryGridScores;
        recoveryGridPositions.reserve(2 * halfGridCount + 1);
        recoveryGridScores.reserve(2 * halfGridCount + 1);
        int recoveryBestGridIndex = -1;
        double recoveryBestGridScore = -1.0;
        for (int gridIndex = -halfGridCount; gridIndex <= halfGridCount; ++gridIndex) {
            const double gridPositionUm =
                startPositionUm + gridIndex * recoveryGridStepUm;
            const double gridScore = measureAtPosition(
                gridPositionUm, nullptr, 1);
            if (gridScore < 0.0) {
                moveToPosition(startPositionUm,
                               static_cast<int>(profile.settleMs));
                return;
            }
            recoveryGridPositions.push_back(lastReachedPositionUm);
            recoveryGridScores.push_back(gridScore);
            if (gridScore > recoveryBestGridScore) {
                recoveryBestGridScore = gridScore;
                recoveryBestGridIndex =
                    static_cast<int>(recoveryGridScores.size()) - 1;
            }
            if (gridScore > recoveryScore) {
                recoveryScore = gridScore;
                recoveryPositionUm = lastReachedPositionUm;
            }
        }

        // The wide grid finds only the capture interval. Refine inside the
        // two neighbouring coarse samples so the 6 um grid cannot quantize
        // or skip the narrow focus peak.
        if (recoveryBestGridIndex > 0 &&
            recoveryBestGridIndex + 1 <
                static_cast<int>(recoveryGridPositions.size())) {
            double recoveryBracketLeft =
                recoveryGridPositions[recoveryBestGridIndex - 1];
            double recoveryBracketRight =
                recoveryGridPositions[recoveryBestGridIndex + 1];
            double recoveryX1 = recoveryBracketRight - goldenRatio *
                (recoveryBracketRight - recoveryBracketLeft);
            double recoveryX2 = recoveryBracketLeft + goldenRatio *
                (recoveryBracketRight - recoveryBracketLeft);
            double recoveryF1 = measureAtPosition(
                recoveryX1, nullptr, coarseSampleCount);
            recoveryX1 = lastReachedPositionUm;
            double recoveryF2 = measureAtPosition(
                recoveryX2, nullptr, coarseSampleCount);
            recoveryX2 = lastReachedPositionUm;
            if (recoveryF1 < 0.0 || recoveryF2 < 0.0) {
                moveToPosition(startPositionUm,
                               static_cast<int>(profile.settleMs));
                return;
            }

            for (int iter = 0;
                 iter < 10 &&
                 recoveryBracketRight - recoveryBracketLeft > 0.15;
                 ++iter) {
                if (recoveryF1 < recoveryF2) {
                    recoveryBracketLeft = recoveryX1;
                    recoveryX1 = recoveryX2;
                    recoveryF1 = recoveryF2;
                    recoveryX2 = recoveryBracketLeft + goldenRatio *
                        (recoveryBracketRight - recoveryBracketLeft);
                    recoveryF2 = measureAtPosition(
                        recoveryX2, nullptr, coarseSampleCount);
                    recoveryX2 = lastReachedPositionUm;
                    if (recoveryF2 < 0.0) {
                        moveToPosition(startPositionUm,
                                       static_cast<int>(profile.settleMs));
                        return;
                    }
                } else {
                    recoveryBracketRight = recoveryX2;
                    recoveryX2 = recoveryX1;
                    recoveryF2 = recoveryF1;
                    recoveryX1 = recoveryBracketRight - goldenRatio *
                        (recoveryBracketRight - recoveryBracketLeft);
                    recoveryF1 = measureAtPosition(
                        recoveryX1, nullptr, coarseSampleCount);
                    recoveryX1 = lastReachedPositionUm;
                    if (recoveryF1 < 0.0) {
                        moveToPosition(startPositionUm,
                                       static_cast<int>(profile.settleMs));
                        return;
                    }
                }
            }
            if (recoveryF1 > recoveryScore) {
                recoveryScore = recoveryF1;
                recoveryPositionUm = recoveryX1;
            }
            if (recoveryF2 > recoveryScore) {
                recoveryScore = recoveryF2;
                recoveryPositionUm = recoveryX2;
            }
            if (debugTrace) {
                qInfo() << "[AutoFocus][recovery] refined interval (um):"
                        << recoveryBracketLeft << recoveryBracketRight
                        << "best position/score:"
                        << recoveryPositionUm << recoveryScore;
            }
        } else {
            qWarning() << "[AutoFocus][recovery] best sample is at the +/-60 um boundary;"
                       << "the true focus may be outside the recovery range.";
        }

        // 对恢复扫描的最佳网格点再做一次左右细化。
        const double recoveryLeftScore =
            measureAtPositionOneDirection(recoveryPositionUm - verifyStepUm);
        const double recoveryLeftPositionUm = lastReachedPositionUm;
        const double recoveryCenterScore =
            measureAtPositionOneDirection(recoveryPositionUm);
        const double recoveryCenterPositionUm = lastReachedPositionUm;
        const double recoveryRightScore =
            measureAtPositionOneDirection(recoveryPositionUm + verifyStepUm);
        const double recoveryRightPositionUm = lastReachedPositionUm;
        acceptedPositionUm = recoveryCenterPositionUm;
        acceptedScore = recoveryCenterScore;
        if (recoveryLeftScore > acceptedScore) {
            acceptedPositionUm = recoveryLeftPositionUm;
            acceptedScore = recoveryLeftScore;
        }
        if (recoveryRightScore > acceptedScore) {
            acceptedPositionUm = recoveryRightPositionUm;
            acceptedScore = recoveryRightScore;
        }
    }

    // 首次成功结果建立基准；后续只用接近主峰质量的结果缓慢更新基准，
    // 避免一次错误次峰把可靠阈值拉低。
    if (profile.precisionMode &&
        !moveToPosition(acceptedPositionUm - finalApproachPreloadUm,
                        std::max(80, static_cast<int>(profile.settleMs * 0.5)))) {
        moveToPosition(startPositionUm, static_cast<int>(profile.settleMs));
        return;
    }
    if (!moveToPosition(acceptedPositionUm, static_cast<int>(profile.settleMs))) {
        moveToPosition(startPositionUm, static_cast<int>(profile.settleMs));
        return;
    }
    acceptedPositionUm = lastReachedPositionUm;

    if (reliableFocusScore_ <= 0.0) {
        reliableFocusScore_ = acceptedScore;
    } else if (acceptedScore >= 0.90 * reliableFocusScore_) {
        // 新发现更清晰的主峰时立即抬高基准；否则只允许非常缓慢地下调，
        // 兼顾曝光缓变，同时避免较差焦点快速污染基准。
        reliableFocusScore_ =
            std::max(0.995 * reliableFocusScore_, acceptedScore);
    }
    if (profile.precisionMode) {
        qInfo() << "[AutoFocus][precision] start/candidate/accepted (um):"
                << startPositionUm << candidatePositionUm << acceptedPositionUm
                << "verify scores L/C/R:"
                << leftVerifyScore << candidateScore << rightVerifyScore
                << "accepted score:" << acceptedScore
                << "recovery:" << recoveryScanUsed;
    }
    if (debugTrace) {
        double finalFeedbackUm = 0.0;
        double finalHardwareTargetUm = 0.0;
        MotonDriver_->getPosition(finalFeedbackUm);
        MotonDriver_->getTargetPosition(finalHardwareTargetUm);
        qInfo() << "[AutoFocus][debug] END logical/feedback/target-register (um):"
                << acceptedPositionUm << finalFeedbackUm
                << finalHardwareTargetUm
                << "verify sigma L/C/R:"
                << leftVerifySigma << candidateSigma << rightVerifySigma;
    }
    // qInfo() << "[AutoFocus] start/candidate/accepted (um):"
    //         << startPositionUm << candidatePositionUm << acceptedPositionUm
    //         << "local scores L/C/R:"
    //         << leftVerifyScore << candidateScore << rightVerifyScore
    //         << "accepted/baseline score:" << acceptedScore << reliableFocusScore_
    //         << "recovery scan used:" << recoveryScanUsed;
}

void AgeMotionActor::getPoint(cv::Vec3d& Point, double umPerPixel) {
    // 初始化设置与对焦
    // this->enableGetCircle(false);
    this->AutoFocusOnce(umPerPixel);
    // this->enableGetCircle(true);

    QThread::msleep(1200);

    const int sampleCount = 10;
    const bool useRobustHighMagnificationAggregation =
        buildFocusProfile(umPerPixel).precisionMode;
    auto collectPointSamples = [&]() {
        cv::Vec3d sumPoint(0, 0, 0);
        std::vector<double> sampleXs;
        std::vector<double> sampleYs;
        int validCount = 0;
        const int maxAttempts = sampleCount * 2;
        for (int attempts = 0; validCount < sampleCount && attempts < maxAttempts; ++attempts) {
            cv::Vec3d currentCircle;
            {
                std::lock_guard<std::mutex> lock(circleMutex_);
                currentCircle = preciseCircle_;
            }
            if (currentCircle != cv::Vec3d(-1, -1, -1)) {
                double position = 0;
                if (getCommandedPosition(position)) {
                    sumPoint[0] += currentCircle[0];
                    sumPoint[1] += currentCircle[1];
                    sumPoint[2] += -static_cast<double>(1000.0 * position);
                    sampleXs.push_back(currentCircle[0]);
                    sampleYs.push_back(currentCircle[1]);
                    validCount++;
                }
            }
            QThread::msleep(100);
        }
        // 至少需要半数有效读数，避免仅凭一两帧偶然检测结果生成观测。
        if (validCount < sampleCount / 2)
            return false;
        Point = sumPoint / static_cast<double>(validCount);
        if (useRobustHighMagnificationAggregation) {
            // Median aggregation rejects occasional halo-induced centre jumps
            // without changing the downstream 3-D observation interface.
            const size_t middle = sampleXs.size() / 2;
            std::nth_element(sampleXs.begin(), sampleXs.begin() + middle, sampleXs.end());
            std::nth_element(sampleYs.begin(), sampleYs.begin() + middle, sampleYs.end());
            Point[0] = sampleXs[middle];
            Point[1] = sampleYs[middle];
        }
        return true;
    };

    bool acquired = collectPointSamples();
    if (!acquired) {
        qWarning() << "[AutoFocus] Circle missing after first focus; retrying full autofocus once.";
        this->AutoFocusOnce(umPerPixel);
        QThread::msleep(1200);
        acquired = collectPointSamples();
    }

    if (!acquired) {
        Point = cv::Vec3d(-1, -1, -1);
        throw MAR::RobotException(MAR::MARInternalError, "[MHECTask]: Data acquisition failed after multiple attempts");
    }
    // this->enableGetCircle(false);
}

void AgeMotionActor::enableGetCircle(bool enable) {
    this->enableGetCircle_ = enable;
}



bool AgeMotionActor::findReference(bool toHigh) {
    return MotonDriver_->findReference(toHigh);
}
bool AgeMotionActor::setCurrPositionToZero() {
    std::lock_guard<std::mutex> lock(motionCommandMutex_);
    if (!MotonDriver_->setCurrPositionToZero()) {
        return false;
    }
    commandedPositionUm_.store(0.0, std::memory_order_release);
    commandedPositionValid_.store(true, std::memory_order_release);
    AgeMotionValue_.store(0.0, std::memory_order_release);
    emit sigAgeMotionValue(0.0);
    return true;
}
bool AgeMotionActor::setTargetPosition(double positionUm) {
    if (!std::isfinite(positionUm)) {
        return false;
    }
    const double snappedPositionUm = snapToAutofocusGrid(positionUm);
    std::lock_guard<std::mutex> lock(motionCommandMutex_);
    if (!MotonDriver_->setTargetPosition(snappedPositionUm)) {
        return false;
    }
    commandedPositionUm_.store(snappedPositionUm, std::memory_order_release);
    commandedPositionValid_.store(true, std::memory_order_release);
    AgeMotionValue_.store(snappedPositionUm, std::memory_order_release);
    emit sigAgeMotionValue(snappedPositionUm);
    return true;
}
bool AgeMotionActor::setRelativePosition(double deltaUm) {
    if (!std::isfinite(deltaUm)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(motionCommandMutex_);
    double basePositionUm = 0.0;
    if (commandedPositionValid_.load(std::memory_order_acquire)) {
        basePositionUm =
            commandedPositionUm_.load(std::memory_order_acquire);
    } else if (!MotonDriver_->getPosition(basePositionUm)) {
        return false;
    }

    const double targetPositionUm =
        snapToAutofocusGrid(basePositionUm + deltaUm);
    if (!MotonDriver_->setTargetPosition(targetPositionUm)) {
        return false;
    }
    commandedPositionUm_.store(targetPositionUm, std::memory_order_release);
    commandedPositionValid_.store(true, std::memory_order_release);
    AgeMotionValue_.store(targetPositionUm, std::memory_order_release);
    emit sigAgeMotionValue(targetPositionUm);
    return true;
}

bool AgeMotionActor::getCommandedPosition(double& positionUm) const {
    if (!commandedPositionValid_.load(std::memory_order_acquire)) {
        return false;
    }
    positionUm = commandedPositionUm_.load(std::memory_order_acquire);
    return std::isfinite(positionUm);
}
