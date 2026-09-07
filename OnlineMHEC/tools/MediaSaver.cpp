#include "MediaSaver.h"
#include <QDir>
#include <QPainter>
#include <QMutexLocker>
#include <QThread>

namespace {

static int recordingIntervalFromFps(int fps)
{
    return qMax(1, 1000 / qMax(1, fps));
}

} // namespace

MediaSaver::MediaSaver(QObject *parent) : QObject(parent) {
    m_recordingTimer = new QTimer(this);
    m_recordingTimer->setTimerType(Qt::CoarseTimer);
    connect(m_recordingTimer, &QTimer::timeout, this, &MediaSaver::flushPendingRecordingFrame);
}

MediaSaver::~MediaSaver() {
    if (m_isRecording.load(std::memory_order_relaxed)) {
        stopVideoRecording();
    }
}

void MediaSaver::saveImage(QImage image, QString filePath) {
    if (image.isNull()) {
        emit errorOccurred("Error: Attempting to save a null image.");
        emit imageSaveFinished(filePath, false);
        return;
    }
    bool success = image.save(filePath);
    if (!success) {
        emit errorOccurred("Error: Could not save image to " + filePath);
    }
    emit imageSaveFinished(filePath, success);
}

void MediaSaver::addFrame(QImage frame) {
    if (!frame.isNull()) {
        m_frameBuffer.append(frame);
    } else {
        qWarning() << "Warning: Attempted to add a null frame to buffer.";
    }
}

void MediaSaver::clearFrames() {
    m_frameBuffer.clear();
}

void MediaSaver::saveVideo(QString filePath, int fps, QString codec, bool isColor) {
    bool success = internalSaveVideo(m_frameBuffer, filePath, fps, codec, isColor);
    emit videoSaveFinished(filePath, success);
}

void MediaSaver::save2Video(QVector<QImage> frames, QString filePath, int fps, QString codec, bool isColor) {
    bool success = internalSaveVideo(frames, filePath, fps, codec, isColor);
    emit videoSaveFinished(filePath, success);
}

void MediaSaver::startVideoRecording(QString filePath, int fps, QString codec, bool isColor)
{
    if (m_isRecording.load(std::memory_order_relaxed)) {
        emit errorOccurred("Error: Recording has already started.");
        return;
    }
    if (filePath.isEmpty()) {
        emit errorOccurred("Error: Video save path is empty.");
        return;
    }
    if (fps <= 0) {
        emit errorOccurred("Error: Frame rate (FPS) must be greater than 0.");
        return;
    }

    m_recordingPath = filePath;
    m_recordingFps = fps;
    m_recordingCodec = codec;
    m_recordingIsColor = isColor;
    m_recordedFrameCount = 0;
    m_droppedFrameCount = 0;
    m_recordingFrameSize = QSize();
    m_pendingRecordingFrame = QImage();
    m_pendingRecordingFrameSeq = 0;
    m_writtenRecordingFrameSeq = 0;
    m_videoWriter.release();
    m_isRecording.store(true, std::memory_order_release);
    if (m_recordingTimer) {
        m_recordingTimer->start(recordingIntervalFromFps(m_recordingFps));
    }
    emit recordingStateChanged(true, m_recordingPath);
}

void MediaSaver::updateRecordingFrame(const QImage& frame)
{
    if (!m_isRecording.load(std::memory_order_acquire) || frame.isNull()) {
        return;
    }

    QMutexLocker locker(&m_recordingFrameMutex);
    m_pendingRecordingFrame = frame;
    ++m_pendingRecordingFrameSeq;
}

void MediaSaver::addFrameToRecording(const QImage& frame)
{
    updateRecordingFrame(frame);
}

void MediaSaver::flushPendingRecordingFrame()
{
    if (!m_isRecording.load(std::memory_order_acquire)) {
        return;
    }

    QImage frameToWrite;
    quint64 frameSeq = 0;
    {
        QMutexLocker locker(&m_recordingFrameMutex);
        if (m_pendingRecordingFrame.isNull()) {
            return;
        }
        if (m_pendingRecordingFrameSeq == m_writtenRecordingFrameSeq) {
            return;
        }
        frameToWrite = m_pendingRecordingFrame;
        frameSeq = m_pendingRecordingFrameSeq;
    }

    if (frameToWrite.isNull()) {
        ++m_droppedFrameCount;
        emit recordingFrameDropped(m_droppedFrameCount);
        return;
    }

    if (!ensureRecordingWriterOpened(frameToWrite)) {
        ++m_droppedFrameCount;
        emit recordingFrameDropped(m_droppedFrameCount);
        return;
    }

    cv::Mat cvFrame = qImageToCvMat(frameToWrite);
    if (cvFrame.empty()) {
        ++m_droppedFrameCount;
        emit recordingFrameDropped(m_droppedFrameCount);
        return;
    }

    if (m_recordingFrameSize.isValid() &&
        (cvFrame.cols != m_recordingFrameSize.width() || cvFrame.rows != m_recordingFrameSize.height())) {
        cv::resize(cvFrame,
                   cvFrame,
                   cv::Size(m_recordingFrameSize.width(), m_recordingFrameSize.height()),
                   0.0,
                   0.0,
                   cv::INTER_LINEAR);
    }

    if (m_recordingIsColor) {
        if (cvFrame.channels() == 1) {
            cv::cvtColor(cvFrame, cvFrame, cv::COLOR_GRAY2BGR);
        } else if (cvFrame.channels() == 4) {
            cv::cvtColor(cvFrame, cvFrame, cv::COLOR_BGRA2BGR);
        }
    } else {
        if (cvFrame.channels() == 3) {
            cv::cvtColor(cvFrame, cvFrame, cv::COLOR_BGR2GRAY);
        } else if (cvFrame.channels() == 4) {
            cv::cvtColor(cvFrame, cvFrame, cv::COLOR_BGRA2GRAY);
        }
    }

    if ((m_recordingIsColor && cvFrame.channels() != 3) ||
        (!m_recordingIsColor && cvFrame.channels() != 1)) {
        ++m_droppedFrameCount;
        emit recordingFrameDropped(m_droppedFrameCount);
        return;
    }

    m_videoWriter.write(cvFrame);
    ++m_recordedFrameCount;

    QMutexLocker locker(&m_recordingFrameMutex);
    m_writtenRecordingFrameSeq = frameSeq;
}

void MediaSaver::stopVideoRecording()
{
    if (!m_isRecording.load(std::memory_order_acquire)) {
        return;
    }

    const QString currentPath = m_recordingPath;
    const bool hasFrames = m_recordedFrameCount > 0;
    if (m_videoWriter.isOpened()) {
        m_videoWriter.release();
    }

    if (m_recordingTimer) {
        m_recordingTimer->stop();
    }

    m_isRecording.store(false, std::memory_order_release);
    m_recordingPath.clear();
    m_recordingCodec = "MJPG";
    m_recordingFps = 0;
    m_recordingIsColor = true;
    m_recordedFrameCount = 0;
    m_droppedFrameCount = 0;
    m_recordingFrameSize = QSize();

    emit recordingStateChanged(false, currentPath);
    emit videoSaveFinished(currentPath, hasFrames);
}

int MediaSaver::buildFourcc(const QString& codec)
{
    QString normalizedCodec = codec.trimmed().toUpper();
    if (normalizedCodec.size() != 4) {
        normalizedCodec = "MJPG";
    }
    return cv::VideoWriter::fourcc(normalizedCodec.at(0).toLatin1(),
                                   normalizedCodec.at(1).toLatin1(),
                                   normalizedCodec.at(2).toLatin1(),
                                   normalizedCodec.at(3).toLatin1());
}

bool MediaSaver::ensureRecordingWriterOpened(const QImage& firstFrame)
{
    if (m_videoWriter.isOpened()) {
        return true;
    }

    const int width = firstFrame.width();
    const int height = firstFrame.height();
    if (width <= 0 || height <= 0) {
        emit errorOccurred("Error: Invalid frame size for recording.");
        return false;
    }

    const int fourcc = buildFourcc(m_recordingCodec);
    m_videoWriter.open(m_recordingPath.toStdString(),
                       fourcc,
                       m_recordingFps,
                       cv::Size(width, height),
                       m_recordingIsColor);
    if (!m_videoWriter.isOpened()) {
        emit errorOccurred("Error: Could not open video writer for file: " + m_recordingPath);
        return false;
    }
    m_recordingFrameSize = QSize(width, height);
    return true;
}

bool MediaSaver::internalSaveVideo(const QVector<QImage>& framesToSave, QString filePath, int fps, QString codec, bool isColor) {
    if (framesToSave.isEmpty()) {
        emit errorOccurred("Error: No frames provided for video generation.");
        return false;
    }

    if (filePath.isEmpty()) {
        emit errorOccurred("Error: Video save path is empty.");
        return false;
    }

    if (fps <= 0) {
        emit errorOccurred("Error: Frame rate (FPS) must be greater than 0.");
        return false;
    }

    const QImage& firstFrame = framesToSave.first();
    int width = firstFrame.width();
    int height = firstFrame.height();

    int fourcc = buildFourcc(codec);
    if (fourcc == 0) {
        emit errorOccurred("Error: Invalid video codec provided: " + codec + ". Please check supported codecs for your OpenCV build. Attempting to use MJPG as fallback.");
        fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // Fallback
    }

    cv::VideoWriter videoWriter(filePath.toStdString(), fourcc, fps, cv::Size(width, height), isColor);

    if (!videoWriter.isOpened()) {
        emit errorOccurred("Error: Could not open video writer for file: " + filePath);
        return false;
    }

    for (int i = 0; i < framesToSave.size(); ++i) {
        const QImage& frame = framesToSave.at(i);
        if (frame.isNull()) {
            emit errorOccurred("Warning: Skipping null frame at index " + QString::number(i) + ".");
            continue;
        }

        cv::Mat cvFrame = qImageToCvMat(frame);
        if (cvFrame.empty()) {
            emit errorOccurred("Warning: Failed to convert QImage to cv::Mat for frame at index " + QString::number(i) + ". Skipping.");
            continue;
        }

        videoWriter.write(cvFrame);

        // Report progress
        int progress = (static_cast<double>(i + 1) / framesToSave.size()) * 100;
        emit videoSaveProgress(progress);
    }
    videoWriter.release();
    return true;
}

cv::Mat MediaSaver::qImageToCvMat(const QImage& qimage) {
    if (qimage.isNull()) {
        return cv::Mat();
    }

    // Attempt to convert indexed or unsupported formats to RGB888 first.
    // RGB888 is a common and direct mapping to OpenCV's BGR (on little-endian systems).
    QImage processedImage = qimage;
    if (qimage.format() != QImage::Format_ARGB32 &&
        qimage.format() != QImage::Format_ARGB32_Premultiplied &&
        qimage.format() != QImage::Format_RGB32 &&
        qimage.format() != QImage::Format_RGB888 &&
        qimage.format() != QImage::Format_Grayscale8)
    {
        // If it's an indexed format or other unsupported format, convert to RGB888
        // This handles QImage::Format_Indexed8 directly, removing the warning.
        processedImage = qimage.convertToFormat(QImage::Format_RGB888);
        if (processedImage.isNull()) {
            qWarning() << "Error: Failed to convert QImage format" << qimage.format() << "to Format_RGB888.";
            return cv::Mat();
        }
    }

    switch (processedImage.format()) {
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
        // Convert to 8-bit, 4-channel BGRA for OpenCV
        return cv::Mat(processedImage.height(), processedImage.width(), CV_8UC4, (void*)processedImage.constBits(), processedImage.bytesPerLine()).clone();
    case QImage::Format_RGB32: // In QImage, this is BGRX (on little-endian) or RGBX
        // For OpenCV, CV_8UC4 with QImage::Format_RGB32 often means BGRA or RGBA.
        // It's generally safe to map RGB32 directly to CV_8UC4.
        return cv::Mat(processedImage.height(), processedImage.width(), CV_8UC4, (void*)processedImage.constBits(), processedImage.bytesPerLine()).clone();
    case QImage::Format_RGB888: {// In QImage, this is BGR (on little-endian systems)
        cv::Mat mat(processedImage.height(), processedImage.width(), CV_8UC3,
                    const_cast<uchar*>(processedImage.bits()),
                    static_cast<size_t>(processedImage.bytesPerLine()));
        cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
        return mat.clone();
    }
    case QImage::Format_Grayscale8:
        return cv::Mat(processedImage.height(), processedImage.width(), CV_8UC1, (void*)processedImage.constBits(), processedImage.bytesPerLine()).clone();
    case  QImage::Format_Indexed8: {
        cv::Mat mat(processedImage.height(), processedImage.width(), CV_8UC1,
                    const_cast<uchar*>(processedImage.bits()),
                    processedImage.bytesPerLine());
        cv::Mat image;
        if (processedImage.colorTable().isEmpty()) {
            image =  mat.clone();
        }
        else{
            // 预计算调色板（BGR顺序）
            std::vector<cv::Vec3b> palette;
            for (QRgb rgb : processedImage.colorTable()) {
                palette.emplace_back(cv::Vec3b(qBlue(rgb), qGreen(rgb), qRed(rgb)));
            }

            // 应用调色板
            image =cv::Mat(mat.size(), CV_8UC3);
            for (int y = 0; y < mat.rows; ++y) {
                const uchar* src = mat.ptr<uchar>(y);
                cv::Vec3b* dst = image.ptr<cv::Vec3b>(y);
                for (int x = 0; x < mat.cols; ++x) {
                    dst[x] = palette[src[x]];
                }
            }
        }
        return image.clone();
    }
    default:
        // This case should ideally not be reached after the initial conversion attempt
        qWarning() << "Error: Unhandled QImage format" << processedImage.format() << "after conversion attempt.";
        return cv::Mat();
    }
}
