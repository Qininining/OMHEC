// media_saver.h
#ifndef MEDIA_SAVER_H
#define MEDIA_SAVER_H

#include <QObject>
#include <QImage>
#include <QString>
#include <QVector>
#include <QDebug>
#include <QSize>
#include <QMutex>
#include <QTimer>
#include <QtGlobal>
#include <atomic>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

/**
 * @brief MediaSaver class for saving QImage pictures and generating video from a QImage sequence.
 *
 * This class acts as the worker object that will be moved to a separate QThread.
 * Its public slots are designed to be executed in the thread it lives in.
 * Video saving uses the OpenCV library.
 */
class MediaSaver : public QObject {
    Q_OBJECT // Required for signals and slots

public:
    explicit MediaSaver(QObject *parent = nullptr);
    ~MediaSaver();

public slots:
    void saveImage(QImage image, QString filePath);
    void addFrame(QImage frame);
    void clearFrames();
    void saveVideo(QString filePath, int fps, QString codec = "MJPG", bool isColor = true);
    void save2Video(QVector<QImage> frames, QString filePath, int fps, QString codec = "MJPG", bool isColor = true);
    void startVideoRecording(QString filePath, int fps, QString codec = "MJPG", bool isColor = true);
    void updateRecordingFrame(const QImage& frame);
    void addFrameToRecording(const QImage& frame);
    void stopVideoRecording();

signals:
    void imageSaveFinished(QString filePath, bool success);
    void videoSaveFinished(QString filePath, bool success);
    void videoSaveProgress(int progress);
    void errorOccurred(QString message);
    void recordingStateChanged(bool isRecording, QString filePath);
    void recordingFrameHandled();
    void recordingFrameDropped(qint64 droppedFrames);

private:
    QVector<QImage> m_frameBuffer;
    std::atomic_bool m_isRecording{false};
    cv::VideoWriter m_videoWriter;
    QString m_recordingPath;
    QString m_recordingCodec = "MJPG";
    int m_recordingFps = 0;
    bool m_recordingIsColor = true;
    qint64 m_recordedFrameCount = 0;
    qint64 m_droppedFrameCount = 0;
    QSize m_recordingFrameSize;

    bool internalSaveVideo(const QVector<QImage>& framesToSave, QString filePath, int fps, QString codec, bool isColor);
    bool ensureRecordingWriterOpened(const QImage& firstFrame);
    static int buildFourcc(const QString& codec);
    cv::Mat qImageToCvMat(const QImage& qimage);
    void flushPendingRecordingFrame();

    QTimer* m_recordingTimer = nullptr;
    QMutex m_recordingFrameMutex;
    QImage m_pendingRecordingFrame;
    quint64 m_pendingRecordingFrameSeq = 0;
    quint64 m_writtenRecordingFrameSeq = 0;
};

#endif // MEDIA_SAVER_H
