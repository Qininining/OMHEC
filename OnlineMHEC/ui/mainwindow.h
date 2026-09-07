#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts>
#include <QElapsedTimer>
#include <QFuture>
#include <QMutex>
#include <QString>
#include <QThread>
#include <atomic>

#include "MARobot.h"
#include "MediaSaver.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_MHEC_Task_clicked(bool checked);

    void on_ConnectCamera_clicked(bool checked);

    void on_StartCamera_clicked(bool checked);

    void on_expose_set_clicked();

    void on_SliderExpose_sliderMoved(int position);

    void on_StartUpdate_clicked();

    void on_linear_position_step_editingFinished();

    void on_angular_position_step_editingFinished();

    void on_Px_p_clicked();

    void on_Px_n_clicked();

    void on_Py_p_clicked();

    void on_Py_n_clicked();

    void on_Pz_p_clicked();

    void on_Pz_n_clicked();

    void on_Rx_p_clicked();

    void on_Rx_n_clicked();

    void on_Ry_p_clicked();

    void on_Ry_n_clicked();

    void on_Rz_p_clicked();

    void on_Rz_n_clicked();

    void on_MoveToZero_2_clicked();

    void on_Age_ref_p_clicked();

    void on_Age_ref_n_clicked();

    void on_setcurr2zero_clicked();

    void on_Age_P_p_clicked();

    void on_Age_P_n_clicked();

    void on_move2Zero_clicked();

    void on_AutoFocus10XTest_clicked();

    void on_showCircle_clicked(bool checked);

    void on_Age_ref_p_2_clicked();

    void on_Age_ref_p_3_clicked();

    void on_Age_ref_p_4_clicked();

    void on_showCircle_2_clicked(bool checked);

    void on_showCircle_3_clicked(bool checked);

    void on_FindReferance_clicked();

    void on_Stop_2_clicked();

    void on_RecordVideo_clicked(bool checked);
    void on_RecordProcessedVideo_clicked(bool checked);

    void onVideoRecordFinished(QString filePath, bool success);
    void onVideoRecordError(QString message);
    void onRecordingFrameHandled();
    void onRecordingStateChanged(bool isRecording, QString filePath);
    void onProcessedVideoRecordFinished(QString filePath, bool success);
    void onProcessedVideoRecordError(QString message);
    void onProcessedRecordingFrameHandled();
    void onProcessedRecordingStateChanged(bool isRecording, QString filePath);

public slots:
    void updateCamImage(QImage img);
    void refreshPreviewFrame();
    void updateRobotStatus(const  Eigen::VectorXd current_positions, const Eigen::VectorXd target_velocity, const Eigen::VectorXd current_pose);
    void updateFocusValue(double value);
    void updateAgeMotionValue(double value);
    void updateCircle(cv::Vec3d Circle);
    void updateTargetCircle(cv::Vec3d Circle);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    Ui::MainWindow *ui;

    MARobot* maRobot_;

    cv::Vec3d Circle_ = cv::Vec3d(-1,-1,-1);
    cv::Vec3d TargetCircle_ = cv::Vec3d(-1,-1,-1);
    cv::Vec3d InitCircle_ = cv::Vec3d(-1,-1,-1);

    bool showCircle_ = false;
    bool showTargetCircle_ = false;
    bool showInitCircle_ = false;

    double agemotionPos_ = 0;

    double angularPositionStep = 0;
    double linearPositionStep = 0;

    cv::Mat T_cs_est_ = cv::Mat::eye(4, 4, CV_64F);
    double angStep_ = 0.01;
    double scaleFactor_ = 562;

    QLineSeries *BlueSeries;
    QChart *chart;
    QValueAxis *axisX;
    QValueAxis *axisY;
    double m_chartXCounter = 0;
    void updateChartAxes();

    bool piHexapodPrepared_ = false;
    QFuture<void> autofocusTestFuture_;

signals:
    void sigStartVideoRecording(QString filePath, int fps, QString codec, bool isColor);
    void sigRecordFrame(QImage frame);
    void sigStopVideoRecording();
    void sigStartProcessedVideoRecording(QString filePath, int fps, QString codec, bool isColor);
    void sigRecordProcessedFrame(QImage frame);
    void sigStopProcessedVideoRecording();

private:
    QThread* mediaSaverThread_ = nullptr;
    MediaSaver* mediaSaver_ = nullptr;
    QThread* processedMediaSaverThread_ = nullptr;
    MediaSaver* processedMediaSaver_ = nullptr;
    QMutex previewFrameMutex_;
    std::atomic_bool previewRefreshPending_{false};
    std::atomic<quint64> previewFrameSequence_{0};
    QElapsedTimer previewRefreshThrottle_;
    QImage latestRawFrame_;
    QImage latestProcessedFrame_;
    bool isRecordingVideo_ = false;
    bool isProcessedRecordingVideo_ = false;
    int recordingFps_ = 30;
    qint64 recordingDroppedFrames_ = 0;
    qint64 processedRecordingDroppedFrames_ = 0;
    QElapsedTimer recordErrorTimer_;
    QString lastRecordErrorMessage_;
    QElapsedTimer processedRecordErrorTimer_;
    QString lastProcessedRecordErrorMessage_;
};
#endif // MAINWINDOW_H
