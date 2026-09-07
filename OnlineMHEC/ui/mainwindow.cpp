#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QMetaObject>
#include <QMessageBox>
#include <QMutexLocker>
#include <QImage>
#include <QFutureWatcher>
#include <QtConcurrent>

#include "modern_robotic_lib.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    maRobot_ = new MARobot();

    ui->label_pic->setCursor(Qt::CrossCursor);
    ui->label_pic->installEventFilter(this);

    this->angularPositionStep = 10000;
    this->linearPositionStep = 10000;
    this->ui->linear_position_step->setText(QString::number(this->angularPositionStep));
    this->ui->angular_position_step->setText(QString::number(this->linearPositionStep));

    this->ui->Agemotion_pos_step->setText(QString::number(50));

    BlueSeries = new QLineSeries();
    BlueSeries->setColor(Qt::blue);
    BlueSeries->setPointsVisible(false);
    chart = new QChart();
    chart->addSeries(BlueSeries);
    chart->legend()->hide(); // 隐藏图例
    chart->setTitle("Focus Sharpness Score"); // 修改标题：从 Force 改为 Focus

    axisX = new QValueAxis();
    axisX->setLabelFormat("%d"); // 显示整数帧数
    axisX->setTitleText("Frame");
    axisX->setTickCount(11); // 网格密度
    chart->addAxis(axisX, Qt::AlignBottom);
    BlueSeries->attachAxis(axisX);

    axisY = new QValueAxis();
    axisY->setTitleText("Score");
    axisY->setLabelFormat("%.1f"); // 保留一位小数
    chart->addAxis(axisY, Qt::AlignLeft);
    BlueSeries->attachAxis(axisY);

    // 4. 设置视图
    ui->charts->setChart(chart);
    ui->charts->setRenderHint(QPainter::Antialiasing); // 抗锯齿

    // 初始化计数器
    m_chartXCounter = 0;

    T_cs_est_ = (cv::Mat_<double>(4, 4) <<
                     0.13265873,			0.99116071,			0.00145262,			638845.62919304,
                 0.99115603,			-0.13265282,			-0.00359996,			92700.56088937,
                 -0.00337545,			0.00191734,			-0.99999247,			15481644.84992953,
                 0.00000000,			0.00000000,			0.00000000,			1.00000000);

    angStep_ = 10.0/180.0*M_PI;

    scaleFactor_ = 113.807;

    connect(this->maRobot_->camera_, &BaslerCameraControl::sigCurrentImage, this, &MainWindow::updateCamImage);
    connect(this->maRobot_->robot_, &ParallelPlatform::sig_updateRobotData, this, &MainWindow::updateRobotStatus);
    connect(this->maRobot_->motionActor_, &AgeMotionActor::sigFocusValue, this, &MainWindow::updateFocusValue);
    connect(this->maRobot_->motionActor_, &AgeMotionActor::sigAgeMotionValue, this, &MainWindow::updateAgeMotionValue);
    connect(this->maRobot_->motionActor_, &AgeMotionActor::sigCircle, this, &MainWindow::updateCircle);
    connect(this->maRobot_->mhecTask_, &MHECTask::sigTargetCircle, this, &MainWindow::updateTargetCircle);

    mediaSaver_ = new MediaSaver();
    mediaSaverThread_ = new QThread(this);
    mediaSaver_->moveToThread(mediaSaverThread_);
    connect(mediaSaverThread_, &QThread::finished, mediaSaver_, &QObject::deleteLater);
    connect(this, &MainWindow::sigStartVideoRecording, mediaSaver_, &MediaSaver::startVideoRecording, Qt::QueuedConnection);
    connect(this, &MainWindow::sigRecordFrame, mediaSaver_, &MediaSaver::addFrameToRecording, Qt::QueuedConnection);
    connect(this, &MainWindow::sigStopVideoRecording, mediaSaver_, &MediaSaver::stopVideoRecording, Qt::QueuedConnection);
    connect(this->maRobot_->camera_, &BaslerCameraControl::sigCurrentImage, mediaSaver_, &MediaSaver::updateRecordingFrame, Qt::DirectConnection);
    connect(mediaSaver_, &MediaSaver::videoSaveFinished, this, &MainWindow::onVideoRecordFinished, Qt::QueuedConnection);
    connect(mediaSaver_, &MediaSaver::errorOccurred, this, &MainWindow::onVideoRecordError, Qt::QueuedConnection);
    connect(mediaSaver_, &MediaSaver::recordingFrameHandled, this, &MainWindow::onRecordingFrameHandled, Qt::QueuedConnection);
    connect(mediaSaver_, &MediaSaver::recordingStateChanged, this, &MainWindow::onRecordingStateChanged, Qt::QueuedConnection);
    connect(mediaSaver_, &MediaSaver::recordingFrameDropped, this, [this](qint64 droppedFrames) {
        recordingDroppedFrames_ = droppedFrames;
    }, Qt::QueuedConnection);
    mediaSaverThread_->start();

    processedMediaSaver_ = new MediaSaver();
    processedMediaSaverThread_ = new QThread(this);
    processedMediaSaver_->moveToThread(processedMediaSaverThread_);
    connect(processedMediaSaverThread_, &QThread::finished, processedMediaSaver_, &QObject::deleteLater);
    connect(this, &MainWindow::sigStartProcessedVideoRecording, processedMediaSaver_, &MediaSaver::startVideoRecording, Qt::QueuedConnection);
    connect(this, &MainWindow::sigRecordProcessedFrame, processedMediaSaver_, &MediaSaver::addFrameToRecording, Qt::QueuedConnection);
    connect(this, &MainWindow::sigStopProcessedVideoRecording, processedMediaSaver_, &MediaSaver::stopVideoRecording, Qt::QueuedConnection);
    connect(processedMediaSaver_, &MediaSaver::videoSaveFinished, this, &MainWindow::onProcessedVideoRecordFinished, Qt::QueuedConnection);
    connect(processedMediaSaver_, &MediaSaver::errorOccurred, this, &MainWindow::onProcessedVideoRecordError, Qt::QueuedConnection);
    connect(processedMediaSaver_, &MediaSaver::recordingFrameHandled, this, &MainWindow::onProcessedRecordingFrameHandled, Qt::QueuedConnection);
    connect(processedMediaSaver_, &MediaSaver::recordingStateChanged, this, &MainWindow::onProcessedRecordingStateChanged, Qt::QueuedConnection);
    connect(processedMediaSaver_, &MediaSaver::recordingFrameDropped, this, [this](qint64 droppedFrames) {
        processedRecordingDroppedFrames_ = droppedFrames;
    }, Qt::QueuedConnection);
    processedMediaSaverThread_->start();

    previewRefreshThrottle_.start();

}

MainWindow::~MainWindow()
{
    if (autofocusTestFuture_.isRunning()) {
        autofocusTestFuture_.waitForFinished();
    }
    if (mediaSaver_) {
        QMetaObject::invokeMethod(mediaSaver_, "stopVideoRecording", Qt::BlockingQueuedConnection);
    }
    if (processedMediaSaver_) {
        QMetaObject::invokeMethod(processedMediaSaver_, "stopVideoRecording", Qt::BlockingQueuedConnection);
    }
    if (mediaSaverThread_) {
        mediaSaverThread_->quit();
        mediaSaverThread_->wait();
        mediaSaverThread_ = nullptr;
        mediaSaver_ = nullptr;
    }
    if (processedMediaSaverThread_) {
        processedMediaSaverThread_->quit();
        processedMediaSaverThread_->wait();
        processedMediaSaverThread_ = nullptr;
        processedMediaSaver_ = nullptr;
    }
    delete maRobot_;
    delete ui;
}

void MainWindow::updateCamImage(QImage img)
{
    QMutexLocker locker(&previewFrameMutex_);
    latestRawFrame_ = img;
    locker.unlock();

    previewFrameSequence_.fetch_add(1, std::memory_order_relaxed);

    const int previewIntervalMs = (isRecordingVideo_ || isProcessedRecordingVideo_) ? 140 : 66;
    if (previewRefreshThrottle_.isValid() && previewRefreshThrottle_.elapsed() < previewIntervalMs) {
        return;
    }

    if (!previewRefreshPending_.exchange(true)) {
        previewRefreshThrottle_.restart();
        QMetaObject::invokeMethod(this, "refreshPreviewFrame", Qt::QueuedConnection);
    }
}

void MainWindow::refreshPreviewFrame()
{
    const quint64 frameSequenceAtStart = previewFrameSequence_.load(std::memory_order_relaxed);
    previewRefreshPending_.store(false);

    QImage rawFrame;
    {
        QMutexLocker locker(&previewFrameMutex_);
        if (latestRawFrame_.isNull()) {
            return;
        }
        rawFrame = latestRawFrame_;
    }

    QImage processedImage = rawFrame;
    QPainter painter(&processedImage);

    if (showCircle_) {
        QPen pen(Qt::red, 3);
        painter.setPen(pen);
        float centerX = Circle_[0];
        float centerY = Circle_[1];
        float radius  = Circle_[2];
        painter.drawPoint(QPointF(centerX, centerY));
        painter.drawEllipse(QPointF(centerX, centerY), radius, radius);
        // painter.end(); // 完成绘制
    }

    if (showTargetCircle_) {
        QPen pen(Qt::green, 3);
        painter.setPen(pen);
        float centerX = TargetCircle_[0];
        float centerY = TargetCircle_[1];
        float radius  = TargetCircle_[2];
        painter.drawPoint(QPointF(centerX, centerY));
        painter.drawEllipse(QPointF(centerX, centerY), radius, radius);
        // painter.end(); // 完成绘制
    }

    if (showInitCircle_) {
        QPen pen(Qt::yellow, 3);
        painter.setPen(pen);
        float centerX = InitCircle_[0];
        float centerY = InitCircle_[1];
        float radius  = InitCircle_[2];
        painter.drawPoint(QPointF(centerX, centerY));
        painter.drawEllipse(QPointF(centerX, centerY), radius, radius);
    }
    if (true) {
        float centerX = Circle_[0];
        float centerY = Circle_[1];
        float rectWidth = 500;
        float rectHeight = 500;
        float rectX = centerX - rectWidth / 2;
        float rectY = centerY - rectHeight / 2;
        QPen rectPen(Qt::green, 5);
        painter.setPen(rectPen);
        painter.drawRect(QRectF(rectX, rectY, rectWidth, rectHeight));
    }
    painter.end(); // 完成绘制

    {
        QMutexLocker locker(&previewFrameMutex_);
        latestProcessedFrame_ = processedImage;
    }

    if (isProcessedRecordingVideo_ && processedMediaSaver_) {
        processedMediaSaver_->updateRecordingFrame(processedImage);
    }

    QPixmap pix = QPixmap::fromImage(processedImage);
    ui->label_pic->setPixmap(pix.scaled(ui->label_pic->size(), Qt::KeepAspectRatio));

    // 计算频率
    static int frameCount = 0;
    static QElapsedTimer timer;
    frameCount++;
    if (!timer.isValid()) {
        timer.start();
    }
    if (timer.elapsed() >= 1000) { // 每秒更新一次
        int fps = frameCount * 1000 / timer.elapsed();
        ui->label_fps->setText(QString::number(fps) + " FPS");
        frameCount = 0;
        timer.restart();
    }

    if (previewFrameSequence_.load(std::memory_order_relaxed) != frameSequenceAtStart &&
        !previewRefreshPending_.exchange(true)) {
        previewRefreshThrottle_.restart();
        QMetaObject::invokeMethod(this, "refreshPreviewFrame", Qt::QueuedConnection);
    }

}
void MainWindow::updateRobotStatus(const  Eigen::VectorXd current_positions, const Eigen::VectorXd target_velocity, const Eigen::VectorXd current_pose)
{
    // Pose
    ui->current_pos_x->setText(QString::number(current_pose(0)));
    ui->current_pos_y->setText(QString::number(current_pose(1)));
    ui->current_pos_z->setText(QString::number(current_pose(2)));
    ui->current_pos_rx->setText(QString::number(current_pose(3)));
    ui->current_pos_ry->setText(QString::number(current_pose(4)));
    ui->current_pos_rz->setText(QString::number(current_pose(5)));
    // Velocity
    ui->current_vec_x->setText(QString::number(target_velocity(0)));
    ui->current_vec_y->setText(QString::number(target_velocity(1)));
    ui->current_vec_z->setText(QString::number(target_velocity(2)));
    ui->current_vec_rx->setText(QString::number(target_velocity(3)));
    ui->current_vec_ry->setText(QString::number(target_velocity(4)));
    ui->current_vec_rz->setText(QString::number(target_velocity(5)));
    // Position
    ui->current_joint_pos_1->setText(QString::number(current_positions(0)));
    ui->current_joint_pos_2->setText(QString::number(current_positions(1)));
    ui->current_joint_pos_3->setText(QString::number(current_positions(2)));
    ui->current_joint_pos_4->setText(QString::number(current_positions(3)));
    ui->current_joint_pos_5->setText(QString::number(current_positions(4)));
    ui->current_joint_pos_6->setText(QString::number(current_positions(5)));

    static int frameCount = 0;
    static QElapsedTimer timer;
    frameCount++;
    if (!timer.isValid()) {
        timer.start();
    }
    if (timer.elapsed() >= 1000) { // 每秒更新一次
        int fps = frameCount * 1000 / timer.elapsed();
        ui->label_fps_2->setText(QString::number(fps) + " FPS");
        frameCount = 0;
        timer.restart();
    }
}

void MainWindow::updateFocusValue(double value)
{
    // ui->focus_value_label->setText(QString::number(value, 'f', 2));

    // 更新图表数据
    BlueSeries->append(m_chartXCounter, value);
    m_chartXCounter += 1;

    // 每次添加新数据后，更新坐标轴范围
    updateChartAxes();

    static int frameCount = 0;
    static QElapsedTimer timer;
    frameCount++;
    if (!timer.isValid()) {
        timer.start();
    }
    if (timer.elapsed() >= 1000) { // 每秒更新一次
        int fps = frameCount * 1000 / timer.elapsed();
        ui->label_fps_3->setText(QString::number(fps) + " FPS");
        frameCount = 0;
        timer.restart();
    }

    ui->statusbar->showMessage(QString("Current Sharpness: %1").arg(value));
}
void MainWindow::updateAgeMotionValue(double value) {
    agemotionPos_ =  -1000.0 * value;
    // The autofocus stage is configured on a 0.01 um command grid.
    ui->Agemotion_pos->setText(QString::number(value, 'f', 2));
}

void MainWindow::updateCircle(cv::Vec3d Circle) {
    Circle_ = Circle; // u、v、r

    float centerX = Circle_[0];
    float centerY = Circle_[1];
    // float radius  = Circle_[2];

    QString circleText = "(" +
                         QString::number(centerX, 'g', 6) + ", \t" + // 'g'格式自动选择定点或科学计数法，6是有效数字位数
                         QString::number(centerY, 'g', 6) + ")";
    ui->label_circle->setText(circleText);

    // 计算频率
    static double frameCount = 0;
    static QElapsedTimer timer;
    frameCount++;
    if (!timer.isValid()) {
        timer.start();
    }
    if (timer.elapsed() >= 1000) { // 每秒更新一次
        double fps = frameCount * 1000 / timer.elapsed();
        ui->label_fps_4->setText(QString::number(fps) + " FPS");
        frameCount = 0;
        timer.restart();
    }
}


void MainWindow::updateTargetCircle(cv::Vec3d Circle) {
    TargetCircle_ = Circle; // u、v、r
}


void MainWindow::updateChartAxes()
{
    // 定义图表显示的最大数据点数（窗口宽度）
    const int MAX_DISPLAY_POINTS = 100;

    // --- X轴管理 (滚动效果) ---
    if (m_chartXCounter > MAX_DISPLAY_POINTS) {
        // 如果数据点超过限制，X轴向右移动，形成滚动效果
        axisX->setRange(m_chartXCounter - MAX_DISPLAY_POINTS, m_chartXCounter);

        // 为了防止内存无限增长，移除太旧的数据点
        // 注意：remove(0) 操作虽然可以省内存，但在大量高速数据下可能耗时。
        // 对于几百个点，直接 remove 是安全的。
        if (BlueSeries->count() > MAX_DISPLAY_POINTS + 10) {
            BlueSeries->remove(0);
        }
    } else {
        // 数据还未填满屏幕时，固定从0开始
        axisX->setRange(0, MAX_DISPLAY_POINTS);
    }

    // --- Y轴管理 (自适应高度) ---
    // 为了让波形看起来更明显，我们需要动态调整Y轴的最小值和最大值
    if (BlueSeries->count() > 0) {
        double minVal = 1e9;
        double maxVal = -1e9;

        // 获取当前显示窗口内数据的极值
        QList<QPointF> points = BlueSeries->points();
        for (const QPointF &p : points) {
            if (p.y() < minVal) minVal = p.y();
            if (p.y() > maxVal) maxVal = p.y();
        }

        // 如果还没数据或极值相等，给一个默认范围
        if (minVal >= maxVal) {
            maxVal = minVal + 10.0;
            minVal = minVal - 10.0;
        }

        // 给上下留一点 Padding (例如 10%)，这样曲线不会顶格显示，更好看
        double padding = (maxVal - minVal) * 0.1;

        // 只有当变化较大时才更新Y轴，避免画面闪烁
        axisY->setRange(minVal - padding, maxVal + padding);
    }
}




// 然后在主窗口类中实现事件过滤器
bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->label_pic)
    {
        QWidget* targetWidget = ui->label_pic;
        if (event->type() == QEvent::MouseMove || event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            QPoint pos = mouseEvent->pos();

            // 计算缩放比例
            static const QSize originalImageSize(2448, 2048);
            double scaleX = static_cast<double>(originalImageSize.width()) / targetWidget->width();
            double scaleY = static_cast<double>(originalImageSize.height()) / targetWidget->height();

            // 转换为像素坐标
            int pixel_U = pos.x() * scaleX;
            int pixel_V = pos.y() * scaleY;


            // --- 定义 T_cs ---
            cv::Mat T_cs_est = T_cs_est_;

            cv::Mat T_sc_est = T_cs_est.inv();
            double scaleFactor = scaleFactor_;

            double Z = 0;
            this->maRobot_->motionActor_->getCommandedPosition(Z);
            Z = -Z * 1000;
            double X = (pixel_U - originalImageSize.width() / 2) * scaleFactor;
            double Y = (pixel_V - originalImageSize.height() / 2) * scaleFactor;


            cv::Mat p_camera;
            p_camera  = (cv::Mat_<double>(3, 1) << X, Y, Z);

            qDebug().nospace() << "[Motion] p_camera (Camera Frame Point): ["
                               << p_camera.at<double>(0) << ", " << p_camera.at<double>(1) << ", " << p_camera.at<double>(2) << "]";


            // // 旋转验证
            // int isInv = 1 + ui->checkBox_inv->isChecked() * -2;
            // cv::Mat omega_c;
            // // omega_c << 0.01 * isInv , 0.0, 0.0;
            // omega_c = (cv::Mat_<double>(3, 1) << 0, 0.08 * isInv, 0);
            // // omega_c <<  0.0, 0.01 * isInv, 0.0;
            // // omega_c << 0.0, 0.0, 0.01 * isInv;
            // // omega_c = (cv::Mat_<double>(3, 1) << 0, 0, 0);

            // cv::Mat v_c = p_camera.cross(omega_c);

            // cv::Mat S_c = cv::Mat::zeros(6, 1, CV_64F);
            // S_c.at<double>(0) = omega_c.at<double>(0, 0);
            // S_c.at<double>(1) = omega_c.at<double>(1, 0);
            // S_c.at<double>(2) = omega_c.at<double>(2, 0);
            // S_c.at<double>(3) = v_c.at<double>(0, 0);
            // S_c.at<double>(4) = v_c.at<double>(1, 0);
            // S_c.at<double>(5) = v_c.at<double>(2, 0);
            // qDebug().nospace() << "[Motion] Final Twist S_c: "
            //                    << "w[" << S_c.at<double>(0) << " " << S_c.at<double>(1) << " " << S_c.at<double>(2) << "] | "
            //                    << "v[" << S_c.at<double>(3) << " " << S_c.at<double>(4) << " " << S_c.at<double>(5) << "]";

            // cv::Mat Ad_T_sc = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est)));
            // cv::Mat V_end_effector = Ad_T_sc * S_c;

            // cv::Mat T_increment = cv::Mat::eye(4, 4, CV_64F);
            // T_increment = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(V_end_effector))));

            // cv::Mat T_se_curr = this->maRobot_->robot_->getTaskMat_cv();
            // cv::Mat T_next = T_increment * T_se_curr;

            // this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
            // this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_next));


            // // // 位移验证
            // float centerX = Circle_[0];
            // float centerY = Circle_[1];
            // centerX = (centerX - originalImageSize.width() / 2) * scaleFactor;
            // centerY = (centerY - originalImageSize.height() / 2) * scaleFactor;
            // double centerZ = 0;
            // this->maRobot_->motionActor_->MotonDriver_->getPosition(centerZ);
            // centerZ = -centerZ * 1000;

            // cv::Mat T_se_curr = this->maRobot_->robot_->getTaskMat_cv();
            // cv::Mat T_es = T_se_curr.inv();

            // cv::Mat p_camera_homo = (cv::Mat_<double>(4, 1) <<
            //                             centerX,
            //                             centerY,
            //                             centerZ,
            //                             1.0);

            // cv::Mat p_space_homo = T_sc_est * p_camera_homo;
            // cv::Mat p_eo_homo = T_es * p_space_homo;

            // cv::Mat t_eo = (cv::Mat_<double>(3, 1) <<
            //                     p_eo_homo.at<double>(0),
            //                     p_eo_homo.at<double>(1),
            //                     p_eo_homo.at<double>(2));

            // // cv::Mat t_eo = (cv::Mat_<double>(3, 1) <<
            // //                     -5774231,
            // //                     -10865387,
            // //                     5577472);

            // qDebug().nospace() << "[Motion] t_eo (Object Translation in EE Frame): ["
            //                    << QString::number(t_eo.at<double>(0), 'f', 2) << ", "
            //                    << QString::number(t_eo.at<double>(1), 'f', 2) << ", "
            //                    << QString::number(t_eo.at<double>(2), 'f', 2) << "]";

            // cv::Mat T_eo = cv::Mat::eye(4, 4, CV_64F);
            // T_eo.at<double>(0, 3) = t_eo.at<double>(0);
            // T_eo.at<double>(1, 3) = t_eo.at<double>(1);
            // T_eo.at<double>(2, 3) = t_eo.at<double>(2);
            // cv::Mat T_oe = T_eo.inv();

            // cv::Mat p_c_target_homo = (cv::Mat_<double>(4, 1) <<
            //                         p_camera.at<double>(0),
            //                         p_camera.at<double>(1),
            //                         p_camera.at<double>(2),
            //                         1.0);
            // cv::Mat p_space_target_homo = T_sc_est * p_c_target_homo;

            // cv::Mat T_so_next = T_se_curr.clone();
            // T_so_next.at<double>(0, 3) = p_space_target_homo.at<double>(0);
            // T_so_next.at<double>(1, 3) = p_space_target_homo.at<double>(1);
            // T_so_next.at<double>(2, 3) = p_space_target_homo.at<double>(2);

            // cv::Mat T_se_next = T_so_next * T_oe;

            // this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
            // this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_se_next));
            // // QThread::msleep(2000);
            // // this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_se_next));

            // return true;  // 事件已处理




            // // 位移验证
            float centerX = Circle_[0];
            float centerY = Circle_[1];
            centerX = (centerX - originalImageSize.width() / 2) * scaleFactor;
            centerY = (centerY - originalImageSize.height() / 2) * scaleFactor;
            double centerZ = 0;
            this->maRobot_->motionActor_->getCommandedPosition(centerZ);
            centerZ = -centerZ * 1000;

            cv::Mat T_se_curr = this->maRobot_->robot_->getTaskMat_cv();
            cv::Mat T_es = T_se_curr.inv();

            cv::Mat p_camera_homo = (cv::Mat_<double>(4, 1) <<
                                         centerX,
                                     centerY,
                                     centerZ,
                                     1.0);

            cv::Mat p_c_target_homo = (cv::Mat_<double>(4, 1) <<
                                           p_camera.at<double>(0),
                                       p_camera.at<double>(1),
                                       p_camera.at<double>(2),
                                       1.0);

            cv::Mat delta_t_co = (cv::Mat_<double>(3, 1) <<
                                      p_c_target_homo.at<double>(0) - p_camera_homo.at<double>(0),
                                  p_c_target_homo.at<double>(1) - p_camera_homo.at<double>(1),
                                  p_c_target_homo.at<double>(2) - p_camera_homo.at<double>(2));

            qDebug().nospace() << "[Motion] delta_t_co (Displacement in Camera Frame): ["
                               << QString::number(delta_t_co.at<double>(0), 'f', 2) << ", "
                               << QString::number(delta_t_co.at<double>(1), 'f', 2) << ", "
                               << QString::number(delta_t_co.at<double>(2), 'f', 2) << "]";

            cv::Mat R_sc = T_sc_est(cv::Rect(0, 0, 3, 3));

            cv::Mat delta_t_se = R_sc * delta_t_co;

            cv::Mat T_se_next = T_se_curr.clone();

            T_se_next.at<double>(0, 3) += delta_t_se.at<double>(0);
            T_se_next.at<double>(1, 3) += delta_t_se.at<double>(1);
            T_se_next.at<double>(2, 3) += delta_t_se.at<double>(2);

            this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
            this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_se_next));
            // QThread::msleep(2000);
            // this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_se_next));

            return true;  // 事件已处理
        }
    }
    return QMainWindow::eventFilter(watched, event);
}


void MainWindow::on_pushButton_MHEC_Task_clicked(bool checked)
{
    if (checked) {
        maRobot_->startMHECTask();
        ui->pushButton_MHEC_Task->setText("Stop MHEC Task");
    } else {
        maRobot_->stopMHECTask();
        ui->pushButton_MHEC_Task->setText("Start MHEC Task");
    }
}


void MainWindow::on_ConnectCamera_clicked(bool checked)
{
    if(checked)
    {
        const auto cameras = this->maRobot_->camera_->cameras(); // 获取相机列表

        if (!cameras.isEmpty()) { // 检查是否至少存在一个相机
            const QString& firstCameraSN = cameras.first(); // 获取第一个相机序列号
            if (!this->maRobot_->camera_->openCamera(firstCameraSN)) { // 尝试打开相机
                qDebug() << "成功打开相机: " << firstCameraSN;
                ui->label_pic->setText("成功打开相机！");
            } else {
                qCritical() << "无法打开相机: " << firstCameraSN;
                ui->label_pic->setText("无法打开相机!");
            }
        } else {
            qWarning() << "未检测到任何相机！";
            ui->label_pic->setText("未检测到相机，请连接设备！");
        }
        ui->ConnectCamera->setText("Disconnect");// 断开连接

        int exposureTime =  this->maRobot_->camera_->getExposureTime();
        this->ui->lineEdit_expose_->setText(QString::number(exposureTime));
        this->ui->SliderExpose->setMaximum(exposureTime * 2);
        this->ui->SliderExpose->setValue(exposureTime);
    }
    else{
        this->maRobot_->camera_->closeCamera();
        ui->label_pic->setText("相机已断开！");
        ui->ConnectCamera->setText("ConnectCamera");// 连接相机
    }
}


void MainWindow::on_StartCamera_clicked(bool checked)
{
    if(checked) {// 开始采集
        this->maRobot_->camera_->StartAcquire();
        ui->ConnectCamera->setText("StopCamera");// 结束采集
        qDebug() << "开始采集";
    } else {
        this->maRobot_->camera_->StopAcquire();
        ui->ConnectCamera->setText("StartCamera");// 开始采集
    }
}


void MainWindow::on_expose_set_clicked()
{
    bool ok;
    double expouse =  ui->lineEdit_expose_->text().toInt(&ok);
    if(!ok){
        QMessageBox::warning(this, "Error", "input is error !!");
    }
    else{
        this->maRobot_->camera_->setExposureTime(expouse);
    }
}


void MainWindow::on_SliderExpose_sliderMoved(int position)
{
    int exposeSize = 10000;
    ui->SliderExpose->setRange(0,exposeSize);
    // int exposeValue = double(position * exposeSize) / 100;
    if(position <= 10)
    {
        position = 10;
    }
    ui->lineEdit_expose_->setText(QString::number(position,'f',0));
    on_expose_set_clicked();
}


void MainWindow::on_StartUpdate_clicked()
{
    this->maRobot_->robot_->startupdate();
    // ui->FindReferance->setEnabled(false);
}


void MainWindow::on_linear_position_step_editingFinished()
{
    QString text = this->ui->linear_position_step->text();
    this->linearPositionStep = text.toDouble();
    // qDebug() << "linear_position_step: " << this->linearPositionStep;
}


void MainWindow::on_angular_position_step_editingFinished()
{
    QString text = this->ui->angular_position_step->text();
    this->angularPositionStep = text.toDouble();
    // qDebug() << "angular_position_step: " << this->angularPositionStep;
}



void MainWindow::on_Px_p_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(0) += linearPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Px_n_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(0) -= linearPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Py_p_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(1) += linearPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Py_n_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(1) -= linearPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Pz_p_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(2) += linearPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Pz_n_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(2) -= linearPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rx_p_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(3) += angularPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rx_n_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(3) -= angularPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Ry_p_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(4) += angularPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Ry_n_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(4) -= angularPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rz_p_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(5) += angularPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rz_n_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions;
    bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    if(result){
        target_positions(5) -= angularPositionStep;
        this->maRobot_->robot_->setTargetPose(target_positions);
    }
}

void MainWindow::on_MoveToZero_2_clicked()
{
    // this->maRobot_->robot_->setRobotMode(ROBOT_MOVE_TO_ZERO);
    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    Eigen::VectorXd target_positions = Eigen::VectorXd::Zero(6);
    // bool result = this->maRobot_->robot_->getTaskPositions(target_positions);
    this->maRobot_->robot_->setTargetPose(target_positions);
}

void MainWindow::on_Age_ref_p_clicked()
{
    this->maRobot_->motionActor_->findReference(true);
}


void MainWindow::on_Age_ref_n_clicked()
{
    this->maRobot_->motionActor_->findReference(false);
}


void MainWindow::on_setcurr2zero_clicked()
{
    this->maRobot_->motionActor_->setCurrPositionToZero();
}


void MainWindow::on_Age_P_p_clicked()
{
    QString text = this->ui->Agemotion_pos_step->text();
    double step = text.toDouble();
    this->maRobot_->motionActor_->setRelativePosition(step);
}


void MainWindow::on_Age_P_n_clicked()
{
    QString text = this->ui->Agemotion_pos_step->text();
    double step = text.toDouble();
    this->maRobot_->motionActor_->setRelativePosition(-step);
}


void MainWindow::on_move2Zero_clicked()
{
    this->maRobot_->motionActor_->setTargetPosition(0);
}

void MainWindow::on_AutoFocus10XTest_clicked()
{
    if (autofocusTestFuture_.isRunning()) {
        return;
    }

    ui->AutoFocus10XTest->setEnabled(false);
    ui->AutoFocus10XTest->setText("10X Focusing...");
    BlueSeries->clear();
    m_chartXCounter = 0;
    ui->statusbar->showMessage("10X autofocus test is running...");

    AgeMotionActor* const motionActor = maRobot_->motionActor_;
    constexpr double scale10XNmPerPixel = 338.640;
    autofocusTestFuture_ = QtConcurrent::run([motionActor, scale10XNmPerPixel]() {
        motionActor->AutoFocusOnce(scale10XNmPerPixel, true);
    });

    auto* watcher = new QFutureWatcher<void>(this);
    connect(watcher, &QFutureWatcher<void>::finished, this, [this, watcher]() {
        double finalPositionUm = 0.0;
        const bool hasPosition =
            maRobot_->motionActor_->getCommandedPosition(finalPositionUm);
        ui->AutoFocus10XTest->setEnabled(true);
        ui->AutoFocus10XTest->setText("10X AutoFocus Test");
        ui->statusbar->showMessage(
            hasPosition
                ? QString("10X autofocus test finished at %1 um; inspect the image and focus curve.")
                      .arg(finalPositionUm, 0, 'f', 2)
                : QString("10X autofocus test finished; position is unavailable."),
            10000);
        watcher->deleteLater();
    });
    watcher->setFuture(autofocusTestFuture_);
}


void MainWindow::on_showCircle_clicked(bool checked)
{
    showCircle_ = checked;
}


void MainWindow::on_Age_ref_p_2_clicked()
{
    QWidget* targetWidget = ui->label_pic;

    cv::Mat T_cs_est = T_cs_est_;
    cv::Mat T_sc_est = T_cs_est.inv();

    static const QSize originalImageSize(2448, 2048);
    double scaleX = static_cast<double>(originalImageSize.width()) / targetWidget->width();
    double scaleY = static_cast<double>(originalImageSize.height()) / targetWidget->height();
    double scaleFactor = scaleFactor_;

    float centerX = Circle_[0];
    float centerY = Circle_[1];
    centerX = (centerX - originalImageSize.width() / 2) * scaleFactor;
    centerY = (centerY - originalImageSize.height() / 2) * scaleFactor;
    double centerZ = 0;
    this->maRobot_->motionActor_->getCommandedPosition(centerZ);
    centerZ = -centerZ * 1000;

    cv::Mat T_se_curr = this->maRobot_->robot_->getTaskMat_cv();
    cv::Mat T_es = T_se_curr.inv();

    cv::Mat p_camera_homo = (cv::Mat_<double>(4, 1) <<
                                 centerX,
                             centerY,
                             centerZ,
                             1.0);

    cv::Mat p_camera = (cv::Mat_<double>(3, 1) <<
                            centerX,
                        centerY,
                        centerZ);




    int isInv = 1 + ui->checkBox_inv->isChecked() * -2;
    cv::Mat omega_c;
    omega_c = (cv::Mat_<double>(3, 1) << angStep_ * isInv, 0, 0);
    cv::Mat v_c = p_camera.cross(omega_c);
    cv::Mat S_c = cv::Mat::zeros(6, 1, CV_64F);
    S_c.at<double>(0) = omega_c.at<double>(0, 0);
    S_c.at<double>(1) = omega_c.at<double>(1, 0);
    S_c.at<double>(2) = omega_c.at<double>(2, 0);
    S_c.at<double>(3) = v_c.at<double>(0, 0);
    S_c.at<double>(4) = v_c.at<double>(1, 0);
    S_c.at<double>(5) = v_c.at<double>(2, 0);
    qDebug().nospace() << "[Motion] Final Twist S_c: "
                       << "w[" << S_c.at<double>(0) << " " << S_c.at<double>(1) << " " << S_c.at<double>(2) << "] | "
                       << "v[" << S_c.at<double>(3) << " " << S_c.at<double>(4) << " " << S_c.at<double>(5) << "]";

    cv::Mat Ad_T_sc = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est)));
    cv::Mat V_end_effector = Ad_T_sc * S_c;

    cv::Mat T_increment = cv::Mat::eye(4, 4, CV_64F);
    T_increment = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(V_end_effector))));

    cv::Mat T_se_next = T_increment * T_se_curr;


    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_se_next));
}


void MainWindow::on_Age_ref_p_3_clicked()
{
    QWidget* targetWidget = ui->label_pic;

    cv::Mat T_cs_est = T_cs_est_;
    cv::Mat T_sc_est = T_cs_est.inv();

    static const QSize originalImageSize(2448, 2048);
    double scaleX = static_cast<double>(originalImageSize.width()) / targetWidget->width();
    double scaleY = static_cast<double>(originalImageSize.height()) / targetWidget->height();
    double scaleFactor = scaleFactor_;

    float centerX = Circle_[0];
    float centerY = Circle_[1];
    centerX = (centerX - originalImageSize.width() / 2) * scaleFactor;
    centerY = (centerY - originalImageSize.height() / 2) * scaleFactor;
    double centerZ = 0;
    this->maRobot_->motionActor_->getCommandedPosition(centerZ);
    centerZ = -centerZ * 1000;

    cv::Mat T_se_curr = this->maRobot_->robot_->getTaskMat_cv();
    cv::Mat T_es = T_se_curr.inv();

    cv::Mat p_camera_homo = (cv::Mat_<double>(4, 1) <<
                                 centerX,
                             centerY,
                             centerZ,
                             1.0);

    cv::Mat p_camera = (cv::Mat_<double>(3, 1) <<
                            centerX,
                        centerY,
                        centerZ);




    int isInv = 1 + ui->checkBox_inv->isChecked() * -2;
    cv::Mat omega_c;
    omega_c = (cv::Mat_<double>(3, 1) << 0, angStep_ * isInv, 0);
    cv::Mat v_c = p_camera.cross(omega_c);
    cv::Mat S_c = cv::Mat::zeros(6, 1, CV_64F);
    S_c.at<double>(0) = omega_c.at<double>(0, 0);
    S_c.at<double>(1) = omega_c.at<double>(1, 0);
    S_c.at<double>(2) = omega_c.at<double>(2, 0);
    S_c.at<double>(3) = v_c.at<double>(0, 0);
    S_c.at<double>(4) = v_c.at<double>(1, 0);
    S_c.at<double>(5) = v_c.at<double>(2, 0);
    qDebug().nospace() << "[Motion] Final Twist S_c: "
                       << "w[" << S_c.at<double>(0) << " " << S_c.at<double>(1) << " " << S_c.at<double>(2) << "] | "
                       << "v[" << S_c.at<double>(3) << " " << S_c.at<double>(4) << " " << S_c.at<double>(5) << "]";

    cv::Mat Ad_T_sc = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est)));
    cv::Mat V_end_effector = Ad_T_sc * S_c;

    cv::Mat T_increment = cv::Mat::eye(4, 4, CV_64F);
    T_increment = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(V_end_effector))));

    cv::Mat T_se_next = T_increment * T_se_curr;


    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_se_next));
}


void MainWindow::on_Age_ref_p_4_clicked()
{
    QWidget* targetWidget = ui->label_pic;

    cv::Mat T_cs_est = T_cs_est_;
    cv::Mat T_sc_est = T_cs_est.inv();

    static const QSize originalImageSize(2448, 2048);
    double scaleX = static_cast<double>(originalImageSize.width()) / targetWidget->width();
    double scaleY = static_cast<double>(originalImageSize.height()) / targetWidget->height();
    double scaleFactor = scaleFactor_;

    float centerX = Circle_[0];
    float centerY = Circle_[1];
    centerX = (centerX - originalImageSize.width() / 2) * scaleFactor;
    centerY = (centerY - originalImageSize.height() / 2) * scaleFactor;
    double centerZ = 0;
    this->maRobot_->motionActor_->getCommandedPosition(centerZ);
    centerZ = -centerZ * 1000;

    cv::Mat T_se_curr = this->maRobot_->robot_->getTaskMat_cv();
    cv::Mat T_es = T_se_curr.inv();

    cv::Mat p_camera_homo = (cv::Mat_<double>(4, 1) <<
                                 centerX,
                             centerY,
                             centerZ,
                             1.0);

    cv::Mat p_camera = (cv::Mat_<double>(3, 1) <<
                            centerX,
                        centerY,
                        centerZ);




    int isInv = 1 + ui->checkBox_inv->isChecked() * -2;
    cv::Mat omega_c;
    omega_c = (cv::Mat_<double>(3, 1) << 0, 0, angStep_ * isInv);
    cv::Mat v_c = p_camera.cross(omega_c);
    cv::Mat S_c = cv::Mat::zeros(6, 1, CV_64F);
    S_c.at<double>(0) = omega_c.at<double>(0, 0);
    S_c.at<double>(1) = omega_c.at<double>(1, 0);
    S_c.at<double>(2) = omega_c.at<double>(2, 0);
    S_c.at<double>(3) = v_c.at<double>(0, 0);
    S_c.at<double>(4) = v_c.at<double>(1, 0);
    S_c.at<double>(5) = v_c.at<double>(2, 0);
    qDebug().nospace() << "[Motion] Final Twist S_c: "
                       << "w[" << S_c.at<double>(0) << " " << S_c.at<double>(1) << " " << S_c.at<double>(2) << "] | "
                       << "v[" << S_c.at<double>(3) << " " << S_c.at<double>(4) << " " << S_c.at<double>(5) << "]";

    cv::Mat Ad_T_sc = cvMatFromEigen(Adjoint(eigenFromCvMat(T_sc_est)));
    cv::Mat V_end_effector = Ad_T_sc * S_c;

    cv::Mat T_increment = cv::Mat::eye(4, 4, CV_64F);
    T_increment = cvMatFromEigen(MatrixExp6(VecTose3(eigenFromCvMat(V_end_effector))));

    cv::Mat T_se_next = T_increment * T_se_curr;


    this->maRobot_->robot_->setRobotMode(ROBOT_POSITION);
    this->maRobot_->robot_->moveAbsolute(eigenFromCvMat(T_se_next));
}


void MainWindow::on_showCircle_2_clicked(bool checked)
{
    showTargetCircle_ = checked;
}


void MainWindow::on_showCircle_3_clicked(bool checked)
{
    showInitCircle_ = checked;
    if(checked == true)
        InitCircle_ = Circle_;
}


void MainWindow::on_FindReferance_clicked()
{
    // maRobot_->robot_->findReferance();
    piHexapodPrepared_ = false;
    // ui->pushButton_MHEC_Task->setEnabled(false);

    bool success = this->maRobot_->robot_->findReferance();
    if (success) {
        success = this->maRobot_->robot_->preparePivotForMotion(120000);
    }

    if (!success) {
        QMessageBox::warning(
            this,
            tr("并联台准备失败"),
            tr("参考状态确认、六轴回零或 Pivot 写入/回读失败，请检查 PI 调试输出。"));
        return;
    }

    piHexapodPrepared_ = true;
    ui->pushButton_MHEC_Task->setEnabled(true);

    QMessageBox::information(this,
                             tr("并联台准备完成"),
                             tr("并联台已回到六轴零位，并按代码开关完成 Pivot 写入和回读验证。"));
}


void MainWindow::on_Stop_2_clicked()
{
    this->maRobot_->robot_->setRobotMode(ROBOT_STOP);
    this->maRobot_->robot_->stop();
}

void MainWindow::on_RecordVideo_clicked(bool checked)
{
    if (checked) {
        if (!maRobot_->camera_->isAcquire()) {
            QMessageBox::warning(this, tr("录制失败"), tr("请先启动相机采集。"));
            ui->RecordVideo->setChecked(false);
            return;
        }

        const QString outputDir = QDir::currentPath() + "\\recordings";
        QDir().mkpath(outputDir);
        const QString defaultFileName = outputDir + "\\camera_" +
                                        QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") +
                                        ".avi";
        QFileDialog saveDialog(this, tr("保存录制视频"), defaultFileName, tr("AVI Video (*.avi)"));
        saveDialog.setAcceptMode(QFileDialog::AcceptSave);
        saveDialog.setFileMode(QFileDialog::AnyFile);
        saveDialog.setDefaultSuffix("avi");
        saveDialog.setOption(QFileDialog::DontUseNativeDialog, true);
        QString filePath;
        if (saveDialog.exec() == QDialog::Accepted && !saveDialog.selectedFiles().isEmpty()) {
            filePath = saveDialog.selectedFiles().first();
        }

        if (filePath.isEmpty()) {
            ui->RecordVideo->setChecked(false);
            return;
        }

        recordingDroppedFrames_ = 0;
        lastRecordErrorMessage_.clear();
        recordErrorTimer_.invalidate();
        isRecordingVideo_ = true;
        ui->RecordVideo->setText("StopRecording");
        ui->statusbar->showMessage(tr("视频录制中..."), 3000);
        emit sigStartVideoRecording(filePath, recordingFps_, "MJPG", true);
        return;
    }

    if (isRecordingVideo_) {
        isRecordingVideo_ = false;
        ui->RecordVideo->setText("StartRecording");
        ui->statusbar->showMessage(tr("正在停止录制并保存..."), 3000);
        emit sigStopVideoRecording();
    }
}

void MainWindow::on_RecordProcessedVideo_clicked(bool checked)
{
    if (checked) {
        if (!maRobot_->camera_->isAcquire()) {
            QMessageBox::warning(this, tr("录制失败"), tr("请先启动相机采集。"));
            ui->RecordProcessedVideo->setChecked(false);
            return;
        }

        const QString outputDir = QDir::currentPath() + "\\recordings";
        QDir().mkpath(outputDir);
        const QString defaultFileName = outputDir + "\\processed_camera_" +
                                        QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") +
                                        ".avi";
        QFileDialog saveDialog(this, tr("保存处理后录制视频"), defaultFileName, tr("AVI Video (*.avi)"));
        saveDialog.setAcceptMode(QFileDialog::AcceptSave);
        saveDialog.setFileMode(QFileDialog::AnyFile);
        saveDialog.setDefaultSuffix("avi");
        saveDialog.setOption(QFileDialog::DontUseNativeDialog, true);
        QString filePath;
        if (saveDialog.exec() == QDialog::Accepted && !saveDialog.selectedFiles().isEmpty()) {
            filePath = saveDialog.selectedFiles().first();
        }

        if (filePath.isEmpty()) {
            ui->RecordProcessedVideo->setChecked(false);
            return;
        }

        processedRecordingDroppedFrames_ = 0;
        lastProcessedRecordErrorMessage_.clear();
        processedRecordErrorTimer_.invalidate();
        isProcessedRecordingVideo_ = true;
        ui->RecordProcessedVideo->setText("StopProcessedRecording");
        ui->statusbar->showMessage(tr("处理后视频录制中..."), 3000);
        emit sigStartProcessedVideoRecording(filePath, recordingFps_, "MJPG", true);
        return;
    }

    if (isProcessedRecordingVideo_) {
        isProcessedRecordingVideo_ = false;
        ui->RecordProcessedVideo->setText("StartProcessedRecording");
        ui->statusbar->showMessage(tr("正在停止处理后录制并保存..."), 3000);
        emit sigStopProcessedVideoRecording();
    }
}

void MainWindow::onVideoRecordFinished(QString filePath, bool success)
{
    if (ui->RecordVideo->isChecked()) {
        ui->RecordVideo->setChecked(false);
    }
    ui->RecordVideo->setText("StartRecording");
    isRecordingVideo_ = false;

    if (success) {
        ui->statusbar->showMessage(
            tr("视频保存成功: %1（丢帧: %2）").arg(filePath).arg(recordingDroppedFrames_),
            8000);
    } else {
        ui->statusbar->showMessage(tr("视频保存失败: %1").arg(filePath), 8000);
    }
}

void MainWindow::onProcessedVideoRecordFinished(QString filePath, bool success)
{
    if (ui->RecordProcessedVideo->isChecked()) {
        ui->RecordProcessedVideo->setChecked(false);
    }
    ui->RecordProcessedVideo->setText("StartProcessedRecording");
    isProcessedRecordingVideo_ = false;

    if (success) {
        ui->statusbar->showMessage(
            tr("处理后视频保存成功: %1（丢帧: %2）").arg(filePath).arg(processedRecordingDroppedFrames_),
            8000);
    } else {
        ui->statusbar->showMessage(tr("处理后视频保存失败: %1").arg(filePath), 8000);
    }
}

void MainWindow::onVideoRecordError(QString message)
{
    if (!recordErrorTimer_.isValid()) {
        recordErrorTimer_.start();
    }

    const bool sameMessage = (message == lastRecordErrorMessage_);
    const bool tooFrequent = recordErrorTimer_.elapsed() < 2000;
    if (sameMessage && tooFrequent) {
        return;
    }

    lastRecordErrorMessage_ = message;
    recordErrorTimer_.restart();
    ui->statusbar->showMessage(message, 5000);
}

void MainWindow::onProcessedVideoRecordError(QString message)
{
    if (!processedRecordErrorTimer_.isValid()) {
        processedRecordErrorTimer_.start();
    }

    const bool sameMessage = (message == lastProcessedRecordErrorMessage_);
    const bool tooFrequent = processedRecordErrorTimer_.elapsed() < 2000;
    if (sameMessage && tooFrequent) {
        return;
    }

    lastProcessedRecordErrorMessage_ = message;
    processedRecordErrorTimer_.restart();
    ui->statusbar->showMessage(message, 5000);
}

void MainWindow::onRecordingFrameHandled()
{
}

void MainWindow::onProcessedRecordingFrameHandled()
{
}

void MainWindow::onRecordingStateChanged(bool isRecording, QString filePath)
{
    Q_UNUSED(filePath);
    if (!isRecording) {
        isRecordingVideo_ = false;
    }
    previewRefreshThrottle_.restart();
}

void MainWindow::onProcessedRecordingStateChanged(bool isRecording, QString filePath)
{
    Q_UNUSED(filePath);
    if (!isRecording) {
        isProcessedRecordingVideo_ = false;
    }
    previewRefreshThrottle_.restart();
}
