//没有获取并联台关节逻辑，原有相关逻辑改成了获取并联台末端位姿逻辑
//伺服模式——对应并联台开闭
//位姿矩阵逻辑：xyz顺序

#include "ParallelPlatform.h"
#include "modern_robotic_lib.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QStringList>
#include <QThread>
#include <QtGlobal>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace {

// PI Hexapod 在 GCS2 中一般使用 X/Y/Z/U/V/W 六个虚拟轴：
// X/Y/Z 单位为 mm，U/V/W 单位为 deg。
// 本项目上层沿用旧 Narpod 约定：X/Y/Z 为 nm，Rx/Ry/Rz 为 micro-degree。
constexpr const char* kPiAxes = "X Y Z U V W";
constexpr const char* kPiReferenceAxis = "X";
constexpr int kPiAxisCount = 6;
constexpr double kNmToMm = 1e-6;
constexpr double kMmToNm = 1e6;
constexpr double kUDegToDeg = 1e-6;
constexpr double kDegToUDeg = 1e6;
constexpr int kUsbEnumerateBufferSize = 4096;

QString envString(const char* name, const QString& fallback)
{
    const QByteArray value = qgetenv(name);
    return value.isEmpty() ? fallback : QString::fromLocal8Bit(value);
}

int envInt(const char* name, int fallback)
{
    bool ok = false;
    const int value = envString(name, QString()).toInt(&ok);
    return ok ? value : fallback;
}

double envDouble(const char* name, double fallback)
{
    bool ok = false;
    const double value = envString(name, QString()).toDouble(&ok);
    return ok ? value : fallback;
}

bool envEnabled(const char* name)
{
    const QString value = envString(name, QString()).trimmed().toLower();
    return value == "1" || value == "true" || value == "yes" || value == "on";
}

bool envDisabled(const char* name)
{
    const QString value = envString(name, QString()).trimmed().toLower();
    return value == "0" || value == "false" || value == "no" || value == "off";
}

QString builtInPiDllPath()
{
#ifdef PI_GCS2_LOCAL_DLL_PATH
    return QString::fromUtf8(PI_GCS2_LOCAL_DLL_PATH);
#else
    return QString();
#endif
}

std::array<double, 6> toPiPose(double x, double y, double z, double rx, double ry, double rz)
{
    return {
        x * kNmToMm,
        y * kNmToMm,
        z * kNmToMm,
        rx * kUDegToDeg,
        ry * kUDegToDeg,
        rz * kUDegToDeg
    };
}

Eigen::VectorXd fromPiPose(const std::array<double, 6>& piPose)
{
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);
    pose(0) = piPose[0] * kMmToNm;
    pose(1) = piPose[1] * kMmToNm;
    pose(2) = piPose[2] * kMmToNm;
    pose(3) = piPose[3] * kDegToUDeg;
    pose(4) = piPose[4] * kDegToUDeg;
    pose(5) = piPose[5] * kDegToUDeg;
    return pose;
}

QByteArray firstUsbDescription(const QByteArray& usbList)
{
    const int cr = usbList.indexOf('\r');
    const int lf = usbList.indexOf('\n');
    int end = usbList.size();
    if (cr >= 0) {
        end = std::min(end, cr);
    }
    if (lf >= 0) {
        end = std::min(end, lf);
    }
    return usbList.left(end).trimmed();
}

} // namespace

ParallelPlatform::ParallelPlatform(double hz)
{
    this->dt_ = 1 / hz;
    current_end_effector_pose_ = Eigen::VectorXd::Zero(6);
    this->b_T_e_ = Eigen::Matrix4d::Identity();

    this->m_preciseTimer_ = new QTimer();
    this->m_preciseTimer_->setInterval(this->dt_ * 1000);
    this->m_preciseTimer_->setTimerType(Qt::PreciseTimer);
    QObject::connect(this->m_preciseTimer_, &QTimer::timeout, this, &ParallelPlatform::controller);

    this->positionx_ = 0;
    this->positiony_ = 0;
    this->positionz_ = 0;
    this->rotationx_ = 0;
    this->rotationy_ = 0;
    this->rotationz_ = 0;
    this->currentPose_Vec_ = Eigen::VectorXd::Zero(6);
    this->currentPose_Mat_ = Eigen::Matrix4d::Identity();
    this->targetVelocity_ = Eigen::VectorXd::Zero(6);
    this->targetPose_ = Eigen::VectorXd::Zero(6);
    // H-811.S2IHP 数据手册给出的最小增量运动约为：
    // XYZ 20~40 nm，UVW 0.5~0.75 urad（约 29~43 udeg）。
    // 容差不能远小于硬件分辨能力，否则控制循环会在到位后重复发送同一 MOV。
    this->epsilon_v_ = envDouble("PI_HEXAPOD_LINEAR_TOLERANCE_NM", 50.0);
    this->epsilon_w_ = envDouble("PI_HEXAPOD_ANGULAR_TOLERANCE_UDEG", 50.0);
    this->controlMode_ = ROBOT_STOP;

    // 真机首次联调默认使用 PI 连接对话框，避免误连本机模拟器。
    // 明确知道控制器 IP 后，可用环境变量切换：
    // PI_HEXAPOD_CONNECTION=tcp|usb|dialog
    // PI_HEXAPOD_HOST / PI_HEXAPOD_PORT / PI_HEXAPOD_CONTROLLER
    piDllPath_ = envString("PI_GCS2_DLL", QString());
    piControllerType_ = envString("PI_HEXAPOD_CONTROLLER", "C-887");
    piHost_ = envString("PI_HEXAPOD_HOST", "127.0.0.1");
    piPort_ = envInt("PI_HEXAPOD_PORT", 50000);
    // Pivot 开关与坐标仅从 ParallelPlatform.h 读取，便于在 Qt Creator 中修改后重新编译。
    piAllowRetargetWhileMoving_ = envEnabled("PI_HEXAPOD_ALLOW_RETARGET_WHILE_MOVING");
    piDisableServoOnDisconnect_ = envEnabled("PI_HEXAPOD_DISABLE_SERVO_ON_DISCONNECT");
    // C-887.52 实机的 qIDN 仅返回 "C-887"，因此这里校验可由控制器查询到的系列名；
    // 平台则继续使用 CST? 精确校验为 H-811.S2IHP。
    piExpectedController_ = envString("PI_HEXAPOD_EXPECTED_CONTROLLER", "C-887");
    piExpectedStage_ = envString("PI_HEXAPOD_EXPECTED_STAGE", "H-811.S2IHP");

    const QString connectionMode = envString("PI_HEXAPOD_CONNECTION", "dialog").trimmed().toLower();
    piUseTcp_ = connectionMode != "usb" && connectionMode != "dialog";
    piUseUsb_ = connectionMode == "usb";
}

ParallelPlatform::~ParallelPlatform()
{
    stop();
    disconnect();
    delete m_preciseTimer_;
}

bool ParallelPlatform::loadPiDll()
{
    if (piDll_) {
        return true;
    }

    QStringList candidates;
    candidates << piDllPath_;
    candidates << QDir(QCoreApplication::applicationDirPath()).filePath("PI_GCS2_DLL_x64.dll");
    candidates << QDir(QCoreApplication::applicationDirPath()).filePath("dependence/PI/bin/x64/PI_GCS2_DLL_x64.dll");
    candidates << builtInPiDllPath();
    candidates << "C:/ProgramData/PI/GCSTranslator/PI_GCS2_DLL_x64.dll";
    candidates << "C:/Program Files (x86)/Physik Instrumente (PI)/Software Suite/Development/C++/API/PI_GCS2_DLL_x64.dll";

    for (const QString& candidate : candidates) {
        if (candidate.trimmed().isEmpty()) {
            continue;
        }

        piDll_ = LoadLibraryW(reinterpret_cast<LPCWSTR>(QDir::toNativeSeparators(candidate).utf16()));
        if (piDll_) {
            piDllPath_ = candidate;
            break;
        }
    }

    if (!piDll_) {
        result_ = false;
        qDebug() << "ParallelPlatform::loadPiDll failed: PI_GCS2_DLL_x64.dll was not found."
                 << "Checked application dir, dependence/PI/bin/x64, PI_GCS2_DLL, and PI install paths.";
        return false;
    }

    // Function signatures come from the vendored PI_GCS2_DLL_PF.h.
    bool ok = true;
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_InterfaceSetupDlg_), "PI_InterfaceSetupDlg");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_ConnectTCPIP_), "PI_ConnectTCPIP");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_EnumerateUSB_), "PI_EnumerateUSB");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_ConnectUSB_), "PI_ConnectUSB");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_CloseConnection_), "PI_CloseConnection");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qIDN_), "PI_qIDN");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qCST_), "PI_qCST");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_GetError_), "PI_GetError");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_TranslateError_), "PI_TranslateError");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_MOV_), "PI_MOV");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qPOS_), "PI_qPOS");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_IsMoving_), "PI_IsMoving");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_STP_), "PI_STP");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_StopAll_), "PI_StopAll");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_SVO_), "PI_SVO");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qSVO_), "PI_qSVO");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_FRF_), "PI_FRF");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qFRF_), "PI_qFRF");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_IsControllerReady_), "PI_IsControllerReady");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qTMN_), "PI_qTMN");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qTMX_), "PI_qTMX");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qTRA_), "PI_qTRA");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_SPI_), "PI_SPI");
    ok &= resolvePiFunction(reinterpret_cast<FARPROC&>(pi_qSPI_), "PI_qSPI");

    result_ = ok;
    if (!ok) {
        unloadPiDll();
    }

    return ok;
}

void ParallelPlatform::unloadPiDll()
{
    if (piDll_) {
        FreeLibrary(piDll_);
        piDll_ = nullptr;
    }

    pi_InterfaceSetupDlg_ = nullptr;
    pi_ConnectTCPIP_ = nullptr;
    pi_EnumerateUSB_ = nullptr;
    pi_ConnectUSB_ = nullptr;
    pi_CloseConnection_ = nullptr;
    pi_qIDN_ = nullptr;
    pi_qCST_ = nullptr;
    pi_GetError_ = nullptr;
    pi_TranslateError_ = nullptr;
    pi_MOV_ = nullptr;
    pi_qPOS_ = nullptr;
    pi_IsMoving_ = nullptr;
    pi_STP_ = nullptr;
    pi_StopAll_ = nullptr;
    pi_SVO_ = nullptr;
    pi_qSVO_ = nullptr;
    pi_FRF_ = nullptr;
    pi_qFRF_ = nullptr;
    pi_IsControllerReady_ = nullptr;
    pi_qTMN_ = nullptr;
    pi_qTMX_ = nullptr;
    pi_qTRA_ = nullptr;
    pi_SPI_ = nullptr;
    pi_qSPI_ = nullptr;
}

bool ParallelPlatform::resolvePiFunction(FARPROC& target, const char* name)
{
    target = GetProcAddress(piDll_, name);
    if (!target) {
        qDebug() << "ParallelPlatform::loadPiDll 缺少 PI GCS2 函数:" << name;
        return false;
    }
    return true;
}

bool ParallelPlatform::openPiConnection()
{
    if (piConnected_) {
        return true;
    }

    if (!loadPiDll()) {
        return false;
    }

    if (piUseTcp_) {
        const QByteArray host = piHost_.toLocal8Bit();
        piId_ = pi_ConnectTCPIP_(host.constData(), piPort_);
        if (piId_ < 0) {
            result_ = false;
            piLastError_ = pi_GetError_ ? pi_GetError_(piId_) : 0;
            qDebug() << "ParallelPlatform::connect TCP 连接失败:"
                     << piHost_ << piPort_ << describePiError();
            return false;
        }
    } else if (piUseUsb_) {
        std::array<char, kUsbEnumerateBufferSize> usbBuffer{};
        const QByteArray controller = piControllerType_.toLocal8Bit();
        if (!pi_EnumerateUSB_(usbBuffer.data(), static_cast<int>(usbBuffer.size()), controller.constData())) {
            updatePiError(false, "PI_EnumerateUSB");
            return false;
        }

        const QByteArray description = firstUsbDescription(QByteArray(usbBuffer.data()));
        if (description.isEmpty()) {
            result_ = false;
            qDebug() << "ParallelPlatform::connect USB 未找到控制器，类型过滤:" << piControllerType_;
            return false;
        }

        piId_ = pi_ConnectUSB_(description.constData());
        if (piId_ < 0) {
            result_ = false;
            piLastError_ = pi_GetError_ ? pi_GetError_(piId_) : 0;
            qDebug() << "ParallelPlatform::connect USB 连接失败:" << description << describePiError();
            return false;
        }
    } else {
        const QByteArray controller = piControllerType_.toLocal8Bit();
        piId_ = pi_InterfaceSetupDlg_(controller.constData());
        if (piId_ < 0) {
            result_ = false;
            piLastError_ = pi_GetError_ ? pi_GetError_(piId_) : 0;
            qDebug() << "ParallelPlatform::connect 连接对话框未建立连接:" << describePiError();
            return false;
        }
    }

    // 新连接必须重新读取型号、平台配置与参考状态，不能沿用上一次连接缓存。
    piControllerIdentification_.clear();
    piStageConfiguration_.clear();
    piConfigurationVerified_ = false;
    piMotionArmed_ = false;
    piConnected_ = true;
    result_ = true;
    return true;
}

bool ParallelPlatform::verifyPiConfiguration()
{
    if (!piConnected_) {
        result_ = false;
        return false;
    }

    // verifyPiConfiguration() 也可能由 findReferance() 直接触发，因此不能假设
    // connect() 已经提前读取过 qIDN。
    if (piControllerIdentification_.isEmpty() && !queryPiIdentification()) {
        return false;
    }

    std::array<char, 4096> stages{};
    if (!pi_qCST_(piId_, "", stages.data(), static_cast<int>(stages.size()))) {
        return updatePiError(false, "PI_qCST");
    }

    piStageConfiguration_ = QString::fromLocal8Bit(stages.data()).trimmed();
    const bool controllerMatches =
        piExpectedController_.trimmed().isEmpty() ||
        piControllerIdentification_.contains(piExpectedController_, Qt::CaseInsensitive);
    const bool stageMatches =
        piExpectedStage_.trimmed().isEmpty() ||
        piStageConfiguration_.contains(piExpectedStage_, Qt::CaseInsensitive);

    piConfigurationVerified_ = controllerMatches && stageMatches;
    if (!piConfigurationVerified_) {
        result_ = false;
        qDebug() << "ParallelPlatform::verifyPiConfiguration 配置不匹配，禁止运动。"
                 << "控制器:" << piControllerIdentification_
                 << "CST?:" << piStageConfiguration_
                 << "期望控制器:" << piExpectedController_
                 << "期望平台:" << piExpectedStage_;
        return false;
    }

    qDebug() << "ParallelPlatform::verifyPiConfiguration 配置检查通过。"
             << "控制器:" << piControllerIdentification_
             << "CST?:" << piStageConfiguration_;
    result_ = true;
    return true;
}

bool ParallelPlatform::queryPiIdentification()
{
    if (!piConnected_) {
        result_ = false;
        return false;
    }

    std::array<char, 512> idn{};
    if (!pi_qIDN_(piId_, idn.data(), static_cast<int>(idn.size()))) {
        return updatePiError(false, "PI_qIDN");
    }

    piControllerIdentification_ = QString::fromLocal8Bit(idn.data()).trimmed();
    if (piControllerIdentification_.isEmpty()) {
        result_ = false;
        qDebug() << "ParallelPlatform::queryPiIdentification 失败：PI_qIDN 调用成功但返回内容为空。";
        return false;
    }

    result_ = true;
    return true;
}

bool ParallelPlatform::queryAllAxesReferenced(bool& allReferenced)
{
    allReferenced = false;
    if (!piConnected_) {
        result_ = false;
        return false;
    }

    // PI 随附的 C-887 ReferenceSample 使用虚拟轴 X 表示整个 Hexapod 的参考状态。
    PI_BOOL referenced = 0;
    if (!pi_qFRF_(piId_, kPiReferenceAxis, &referenced)) {
        return updatePiError(false, "PI_qFRF");
    }

    allReferenced = referenced != 0;

    result_ = true;
    return true;
}

bool ParallelPlatform::queryPiMoving(bool& anyMoving)
{
    anyMoving = false;
    if (!piConnected_) {
        result_ = false;
        return false;
    }

    std::array<PI_BOOL, kPiAxisCount> moving{};
    if (!pi_IsMoving_(piId_, kPiAxes, moving.data())) {
        return updatePiError(false, "PI_IsMoving");
    }

    for (PI_BOOL value : moving) {
        anyMoving = anyMoving || value;
    }

    result_ = true;
    return true;
}

bool ParallelPlatform::connect()
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    if (!openPiConnection()) {
        return false;
    }

    if (!queryPiIdentification()) {
        return false;
    }
    qDebug() << "ParallelPlatform::connect 已连接 PI 控制器:" << piControllerIdentification_;

    // H-811 手册要求首次建立通信后查询 CST?，确认控制器加载了与实际平台匹配的配置。
    if (!verifyPiConfiguration()) {
        return false;
    }

    bool allReferenced = false;
    if (!queryAllAxesReferenced(allReferenced)) {
        return false;
    }
    piMotionArmed_ = allReferenced;
    piPivotPrepared_ = false;
    piPivotPreparationInProgress_ = false;

    if (!allReferenced) {
        qDebug() << "ParallelPlatform::connect 已完成只读连接，但尚未允许运动。"
                 << "确认平台固定、运输保护已拆除且工作空间无障碍后，显式调用 findReferance()。";
    } else {
        qDebug() << "ParallelPlatform::connect 六轴参考状态有效，运动命令已允许。";
    }

    refreshPiPose();
    qDebug() << "ParallelPlatform::connect 运行成功；连接阶段不设置 Pivot，"
                "确认参考状态后请调用 preparePivotForMotion()。DLL:" << piDllPath_;
    return true;
}
bool ParallelPlatform::disconnect()
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    stopupdate();

    if (piConnected_ && pi_CloseConnection_) {
        // H-811 自锁性很低，带载或非水平安装时关闭伺服可能导致平台意外改变位置。
        // 因此默认只关闭通信；确需关闭伺服时显式设置对应环境变量。
        if (piDisableServoOnDisconnect_) {
            setPiServo(false);
        }
        pi_CloseConnection_(piId_);
    }

    piId_ = -1;
    piConnected_ = false;
    piConfigurationVerified_ = false;
    piMotionArmed_ = false;
    piPivotPrepared_ = false;
    piPivotPreparationInProgress_ = false;
    result_ = true;
    unloadPiDll();
    qDebug() << "ParallelPlatform::disconnect 运行成功";
    return true;
}
bool ParallelPlatform::findReferance()
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    if (!piConnected_ && !openPiConnection()) {
        return false;
    }
    if (!piConfigurationVerified_ && !verifyPiConfiguration()) {
        return false;
    }

    bool allReferenced = false;
    if (!queryAllAxesReferenced(allReferenced)) {
        return false;
    }

    if (allReferenced) {
        // H-811 绝对测量传感器机型无需重复参考运动。
        piMotionArmed_ = true;
        piPivotPrepared_ = false;
        refreshPiPose();
        qDebug() << "ParallelPlatform::findReferance qFRF 已为 1，平台参考状态已经有效；"
                    "未发送 FRF，不会发生参考运动。";
        return true;
    }

    qDebug() << "ParallelPlatform::findReferance qFRF 为 0，即将执行 FRF，平台会发生运动。";
    if (!setPiServo(true)) {
        return false;
    }
    if (!pi_FRF_(piId_, kPiReferenceAxis)) {
        return updatePiError(false, "PI_FRF");
    }
    if (!waitForPiMotionDone(120000)) {
        return false;
    }
    if (!queryAllAxesReferenced(allReferenced) || !allReferenced) {
        result_ = false;
        qDebug() << "ParallelPlatform::findReferance 失败：平台参考状态仍无效。";
        return false;
    }

    piMotionArmed_ = true;
    piPivotPrepared_ = false;
    refreshPiPose();
    qDebug() << "ParallelPlatform::findReferance 六轴参考状态有效，已允许运动；"
                "本函数不设置 Pivot。";
    return true;
}

bool ParallelPlatform::preparePivotForMotion(int timeoutMs)
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    if (!piConnected_ || !piConfigurationVerified_ || !piMotionArmed_) {
        qDebug() << "ParallelPlatform::preparePivotForMotion 已拒绝："
                    "必须先完成连接、配置检查和参考状态确认。";
        result_ = false;
        return false;
    }

    // 防止状态定时器在准备过程中生成其他运动目标。
    controlMode_ = ROBOT_STOP;
    targetVelocity_ = Eigen::VectorXd::Zero(6);
    piPivotPrepared_ = false;
    piPivotPreparationInProgress_ = true;

    qDebug() << "ParallelPlatform::preparePivotForMotion 开始标准准备流程："
                "等待停止，然后移动到六轴零位。";
    if (!waitForPiMotionDone(timeoutMs) ||
        !moveAbsolute(0, 0, 0, 0, 0, 0) ||
        !waitForPiMotionDone(timeoutMs)) {
        piPivotPreparationInProgress_ = false;
        qDebug() << "ParallelPlatform::preparePivotForMotion 回零失败，未写入 Pivot。";
        result_ = false;
        return false;
    }

    const signed int targetXNm = piApplyPivotOnConnect_
        ? static_cast<signed int>(std::lround(piDefaultPivotXNm_)) : 0;
    const signed int targetYNm = piApplyPivotOnConnect_
        ? static_cast<signed int>(std::lround(piDefaultPivotYNm_)) : 0;
    const signed int targetZNm = piApplyPivotOnConnect_
        ? static_cast<signed int>(std::lround(piDefaultPivotZNm_)) : 0;
    if (!setPivot(targetXNm, targetYNm, targetZNm)) {
        piPivotPreparationInProgress_ = false;
        qDebug() << "ParallelPlatform::preparePivotForMotion 写入或回读 Pivot 失败。";
        return false;
    }

    piPivotPreparationInProgress_ = false;
    piPivotPrepared_ = true;
    qDebug() << "ParallelPlatform::preparePivotForMotion 完成。Pivot"
             << (piApplyPivotOnConnect_ ? "已开启，坐标(nm):" : "已关闭并清零，坐标(nm):")
             << targetXNm << targetYNm << targetZNm;
    result_ = true;
    return true;
}
bool ParallelPlatform::startupdate()
{
    // 状态读取前先建立只读连接并核对 qIDN/CST?；此过程不会开启伺服或触发运动。
    if (!piConnected_ && !connect()) {
        return false;
    }
    this->m_preciseTimer_->start();
    return true;
}

bool ParallelPlatform::stopupdate()
{
    this->m_preciseTimer_->stop();
    return true;
}

bool ParallelPlatform::stop()
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    this->targetVelocity_ = Eigen::VectorXd::Zero(6);
    if (!piConnected_) {
        result_ = true;
        return true;
    }

    // STP 是 GCS2 停止命令；StopAll 作为兼容兜底。
    if (pi_STP_ && pi_STP_(piId_)) {
        result_ = true;
        return true;
    }

    if (pi_StopAll_ && pi_StopAll_(piId_)) {
        result_ = true;
        return true;
    }

    return updatePiError(false, "PI_STP/PI_StopAll");
}

bool ParallelPlatform::moveAbsolute(signed int x, signed int y, signed int z,
                                    signed int rx, signed int ry, signed int rz)
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    if (!piConnected_ && !openPiConnection()) {
        return false;
    }
    if (!piConfigurationVerified_) {
        qDebug() << "ParallelPlatform::moveAbsolute 已拒绝：尚未通过控制器/平台配置检查。";
        result_ = false;
        return false;
    }
    if (!piMotionArmed_) {
        qDebug() << "ParallelPlatform::moveAbsolute 已拒绝：参考状态无效，尚未允许运动。";
        result_ = false;
        return false;
    }

    if (!piPivotPrepared_ && !piPivotPreparationInProgress_) {
        qDebug() << "ParallelPlatform::moveAbsolute 已拒绝：尚未完成回零与 Pivot 准备，"
                    "请先调用 preparePivotForMotion()。";
        result_ = false;
        return false;
    }
    bool moving = false;
    if (!queryPiMoving(moving)) {
        return false;
    }
    if (moving && !piAllowRetargetWhileMoving_) {
        // H-811 手册警告：运动中发送新目标会产生未定义路径。
        // 控制循环会在平台停止后再次尝试，因此这里不将其作为硬错误。
        return true;
    }

    if (!checkPoseReachable(x, y, z, rx, ry, rz)) {
        result_ = false;
        qDebug() << "ParallelPlatform::moveAbsolute 输入位姿超出 PI 轴限位。";
        return false;
    }

    if (!setPiServo(true)) {
        return false;
    }

    const std::array<double, 6> target = toPiPose(x, y, z, rx, ry, rz);
    if (!pi_MOV_(piId_, kPiAxes, target.data())) {
        return updatePiError(false, "PI_MOV");
    }
    // qDebug() << "ParallelPlatform::moveAbsolute 已发送 PI_MOV。"
    //          << "PI 单位目标 [mm, mm, mm, deg, deg, deg]:"
    //          << target[0] << target[1] << target[2]
    //          << target[3] << target[4] << target[5];

    currentPose_Vec_ = Eigen::VectorXd::Zero(6);
    currentPose_Vec_(0) = x;
    currentPose_Vec_(1) = y;
    currentPose_Vec_(2) = z;
    currentPose_Vec_(3) = rx;
    currentPose_Vec_(4) = ry;
    currentPose_Vec_(5) = rz;
    syncCachedPoseFromCurrentVector();

    // PI_MOV 是控制器锁存的绝对位置命令，只需真正发送一次。
    // 位置模式仅用于等待合适时机下发目标；发送成功后切回 STOP，定时器继续刷新 qPOS，
    // 但不会因为实际到位误差略高于软件阈值而反复发送同一个 PI_MOV。
    if (controlMode_ == ROBOT_POSITION) {
        controlMode_ = ROBOT_STOP;
    }

    // qPOS 是实际读回值。运动过程中读数可能尚未到达目标，因此这里只尽量刷新，不阻塞控制循环。
    refreshPiPose();
    result_ = true;
    return true;
}

bool ParallelPlatform::moveAbsolute(Eigen::Matrix4d T)
{
    const Eigen::VectorXd targetList = getList(T);
    this->targetPose_ = targetList;
    return this->moveAbsolute(static_cast<signed int>(std::lround(targetList(0))),
                              static_cast<signed int>(std::lround(targetList(1))),
                              static_cast<signed int>(std::lround(targetList(2))),
                              static_cast<signed int>(std::lround(targetList(3))),
                              static_cast<signed int>(std::lround(targetList(4))),
                              static_cast<signed int>(std::lround(targetList(5))));
}

bool ParallelPlatform::checkresult()
{
    if (result_) {
        return true;
    }

    qDebug() << "ParallelPlatform 函数运行错误:" << describePiError();
    return false;
}

bool ParallelPlatform::checkPoseReachable(int x, int y, int z, int rx, int ry, int rz)
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    if (!piConnected_) {
        // 未连接硬件时无法查询 PI 控制器限位，保守返回 false。
        qDebug() << "ParallelPlatform::checkPoseReachable 未连接 PI 控制器。";
        return false;
    }

    std::array<double, 6> minLimits{};
    std::array<double, 6> maxLimits{};
    if (!pi_qTMN_(piId_, kPiAxes, minLimits.data())) {
        return updatePiError(false, "PI_qTMN");
    }
    if (!pi_qTMX_(piId_, kPiAxes, maxLimits.data())) {
        return updatePiError(false, "PI_qTMX");
    }

    const std::array<double, 6> target = toPiPose(x, y, z, rx, ry, rz);
    constexpr double epsilon = 1e-9;
    for (int i = 0; i < 6; ++i) {
        if (target[i] < minLimits[i] - epsilon || target[i] > maxLimits[i] + epsilon) {
            qDebug() << "ParallelPlatform::checkPoseReachable 轴" << i
                     << "目标" << target[i] << "超出范围"
                     << minLimits[i] << maxLimits[i]
                     << "。注意 PI 单位：XYZ=mm, UVW=deg。";
            result_ = false;
            return false;
        }
    }

    // Hexapod 六个自由度的有效范围互相耦合，仅检查每轴 TMN/TMX 不足。
    // qTRA 返回从当前位置沿给定方向能够到达的最远绝对位置。
    std::array<double, 6> current{};
    if (!pi_qPOS_(piId_, kPiAxes, current.data())) {
        return updatePiError(false, "PI_qPOS");
    }

    std::array<double, 6> direction{};
    double maxDirectionComponent = 0.0;
    bool hasDirection = false;
    for (int i = 0; i < 6; ++i) {
        direction[i] = target[i] - current[i];
        maxDirectionComponent = std::max(maxDirectionComponent, std::abs(direction[i]));
        hasDirection = hasDirection || std::abs(direction[i]) > epsilon;
    }

    if (hasDirection) {
        // qTRA 需要的是六维运动方向。直接传入很小的目标差值时，C-887 可能把方向
        // 量化为接近零，返回几乎等于当前位置的“最大可达位置”，进而造成误判。
        // 用统一比例归一化不会改变该方向对应的射线。
        std::array<double, 6> normalizedDirection{};
        for (int i = 0; i < 6; ++i) {
            normalizedDirection[i] = direction[i] / maxDirectionComponent;
        }

        std::array<double, 6> reachableLimit{};
        if (!pi_qTRA_(piId_, kPiAxes, normalizedDirection.data(), reachableLimit.data())) {
            return updatePiError(false, "PI_qTRA");
        }

        const std::array<double, 6> reachableTolerance = {
            epsilon_v_ * kNmToMm,
            epsilon_v_ * kNmToMm,
            epsilon_v_ * kNmToMm,
            epsilon_w_ * kUDegToDeg,
            epsilon_w_ * kUDegToDeg,
            epsilon_w_ * kUDegToDeg
        };
        for (int i = 0; i < 6; ++i) {
            if ((direction[i] > epsilon &&
                 target[i] > reachableLimit[i] + reachableTolerance[i]) ||
                (direction[i] < -epsilon &&
                 target[i] < reachableLimit[i] - reachableTolerance[i])) {
                qDebug() << "ParallelPlatform::checkPoseReachable qTRA 判定目标位姿不可达。"
                         << "轴索引:" << i << "目标:" << target[i]
                         << "沿当前方向最大可达位置:" << reachableLimit[i]
                         << "判定容差:" << reachableTolerance[i];
                result_ = false;
                return false;
            }
        }
    }

    result_ = true;
    return true;
}

bool ParallelPlatform::getJointPositions(Eigen::VectorXd& positions)
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    // Hexapod 没有对外暴露单独“关节坐标”，这里保持旧接口，返回六个任务空间虚拟轴。
    refreshPiPose();
    positions = currentPose_Vec_;
    return result_;
}

bool ParallelPlatform::getTaskPositions(Eigen::VectorXd& positions)
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    refreshPiPose();
    positions = currentPose_Vec_;
    return result_;
}

bool ParallelPlatform::getTargetVelocity(Eigen::VectorXd& targetVelocity)
{
    targetVelocity = targetVelocity_;
    return true;
}

bool ParallelPlatform::getTaskMat(Eigen::Matrix4d& T)
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    refreshPiPose();
    T = currentPose_Mat_;
    return result_;
}

cv::Mat ParallelPlatform::getTaskMat_cv()
{
    cv::Mat T(4, 4, CV_64F);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T.at<double>(i, j) = currentPose_Mat_(i, j);
        }
    }
    return T.clone();
}

bool ParallelPlatform::setPivot(signed int x, signed int y, signed int z)
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    if (!piConnected_ && !openPiConnection()) {
        return false;
    }
    if (!piConfigurationVerified_ || !piMotionArmed_) {
        qDebug() << "ParallelPlatform::setPivot 已拒绝：必须先完成配置检查和参考状态确认。";
        result_ = false;
        return false;
    }

    bool moving = false;
    if (!queryPiMoving(moving)) {
        return false;
    }
    if (moving) {
        qDebug() << "ParallelPlatform::setPivot 已拒绝：平台仍在运动。";
        result_ = false;
        return false;
    }

    std::array<double, 6> current{};
    if (!pi_qPOS_(piId_, kPiAxes, current.data())) {
        return updatePiError(false, "PI_qPOS");
    }
    // H-811 的最小角运动增量约为几十 micro-degree；1 micro-degree 的旧阈值过严。
    constexpr double rotationZeroToleranceDeg = 1e-4;
    if (std::abs(current[3]) > rotationZeroToleranceDeg ||
        std::abs(current[4]) > rotationZeroToleranceDeg ||
        std::abs(current[5]) > rotationZeroToleranceDeg) {
        qDebug() << "ParallelPlatform::setPivot 已拒绝：设置 Pivot 前应先回到 U=V=W=0。"
                 << "当前 U/V/W(deg):" << current[3] << current[4] << current[5];
        result_ = false;
        return false;
    }

    const std::array<double, 3> pivot = {
        x * kNmToMm,
        y * kNmToMm,
        z * kNmToMm
    };
    if (!pi_SPI_(piId_, "X Y Z", pivot.data())) {
        return updatePiError(false, "PI_SPI");
    }

    std::array<double, 3> readback{};
    if (!pi_qSPI_(piId_, "X Y Z", readback.data())) {
        return updatePiError(false, "PI_qSPI");
    }
    constexpr double pivotReadbackToleranceMm = 1e-6;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(readback[i] - pivot[i]) > pivotReadbackToleranceMm) {
            qDebug() << "ParallelPlatform::setPivot 回读不一致。目标(mm):"
                     << pivot[0] << pivot[1] << pivot[2]
                     << "实际(mm):" << readback[0] << readback[1] << readback[2];
            result_ = false;
            return false;
        }
    }

    piPivotPrepared_ = true;
    qDebug() << "ParallelPlatform::setPivot 写入并回读成功。Pivot(mm):"
             << readback[0] << readback[1] << readback[2];
    result_ = true;
    return true;
}
void ParallelPlatform::update()
{
    refreshPiPose();

    Eigen::VectorXd tList(6);
    tList(0) = positionx_;
    tList(1) = positiony_;
    tList(2) = positionz_;
    tList(3) = rotationx_;
    tList(4) = rotationy_;
    tList(5) = rotationz_;
    b_T_e_ = currentPose_Mat_;

    emit sig_updateRobotData(tList, targetVelocity_, currentPose_Vec_);
}

void ParallelPlatform::controller()
{
    update();

    if (this->controlMode_ == ROBOT_VELOCITY) {
        targetPose_ = currentPose_Vec_;
        if (targetVelocity_.head(3).norm() < epsilon_v_ &&
            targetVelocity_.tail(3).norm() < epsilon_w_) {
            targetVelocity_ = Eigen::VectorXd::Zero(6);
            return;
        }
        this->moveVelocity(this->targetVelocity_, this->dt_);
    } else if (this->controlMode_ == ROBOT_POSITION) {
        this->targetVelocity_ = Eigen::VectorXd::Zero(6);
        const Eigen::VectorXd errorPose = currentPose_Vec_ - targetPose_;
        if (errorPose.segment(0, 3).norm() >= epsilon_v_ ||
            errorPose.segment(3, 3).norm() >= epsilon_w_) {
            this->moveAbsolute(static_cast<signed int>(std::lround(targetPose_(0))),
                               static_cast<signed int>(std::lround(targetPose_(1))),
                               static_cast<signed int>(std::lround(targetPose_(2))),
                               static_cast<signed int>(std::lround(targetPose_(3))),
                               static_cast<signed int>(std::lround(targetPose_(4))),
                               static_cast<signed int>(std::lround(targetPose_(5))));
        }
    } else if (this->controlMode_ == ROBOT_STOP) {
        // ROBOT_STOP 表示不再生成运动命令。真正的 STP 只应由 stop() 显式发送一次；
        // 状态定时器中反复发送 STP 会让 C-887 持续报告停止事件并蜂鸣。
        this->targetVelocity_ = Eigen::VectorXd::Zero(6);
    } else if (this->controlMode_ == ROBOT_MOVE_TO_ZERO) {
        this->targetVelocity_ = Eigen::VectorXd::Zero(6);
        // 只生成一次零位目标，后续由 ROBOT_POSITION 监视到位状态。
        targetPose_ = Eigen::VectorXd::Zero(6);
        controlMode_ = ROBOT_POSITION;
        moveAbsolute(0, 0, 0, 0, 0, 0);
    }
}

Eigen::Matrix4d ParallelPlatform::getT(Eigen::VectorXd Tlist)
{
    // 输入单位沿用本项目：xyz 为 nm，rxyz 为 micro-degree。
    const double x = Tlist(0);
    const double y = Tlist(1);
    const double z = Tlist(2);
    double rx = Tlist(3) / 1000000.0;
    double ry = Tlist(4) / 1000000.0;
    double rz = Tlist(5) / 1000000.0;
    const double h = 0.0;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.col(3).head(3) = Eigen::Vector3d(x, y, z + h);

    Eigen::Matrix4d Rx = Eigen::Matrix4d::Identity();
    rx = rx * M_PI / 180.0;
    Rx(1, 1) = std::cos(rx);
    Rx(1, 2) = -std::sin(rx);
    Rx(2, 1) = std::sin(rx);
    Rx(2, 2) = std::cos(rx);

    Eigen::Matrix4d Ry = Eigen::Matrix4d::Identity();
    ry = ry * M_PI / 180.0;
    Ry(0, 0) = std::cos(ry);
    Ry(0, 2) = std::sin(ry);
    Ry(2, 0) = -std::sin(ry);
    Ry(2, 2) = std::cos(ry);

    Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
    rz = rz * M_PI / 180.0;
    Rz(0, 0) = std::cos(rz);
    Rz(0, 1) = -std::sin(rz);
    Rz(1, 0) = std::sin(rz);
    Rz(1, 1) = std::cos(rz);

    const Eigen::Matrix4d e0RotE1 = Rx * Ry * Rz;
    return T * e0RotE1;
}

Eigen::VectorXd ParallelPlatform::getList(Eigen::Matrix4d T)
{
    const double h = 0.0;
    const double x = T(0, 3);
    const double y = T(1, 3);
    const double z = T(2, 3) - h;

    const double cosYAbs = std::sqrt(std::max(0.0, 1.0 - T(0, 2) * T(0, 2)));
    const double thetaY = std::atan2(T(0, 2), cosYAbs);
    const double thetaX = std::atan2(-T(1, 2), T(2, 2));
    const double thetaZ = std::atan2(-T(0, 1), T(0, 0));

    Eigen::VectorXd taskPosition = Eigen::VectorXd::Zero(6);
    taskPosition(0) = std::lround(x);
    taskPosition(1) = std::lround(y);
    taskPosition(2) = std::lround(z);
    taskPosition(3) = std::lround(thetaX * 180.0 * 1000000.0 / M_PI);
    taskPosition(4) = std::lround(thetaY * 180.0 * 1000000.0 / M_PI);
    taskPosition(5) = std::lround(thetaZ * 180.0 * 1000000.0 / M_PI);
    return taskPosition;
}

bool ParallelPlatform::setTargetVelocity(Eigen::VectorXd V)
{
    setRobotMode(ROBOT_VELOCITY);
    this->targetVelocity_ = V;
    return true;
}

bool ParallelPlatform::setTargetPose(Eigen::VectorXd P)
{
    if (P.size() < 6) {
        qDebug() << "ParallelPlatform::setTargetPose 输入维度不足，需要 [x y z rx ry rz]。";
        result_ = false;
        return false;
    }

    setRobotMode(ROBOT_POSITION);
    this->targetPose_ = P.head(6);
    result_ = true;
    return true;
}

void ParallelPlatform::setRobotMode(int mode)
{
    controlMode_ = mode;
}

bool ParallelPlatform::moveVelocity(Eigen::VectorXd velocityListB, double dt)
{
    if (velocityListB.size() < 6) {
        qDebug() << "ParallelPlatform::moveVelocity 输入维度不足，需要 [vx vy vz wx wy wz]。";
        result_ = false;
        return false;
    }

    // 输入速度单位沿用旧程序：平移为 nm/s，角速度为 micro-degree/s。
    // modern_robotics 的 twist 顺序为 [wx wy wz vx vy vz]，角速度内部转为 rad/s。
    const double angleScale = M_PI / (180.0 * 1000000.0);
    Eigen::VectorXd velocityTemp(6);
    velocityTemp(0) = velocityListB(3) * angleScale;
    velocityTemp(1) = velocityListB(4) * angleScale;
    velocityTemp(2) = velocityListB(5) * angleScale;
    velocityTemp(3) = velocityListB(0);
    velocityTemp(4) = velocityListB(1);
    velocityTemp(5) = velocityListB(2);

    this->b_T_e_ = currentPose_Mat_;
    const Eigen::VectorXd velocityListE = Adjoint(this->b_T_e_.inverse()) * velocityTemp;
    const Eigen::Matrix4d e1TE2 = MatrixExp6(VecTose3(velocityListE) * dt);
    const Eigen::Matrix4d bTE2 = this->b_T_e_ * e1TE2;
    return this->moveAbsolute(bTE2);
}

bool ParallelPlatform::refreshPiPose()
{
    QMutexLocker<QRecursiveMutex> locker(&piMutex_);
    if (!piConnected_) {
        result_ = false;
        return false;
    }

    std::array<double, 6> piPose{};
    if (!pi_qPOS_(piId_, kPiAxes, piPose.data())) {
        return updatePiError(false, "PI_qPOS");
    }

    currentPose_Vec_ = fromPiPose(piPose);
    syncCachedPoseFromCurrentVector();
    result_ = true;
    return true;
}

bool ParallelPlatform::waitForPiMotionDone(int timeoutMs)
{
    QElapsedTimer timer;
    timer.start();

    while (timer.elapsed() < timeoutMs) {
        int ready = 0;
        if (pi_IsControllerReady_ && pi_IsControllerReady_(piId_, &ready) && ready) {
            result_ = true;
            return true;
        }

        PI_BOOL moving[6] = {};
        if (pi_IsMoving_ && pi_IsMoving_(piId_, kPiAxes, moving)) {
            bool anyMoving = false;
            for (PI_BOOL value : moving) {
                anyMoving = anyMoving || value;
            }
            if (!anyMoving) {
                result_ = true;
                return true;
            }
        }

        QThread::msleep(50);
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }

    result_ = false;
    qDebug() << "ParallelPlatform::waitForPiMotionDone 等待 PI 运动完成超时。";
    return false;
}

bool ParallelPlatform::setPiServo(bool enabled)
{
    if (!piConnected_) {
        result_ = false;
        return false;
    }

    // PI 随附的 C-887 ReferenceSample/MoveSample 使用虚拟轴 X 控制整个 Hexapod 的伺服。
    // 不先尝试对 X/Y/Z/U/V/W 分别发送 SVO，避免控制器因不支持的轴列表产生错误或蜂鸣。
    const PI_BOOL servoValue = enabled ? 1 : 0;
    PI_BOOL currentServoValue = 0;
    if (pi_qSVO_(piId_, kPiReferenceAxis, &currentServoValue) &&
        currentServoValue == servoValue) {
        result_ = true;
        piLastError_ = 0;
        return true;
    }

    PI_BOOL singleServoValue = servoValue;
    if (pi_SVO_(piId_, kPiReferenceAxis, &singleServoValue)) {
        result_ = true;
        piLastError_ = 0;
        return true;
    }

    return updatePiError(false, "PI_SVO");
}

bool ParallelPlatform::updatePiError(bool ok, const char* commandName)
{
    result_ = ok;
    if (ok) {
        piLastError_ = 0;
        return true;
    }

    piLastError_ = 0;
    if (piConnected_ && pi_GetError_) {
        piLastError_ = pi_GetError_(piId_);
    }

    qDebug() << "ParallelPlatform" << commandName << "失败:" << describePiError();
    return false;
}

QString ParallelPlatform::describePiError() const
{
    if (!pi_TranslateError_) {
        return QString("PI error %1").arg(piLastError_);
    }

    std::array<char, 512> message{};
    if (pi_TranslateError_(piLastError_, message.data(), static_cast<int>(message.size()))) {
        return QString("PI error %1: %2").arg(piLastError_).arg(QString::fromLocal8Bit(message.data()));
    }

    return QString("PI error %1").arg(piLastError_);
}

void ParallelPlatform::syncCachedPoseFromCurrentVector()
{
    positionx_ = static_cast<int>(std::lround(currentPose_Vec_(0)));
    positiony_ = static_cast<int>(std::lround(currentPose_Vec_(1)));
    positionz_ = static_cast<int>(std::lround(currentPose_Vec_(2)));
    rotationx_ = static_cast<int>(std::lround(currentPose_Vec_(3)));
    rotationy_ = static_cast<int>(std::lround(currentPose_Vec_(4)));
    rotationz_ = static_cast<int>(std::lround(currentPose_Vec_(5)));
    currentPose_Mat_ = getT(currentPose_Vec_);
    current_end_effector_pose_ = currentPose_Vec_;
    b_T_e_ = currentPose_Mat_;
}
