#include "AgeMotionDriver.h"
#include <QCoreApplication>
#include <QDir>
#include <algorithm>
#include <cmath>
#include <limits>
#include <QtGlobal> // 包含 qAbs 函数

// ==========================================
//          构造与基础状态管理
// ==========================================

AgeMotionDriver::AgeMotionDriver(bool isInverted)
    : m_isConnected(false), m_isInverted(isInverted)
{
}

AgeMotionDriver::~AgeMotionDriver()
{
    if (m_lib.isLoaded()) {
        m_lib.unload();
    }
}

// bool AgeMotionDriver::isInverted() const
// {
//     return m_isInverted;
// }

// void AgeMotionDriver::setInverted(bool inverted)
// {
//     m_isInverted = inverted;
// }

QString AgeMotionDriver::getLastError() const
{
    return m_lastError;
}

// ==========================================
//          连接与初始化
// ==========================================

bool AgeMotionDriver::connectDevice()
{
    // 1. 加载 DLL
    if (!loadLibrary()) {
        return false;
    }

    // 2. 执行授权
    if (!authorize()) {
        m_lastError = "License authorization failed.";
        return false;
    }

    // 3. 建立连接 (AutoConnect = 1)
    if (m_api_isValid && !m_api_isValid(1)) {
        m_lastError = "Failed to connect to USB Device (AgeCOMIsValid returned FALSE).";
        return false;
    }

    m_isConnected = true;

    // 0.01 um * 32000 MMS/um = 320 MMS/pulse = 100000 ppr.
    if (!setMinStepUm(AUTOFOCUS_POSITION_RESOLUTION_UM)) {
        m_lastError = "Failed to configure the 0.01 um autofocus resolution.";
        m_isConnected = false;
        return false;
    }
    double configuredStepUm = 0.0;
    if (!getMinStepUm(configuredStepUm) ||
        std::abs(configuredStepUm - AUTOFOCUS_POSITION_RESOLUTION_UM) >
            0.5 / MMS_PER_UM) {
        m_lastError = "Autofocus resolution readback does not match 0.01 um.";
        m_isConnected = false;
        return false;
    }
    qInfo() << "AgeMotion autofocus resolution (um):" << configuredStepUm
            << "pulse length (MMS):" << m_pulseStepLengthMms
            << "PPR:" << MMS_PER_R / m_pulseStepLengthMms;
    qDebug() << "✅ AgeMotionDriver: Device connected successfully.";

    // 读取并保存默认目标速度
    double vel = 0.0;
    if (getTargetVelocity(vel)) {
        m_defaultTargetVelocity = vel;
        qDebug() << "Default Target Velocity saved:" << m_defaultTargetVelocity << "um/s";
    }

    if (getVelZero(vel)) {
        qDebug() << "Default Velocity Zero saved:" << vel << "um/s";
        setVelZero(3000); // 按原有逻辑设定回零速度
    }

    return true;
}

bool AgeMotionDriver::loadLibrary()
{
    const QString dllPath = QDir(QCoreApplication::applicationDirPath())
                                .filePath(QString::fromLatin1(DLL_RELATIVE_PATH));
    m_lib.setFileName(dllPath);

    if (!m_lib.load()) {
        m_lastError = "Failed to load DLL at: " + dllPath + "\nError: " + m_lib.errorString();
        qCritical() << m_lastError;
        return false;
    }

    // 解析函数指针
    m_api_isValid   = (AgeCOMIsValidFunc)m_lib.resolve("AgeCOMIsValid");
    m_api_getUSBID  = (AgeCOMGetUSBIDFunc)m_lib.resolve("AgeCOMGetUSBID");
    m_api_readQWORD = (AgeCOMReadQWORDFunc)m_lib.resolve("AgeCOMReadQWORD");
    m_api_getCOMID  = (AgeCOMGetCOMIDFunc)m_lib.resolve("AgeCOMGetCOMID");
    m_api_setSerial = (AgeCOMSerialFunc)m_lib.resolve("AgeCOMSerial");

    m_api_readWORD   = (AgeCOMReadWORDFunc)m_lib.resolve("AgeCOMReadWORD");
    m_api_writeWORD  = (AgeCOMWriteWORDFunc)m_lib.resolve("AgeCOMWriteWORD");
    m_api_writeQWORD = (AgeCOMWriteQWORDFunc)m_lib.resolve("AgeCOMWriteQWORD");

    m_api_readDWORD  = (AgeCOMReadDWORDFunc)m_lib.resolve("AgeCOMReadDWORD");
    m_api_writeDWORD = (AgeCOMWriteDWORDFunc)m_lib.resolve("AgeCOMWriteDWORD");

    if (!m_api_isValid || !m_api_readQWORD || !m_api_setSerial ||
        !m_api_readWORD || !m_api_writeWORD || !m_api_writeQWORD ||
        !m_api_readDWORD || !m_api_writeDWORD) {
        m_lastError = "Failed to resolve one or more functions from DLL.";
        qCritical() << m_lastError;
        return false;
    }

    return true;
}

bool AgeMotionDriver::authorize()
{
    if (!m_api_setSerial) return false;
    DWORD keyLength = (DWORD)strlen(LICENSE_KEY);
    return m_api_setSerial((BYTE*)LICENSE_KEY, keyLength);
}

// ==========================================
//          位置控制与读取 (带方向处理)
// ==========================================

bool AgeMotionDriver::getPosition(double &positionUm)
{
    if (!m_isConnected || !m_api_readQWORD) {
        m_lastError = "Driver not connected or function pointer invalid.";
        return false;
    }

    QWORD rawPos = 0;
    if (m_api_readQWORD(STATION_ID, AgeReg::ADDR_POS_REAL, rawPos, TIMEOUT_MS)) {
        long long signedPulses = (long long)rawPos;
        positionUm = (double)signedPulses / (MMS_PER_UM);

        // 【方向处理】反向安装时，读取到的硬件位置需要翻转
        if (m_isInverted) {
            positionUm = -positionUm;
        }
        return true;
    } else {
        m_lastError = "Failed to read QWORD from device.";
        return false;
    }
}

bool AgeMotionDriver::getTargetPosition(double &positionUm)
{
    if (!m_isConnected || !m_api_readQWORD) {
        m_lastError = "Driver not connected or function pointer invalid.";
        return false;
    }

    QWORD rawPos = 0;
    if (m_api_readQWORD(STATION_ID, AgeReg::ADDR_POS_TARGET, rawPos, TIMEOUT_MS)) {
        long long signedPulses = (long long)rawPos;
        positionUm = (double)signedPulses / (MMS_PER_UM);

        // 【方向处理】反向安装下，目标位置也需要翻转显示给用户
        if (m_isInverted) {
            positionUm = -positionUm;
        }
        return true;
    } else {
        m_lastError = "Failed to read QWORD from device.";
        return false;
    }
}

bool AgeMotionDriver::setTargetPosition(double positionUm)
{
    if (!m_isConnected || !m_api_writeQWORD) return false;

    // 恢复默认速度
    if (m_defaultTargetVelocity > 0.001) {
        setTargetVelocity(m_defaultTargetVelocity);
    }

    // 【方向处理】反向安装时，将用户视角的正位置转为硬件视角的负位置
    double hwPositionUm = m_isInverted ? -positionUm : positionUm;

    // 将微米转换为脉冲/微步 (MMS)
    long long mms = std::llround(hwPositionUm * MMS_PER_UM);
    if (m_pulseStepLengthMms > 0) {
        const double pulseIndex = static_cast<double>(mms) /
            static_cast<double>(m_pulseStepLengthMms);
        mms = std::llround(pulseIndex) *
            static_cast<long long>(m_pulseStepLengthMms);
    }

    return m_api_writeQWORD(STATION_ID, AgeReg::ADDR_POS_TARGET, (QWORD)mms, TIMEOUT_MS);
}

bool AgeMotionDriver::setRelativePosition(double deltaUm)
{
    double targetPos = 0.0;
    // 使用已封装好的获取函数（已经包含了反向逻辑）
    if (!getTargetPosition(targetPos)) return false;
    // 使用已封装好的写入函数（内部会自动再次处理反向逻辑）
    return setTargetPosition(targetPos + deltaUm);
}

// ==========================================
//          速度控制与读取
// ==========================================

bool AgeMotionDriver::getTargetRPM(double &rpm)
{
    if (!m_isConnected || !m_api_readWORD) return false;

    WORD rawVel = 0;
    if (m_api_readWORD(STATION_ID, AgeReg::ADDR_VEL_SET, rawVel, TIMEOUT_MS)) {
        rpm = (rawVel * KV_DEFAULT * 60000) / MMS_PER_R;
        return true;
    }
    return false;
}

bool AgeMotionDriver::setTargetRPM(double rpm)
{
    if (!m_isConnected || !m_api_writeWORD) return false;

    WORD val = (WORD)((rpm * MMS_PER_R) / (KV_DEFAULT * 60000));
    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_VEL_SET, val, TIMEOUT_MS);
}

bool AgeMotionDriver::getTargetVelocity(double &velocityUmPerSec)
{
    double rpm = 0.0;
    if (getTargetRPM(rpm)) {
        velocityUmPerSec = (rpm / 60.0) * POSITION_PER_R;
        return true;
    }
    return false;
}

bool AgeMotionDriver::setTargetVelocity(double velocityUmPerSec)
{
    // 速度标量不包含方向，传入绝对值计算即可
    double rpm = (qAbs(velocityUmPerSec) * 60.0) / POSITION_PER_R;
    return setTargetRPM(rpm);
}

bool AgeMotionDriver::getVelocity(double &velocityUmPerSec)
{
    if (!m_isConnected || !m_api_readWORD) return false;

    WORD rawVel = 0;
    if (m_api_readWORD(STATION_ID, AgeReg::ADDR_VEL_REAL, rawVel, TIMEOUT_MS)) {
        short signedVel = (short)rawVel;
        double rpm = (signedVel * KV_DEFAULT * 60000.0) / MMS_PER_R;
        velocityUmPerSec = (rpm / 60.0) * POSITION_PER_R;

        // 【方向处理】反向安装时，硬件的运动方向与用户视角相反
        if (m_isInverted) {
            velocityUmPerSec = -velocityUmPerSec;
        }
        return true;
    }
    return false;
}

bool AgeMotionDriver::setVelocity(double velocityUmPerSec)
{
    // 速度为0则停止
    if (qAbs(velocityUmPerSec) < 0.001) {
        return stopMotion();
    }

    // 设置运行速度标量 (取绝对值)
    if (!setTargetVelocity(qAbs(velocityUmPerSec))) return false;

    // 根据用户输入的速度方向确定目标位置（极大值）
    double targetPos = (velocityUmPerSec > 0) ? 100000000.0 : -100000000.0;

    // 【方向处理】反向安装时，硬件的极限运动方向也要反转
    if (m_isInverted) {
        targetPos = -targetPos;
    }

    if (!m_isConnected || !m_api_writeQWORD) return false;
    long long mms = (long long)(targetPos * MMS_PER_UM);
    return m_api_writeQWORD(STATION_ID, AgeReg::ADDR_POS_TARGET, (QWORD)mms, TIMEOUT_MS);
}

bool AgeMotionDriver::getVelZero(double &velocityUmPerSec)
{
    if (!m_isConnected || !m_api_readWORD) return false;

    WORD rawVel = 0;
    if (m_api_readWORD(STATION_ID, AgeReg::ADDR_VEL_ZERO, rawVel, TIMEOUT_MS)) {
        short signedVel = (short)rawVel;
        double rpm = (signedVel * KV_DEFAULT * 60000.0) / MMS_PER_R;
        velocityUmPerSec = (rpm / 60.0) * POSITION_PER_R;
        return true;
    }
    return false;
}

bool AgeMotionDriver::setVelZero(double velocityUmPerSec)
{
    if (!m_isConnected || !m_api_writeWORD) return false;

    double rpm = (velocityUmPerSec * 60.0) / POSITION_PER_R;
    WORD val = (WORD)((rpm * MMS_PER_R) / (KV_DEFAULT * 60000));
    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_VEL_ZERO, val, TIMEOUT_MS);
}

// ==========================================
//          运动控制、急停、错误监测
// ==========================================

bool AgeMotionDriver::stopMotion()
{
    if (!m_isConnected || !m_api_writeWORD) return false;
    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_CONTROL, 0x1000, TIMEOUT_MS);
}

int AgeMotionDriver::checkError()
{
    if (!m_isConnected || !m_api_readWORD) return -1;
    WORD errCode = 0;
    if (m_api_readWORD(STATION_ID, AgeReg::ADDR_ERROR_CODE, errCode, TIMEOUT_MS)) {
        return (int)errCode;
    }
    return -1;
}

bool AgeMotionDriver::setEnable(bool enable)
{
    if (!m_isConnected || !m_api_readWORD || !m_api_writeWORD) return false;
    WORD ctrl = 0;
    if (!m_api_readWORD(STATION_ID, AgeReg::ADDR_CONTROL, ctrl, TIMEOUT_MS)) return false;

    if (enable) {
        ctrl |= 0x0004;
    } else {
        ctrl &= ~0x0004;
    }
    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_CONTROL, ctrl, TIMEOUT_MS);
}

bool AgeMotionDriver::emergencyStop()
{
    if (!m_isConnected || !m_api_writeWORD) return false;
    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_CONTROL, 0x2000, TIMEOUT_MS);
}

// ==========================================
//          归零与限位运动 (带方向处理)
// ==========================================

bool AgeMotionDriver::moveToLimit(bool toUpper)
{
    if (!m_isConnected || !m_api_writeWORD) return false;

    // 【方向处理】反向安装时，用户视角的"上限位"对应硬件的"向低位运动"
    bool hwToUpper = m_isInverted ? !toUpper : toUpper;
    WORD cmd = hwToUpper ? 0x0010 : 0x0020;

    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_CONTROL, cmd, TIMEOUT_MS);
}

bool AgeMotionDriver::setCurrPositionToZero()
{
    // 位置清零不受方向影响
    if (!m_isConnected || !m_api_writeWORD) return false;
    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_CONTROL, 0x0100, TIMEOUT_MS);
}

bool AgeMotionDriver::findReference(bool toHigh)
{
    if (!m_isConnected || !m_api_writeWORD) return false;

    // 【方向处理】反向安装时，向高位找原点对应硬件的向低位运动
    bool hwToHigh = m_isInverted ? !toHigh : toHigh;
    WORD cmd = hwToHigh ? 0x0800 : 0x0400;

    return m_api_writeWORD(STATION_ID, AgeReg::ADDR_CONTROL, cmd, TIMEOUT_MS);
}

// ==========================================
//          状态与标志位读取
// ==========================================

bool AgeMotionDriver::isMotionComplete(bool &isDone)
{
    if (!m_isConnected || !m_api_readQWORD) return false;

    QWORD realPos = 0;
    QWORD targetPos = 0;

    if (!m_api_readQWORD(STATION_ID, AgeReg::ADDR_POS_REAL, realPos, TIMEOUT_MS)) return false;
    if (!m_api_readQWORD(STATION_ID, AgeReg::ADDR_POS_TARGET, targetPos, TIMEOUT_MS)) return false;

    long long diff = (long long)realPos - (long long)targetPos;
    if (diff < 0) diff = -diff;

    // ADDR_POS_REAL is observed to update in approximately 0.5 um steps.  The
    // displayed value may therefore stop as much as one feedback code away
    // from a valid 0.01 um target.  Position alone is not sufficient here:
    // accepting the wider tolerance while the axis is still moving could make
    // a fine autofocus move complete prematurely.  Require both proximity and
    // a zero real-time velocity (one raw velocity count is about 0.625 um/s).
    const long long completionToleranceMms = std::max(
        1LL, std::llround(POSITION_FEEDBACK_RESOLUTION_UM * MMS_PER_UM));

    double velocityUmPerSec = 0.0;
    if (!getVelocity(velocityUmPerSec)) return false;
    constexpr double stoppedVelocityThresholdUmPerSec = 0.5;
    isDone = (diff <= completionToleranceMms &&
              std::abs(velocityUmPerSec) < stoppedVelocityThresholdUmPerSec);
    return true;
}

bool AgeMotionDriver::isHomingComplete(bool &isDone)
{
    if (!m_isConnected || !m_api_readWORD) return false;
    WORD ctrl = 0;
    if (m_api_readWORD(STATION_ID, AgeReg::ADDR_CONTROL, ctrl, TIMEOUT_MS)) {
        isDone = ((ctrl & 0x0C00) == 0);
        return true;
    }
    return false;
}

// ==========================================
//          脉冲位置功能 (带方向处理)
// ==========================================

bool AgeMotionDriver::getPulsePosition(int &pulses)
{
    if (!m_isConnected || !m_api_readDWORD) return false;

    DWORD raw = 0;
    if (m_api_readDWORD(STATION_ID, AgeReg::ADDR_PULSE_POS_REAL, raw, TIMEOUT_MS)) {
        pulses = (int)raw;

        // 【方向处理】反向安装时将读取到的脉冲数取反
        if (m_isInverted) {
            pulses = -pulses;
        }
        return true;
    }
    return false;
}

bool AgeMotionDriver::setTargetPulsePosition(int pulses)
{
    if (!m_isConnected || !m_api_writeDWORD) return false;

    // 【方向处理】反向安装时，需将目标脉冲数取反再写入硬件
    int hwPulses = m_isInverted ? -pulses : pulses;
    return m_api_writeDWORD(STATION_ID, AgeReg::ADDR_PULSE_POS_SET, (DWORD)hwPulses, TIMEOUT_MS);
}

// ==========================================
//          其他信息读取与配置
// ==========================================

bool AgeMotionDriver::getRealTimeCurrent(double &current)
{
    if (!m_isConnected || !m_api_readWORD) return false;
    WORD raw = 0;
    if (m_api_readWORD(STATION_ID, AgeReg::ADDR_CURRENT_REAL, raw, TIMEOUT_MS)) {
        current = raw / 100.0;
        return true;
    }
    return false;
}

bool AgeMotionDriver::getCpuTemperature(int &temp)
{
    if (!m_isConnected || !m_api_readWORD) return false;
    WORD raw = 0;
    if (m_api_readWORD(STATION_ID, AgeReg::ADDR_CPU_TEMP, raw, TIMEOUT_MS)) {
        temp = (short)raw;
        return true;
    }
    return false;
}

bool AgeMotionDriver::getSingleToothResolution(unsigned int &res)
{
    if (!m_isConnected || !m_api_readDWORD) return false;
    DWORD raw = 0;
    if (m_api_readDWORD(STATION_ID, AgeReg::ADDR_T_RESOLUTION, raw, TIMEOUT_MS)) {
        res = (unsigned int)raw;
        return true;
    }
    return false;
}

bool AgeMotionDriver::getPulseStepLength(unsigned int &length)
{
    if (!m_isConnected || !m_api_readDWORD) return false;
    DWORD raw = 0;
    if (m_api_readDWORD(STATION_ID, AgeReg::ADDR_PULSE_LENGTH, raw, TIMEOUT_MS)) {
        length = (unsigned int)raw;
        m_pulseStepLengthMms = length;
        return true;
    }
    return false;
}

bool AgeMotionDriver::setPulseStepLength(unsigned int length)
{
    if (!m_isConnected || !m_api_writeDWORD || length == 0) return false;
    if (!m_api_writeDWORD(STATION_ID, AgeReg::ADDR_PULSE_LENGTH,
                          (DWORD)length, TIMEOUT_MS)) {
        return false;
    }
    m_pulseStepLengthMms = length;
    return true;
}

bool AgeMotionDriver::getMinStepUm(double &stepUm)
{
    unsigned int pulses = 0;
    if (getPulseStepLength(pulses)) {
        stepUm = (double)pulses / MMS_PER_UM;
        return true;
    }
    return false;
}

bool AgeMotionDriver::setMinStepUm(double stepUm)
{
    if (!std::isfinite(stepUm) || stepUm <= 0.0) return false;
    const long long pulseLength = std::llround(stepUm * MMS_PER_UM);
    if (pulseLength < 1 ||
        pulseLength > static_cast<long long>(
            std::numeric_limits<unsigned int>::max())) {
        return false;
    }
    return setPulseStepLength(static_cast<unsigned int>(pulseLength));
}
