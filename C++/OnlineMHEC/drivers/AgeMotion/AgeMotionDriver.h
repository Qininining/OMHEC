#ifndef AGEMOTIONDRIVER_H
#define AGEMOTIONDRIVER_H

#include <QLibrary>
#include <QString>
#include <QDebug>

// --- AgeCOM 类型定义 ---
typedef long BOOL32;
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef unsigned long long QWORD;

namespace AgeReg {
    // --- 1. 控制与状态 (Control & Status) ---
    static constexpr int ADDR_CONTROL         = 0x0000; // [WORD]  控制寄存器 (使能/暂停/急停等)
    static constexpr int ADDR_ERROR_CODE      = 0x0002; // [WORD]  故障寄存器 (读取报警代码)
    static constexpr int ADDR_INPUT_TYPE      = 0x0008; // [WORD]  输入类型 (脉冲方向/正反转等)

    // --- 2. 电流设置 (Current) ---
    // 注意：电流单位通常为 0.01A (例如 200 = 2.00A)
    static constexpr int ADDR_CURRENT_MAX     = 0x0010; // [UINT16] 电流最大值 (只读)
    static constexpr int ADDR_CURRENT_MIN     = 0x0011; // [UINT16] 电流最小值 (只读)
    static constexpr int ADDR_CURRENT_SET     = 0x0012; // [UINT16] 电流设定值 (运行电流)
    static constexpr int ADDR_CURRENT_LOW     = 0x0013; // [UINT16] 闲时降流百分比 (0-100)
    static constexpr int ADDR_CURRENT_LOW_WT  = 0x0014; // [UINT16] 降流等待时间 (ms)
    static constexpr int ADDR_CURRENT_REAL    = 0x0015; // [UINT16] 实时电流 (只读)
    static constexpr int ADDR_VOLTAGE_BREAK   = 0x001E; // [UINT16] 刹车电压

    // --- 3. 位置控制 (Position) ---
    // Position 类型为 INT64 (4个WORD)，单位: MMS (微步)
    static constexpr int ADDR_POS_REAL        = 0x0020; // [INT64] 电机实时位置 (0x0020-0x0023)
    static constexpr int ADDR_POS_TARGET      = 0x0024; // [INT64] 电机目标位置 (0x0024-0x0027)

    // 分辨率相关
    static constexpr int ADDR_T_RESOLUTION    = 0x0028; // [UINT32] 单齿分辨率 (默认76800)
    static constexpr int ADDR_PULSE_LENGTH    = 0x002A; // [UINT32] 脉冲步进长度

    // 脉冲位置 (32位整数，方便通过脉冲数控制)
    static constexpr int ADDR_PULSE_POS_REAL  = 0x002C; // [INT32] 脉冲实时位置
    static constexpr int ADDR_PULSE_POS_SET   = 0x002E; // [INT32] 脉冲目标位置

    // 误差控制 (伺服/闭环功能)
    static constexpr int ADDR_POS_ERR_ALARM   = 0x0030; // [UINT32] 位差报错阈值
    static constexpr int ADDR_POS_ERR_ALLOW   = 0x0032; // [UINT32] 到位允许误差
    static constexpr int ADDR_TIME_ERR_ALLOW  = 0x0034; // [UINT16] 到位允许时间
    static constexpr int ADDR_POS_ERROR       = 0x003A; // [INT64] 实时位置误差

    // --- 4. 速度控制 (Velocity) ---
    // 速度转换公式: RPM = (VelSet * KV * 60000) / (Resolution * T)
    // 简化公式(常规): VelSet = (16 * RPM) / 5
    static constexpr int ADDR_VEL_SET         = 0x0040; // [UINT16] 运行速度设定
    static constexpr int ADDR_VEL_START       = 0x0041; // [UINT16] 启动速度
    static constexpr int ADDR_VEL_FILTER      = 0x0042; // [UINT16] 速度滤波 (加减速平滑度)
    static constexpr int ADDR_VEL_KV          = 0x0043; // [UINT16] 速度系数 (通常20)
    static constexpr int ADDR_VEL_FILTER_COM  = 0x0044; // [UINT16] 通讯控制滤波
    static constexpr int ADDR_VEL_REAL        = 0x0045; // [SHORT]  实时速度
    static constexpr int ADDR_VEL_ZERO        = 0x0046; // [UINT16] 回零速度

    // --- 5. 总线与端口 (Bus & Port) ---
    static constexpr int ADDR_BUS_WDT         = 0x0060; // [UINT16] 看门狗定时
    static constexpr int ADDR_BUS_ADDR        = 0x0061; // [UINT16] 总线站号 (1-247)
    static constexpr int ADDR_BUS_BAUD        = 0x0063; // [UINT32] 波特率
    static constexpr int ADDR_PORT_STATUS     = 0x0080; // [UINT16] IO端口状态
    static constexpr int ADDR_INPUT_BAND      = 0x0091; // [UINT32] 输入带宽

    // --- 6. 设备信息 (Info) ---
    static constexpr int ADDR_CPU_TEMP        = 0x0300; // [INT16]  CPU温度
    static constexpr int ADDR_MOTOR_SN0       = 0x2000; // [UINT64] 电机序列号0
    static constexpr int ADDR_DRIVER_NAME     = 0x8000; // [String] 驱动器型号
}

class AgeMotionDriver
{
public:
    explicit AgeMotionDriver(bool isInverted = false);
    ~AgeMotionDriver();

    bool connectDevice();

    // 注意：这里现在的单位是 微米 (μm)
    bool getPosition(double &positionUm);
    bool getTargetPosition(double &positionUm); // 获取目标位置

    // 新增功能函数
    bool getTargetRPM(double &rpm);
    bool setTargetRPM(double rpm);
    bool getTargetVelocity(double &velocityUmPerSec);
    bool setTargetVelocity(double velocityUmPerSec);

    // 恒定速度运行
    bool getVelocity(double &velocityUmPerSec); // 获取实时速度
    bool setVelocity(double velocityUmPerSec);  // 设置恒定运行速度

    bool getVelZero(double &velocityUmPerSec);
    bool setVelZero(double velocityUmPerSec);

    bool setTargetPosition(double positionUm);
    bool setRelativePosition(double deltaUm);   // 相对运动
    bool stopMotion();
    int checkError();

    // --- 新增：控制寄存器功能 ---
    bool setEnable(bool enable); // 使能/脱机
    bool emergencyStop();        // 急停

    // 运动至传感器
    bool moveToLimit(bool toUpper); // true=上限位, false=下限位
    // 位置偏移
    bool setCurrPositionToZero();
    // 回零运动
    bool findReference(bool toHigh); // true=向高位, false=向低位

    // --- 新增：状态读取 ---
    bool isMotionComplete(bool &isDone); // 运动完成标志
    bool isHomingComplete(bool &isDone); // 回零完成标志
    bool isLimitSensorTriggered(bool &upper, bool &lower); // 限位触发状态

    // --- 新增：脉冲位置功能 (INT32) ---
    bool getPulsePosition(int &pulses);
    bool setTargetPulsePosition(int pulses);

    // --- 新增：其他信息读取 ---
    bool getRealTimeCurrent(double &current); // 获取实时电流 (A)
    bool getCpuTemperature(int &temp);        // 获取CPU温度 (℃)

    // --- 新增：分辨率与步长 (UINT32) ---
    bool getSingleToothResolution(unsigned int &res);
    bool getPulseStepLength(unsigned int &length);
    bool setPulseStepLength(unsigned int length);

    // 最小步长 (μm)
    bool getMinStepUm(double &stepUm);
    bool setMinStepUm(double stepUm);

    QString getLastError() const;

private:
    // 20X autofocus command grid: 0.01 um = 10 nm = 320 MMS,
    // corresponding to 100000 pulses/revolution for the 1 mm lead screw.
    static constexpr double AUTOFOCUS_POSITION_RESOLUTION_UM = 0.01;
    // Observed granularity of ADDR_POS_REAL. This is feedback/display
    // quantization, not the command pulse resolution above.
    static constexpr double POSITION_FEEDBACK_RESOLUTION_UM = 0.5;

    // ==========================================
    //               驱动配置常量
    // ==========================================

    // 1. 物理参数
    static constexpr double TResolution_Default = 640000.0; // 单齿分辨率默认值
    static constexpr double TEETH_COUNT = 50.0; // 电机齿数
    static constexpr double POSITION_PER_R = 1000.0; // 每转微米，即 1 转 = 1 mm，丝杠螺距 1mm
    static constexpr double KV_DEFAULT = 20.0; // 默认电机速度系数，固定值
    static constexpr double MMS_PER_R = TResolution_Default * TEETH_COUNT; // 每转微步数
    static constexpr double MMS_PER_UM = MMS_PER_R / POSITION_PER_R; // 最小细分步 每微米


    // 2. 通信参数
    // 站号 (RTU Address)
    static constexpr int STATION_ID = 1;
    // 自动超时 (0 = Auto)
    static constexpr int TIMEOUT_MS = 0;

    // 4. 路径与授权 (使用 constexpr char* 确保在头文件中定义且无链接错误)
    static constexpr const char* DLL_RELATIVE_PATH = "AgeCOM.dll";

    static constexpr const char* LICENSE_KEY =
        "AgeMotion-20010203-00000000-0000-0000-0000-0000-0000-0000-0000-0000-"
        "0000-0000-0000-0000-0000-0000-0000-0000-0000-0000";

    // ==========================================

    // --- 内部成员 ---
    QLibrary m_lib;
    QString m_lastError;
    bool m_isConnected;
    unsigned int m_pulseStepLengthMms = 0;
    double m_defaultTargetVelocity = 0.0; // 默认目标速度 (um/s)

    bool m_isInverted;

    // --- 函数指针定义 ---
    typedef BOOL32 (*AgeCOMIsValidFunc)(BOOL32);
    typedef BOOL32 (*AgeCOMGetUSBIDFunc)(BYTE*);
    typedef BOOL32 (*AgeCOMReadQWORDFunc)(BYTE, WORD, QWORD&, DWORD);
    typedef BOOL32 (*AgeCOMGetCOMIDFunc)(WORD&);
    typedef BOOL32 (*AgeCOMSerialFunc)(BYTE*, DWORD);
    // 定义函数指针类型
    typedef BOOL32 (*AgeCOMReadWORDFunc)(BYTE, WORD, WORD&, DWORD);
    typedef BOOL32 (*AgeCOMWriteWORDFunc)(BYTE, WORD, WORD, DWORD);
    typedef BOOL32 (*AgeCOMWriteQWORDFunc)(BYTE, WORD, QWORD, DWORD);
    // 新增 32位读写函数指针类型
    typedef BOOL32 (*AgeCOMReadDWORDFunc)(BYTE, WORD, DWORD&, DWORD);
    typedef BOOL32 (*AgeCOMWriteDWORDFunc)(BYTE, WORD, DWORD, DWORD);

    // 成员变量
    AgeCOMReadWORDFunc  m_api_readWORD = nullptr;
    AgeCOMWriteWORDFunc m_api_writeWORD = nullptr;
    AgeCOMWriteQWORDFunc m_api_writeQWORD = nullptr;
    // 新增 32位读写成员
    AgeCOMReadDWORDFunc m_api_readDWORD = nullptr;
    AgeCOMWriteDWORDFunc m_api_writeDWORD = nullptr;

    AgeCOMIsValidFunc   m_api_isValid = nullptr;
    AgeCOMGetUSBIDFunc  m_api_getUSBID = nullptr;
    AgeCOMReadQWORDFunc m_api_readQWORD = nullptr;
    AgeCOMGetCOMIDFunc  m_api_getCOMID = nullptr;
    AgeCOMSerialFunc    m_api_setSerial = nullptr;

    bool loadLibrary();
    bool authorize();
};

#endif // AGEMOTIONDRIVER_H
