#include "LogUtil.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QDateTime>
// #include <QTextStream>
#include <QMutex>
#include <QRegularExpression>

// 全局互斥锁和日志文件路径
static QMutex g_logMutex;
static QString g_logFilePath;

/**
 * @brief 自定义消息处理函数
 * 
 * @param type 消息类型
 * @param context 上下文信息（文件名、行号等）
 * @param msg 消息内容
 */
void customMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
    QMutexLocker locker(&g_logMutex);

    // 1. 控制台输出
    // 如果消息中不包含颜色代码，根据类型添加默认颜色
    QString consoleMsg = msg;
    bool hasColor = msg.contains("\033[");


    // 这里的代码是为了给不同类型的日志消息添加不同的颜色，以便在控制台中更容易区分信息、警告、错误等。
    // 颜色代码是 ANSI 转义序列，例如 "\033[32m" 表示绿色，"\033[0m" 表示重置颜色。
    if (!hasColor) {
        switch (type) {
        case QtDebugMsg:
            break; // 默认颜色
        case QtInfoMsg:
            // 绿色表示信息
            consoleMsg = QString("\033[32minfo - \033[0m%1").arg(msg);
            break;
        case QtWarningMsg:
            // 黄色表示警告
            consoleMsg = QString("\033[33mwarn - \033[0m%1").arg(msg);
            break;
        case QtCriticalMsg:
            // 红色表示错误
            consoleMsg = QString("\033[31mcrit - \033[0m%1").arg(msg);
            break;
        case QtFatalMsg:
            // 红色表示致命错误
            consoleMsg = QString("\033[31mfatal- \033[0m%1").arg(msg);
            break;
        }
    }

    // 输出到标准输出/错误 (Windows下可能需要依赖IDE捕获或在控制台运行)
    // 使用 std::cout/cerr 或者 fprintf
    FILE* outStream = (type >= QtWarningMsg) ? stderr : stdout;
    fprintf(outStream, "%s\n", qPrintable(consoleMsg));
    fflush(outStream);

    // 2. 文件输出
    if (!g_logFilePath.isEmpty()) {
        // 去除 ANSI 颜色代码
        static QRegularExpression regex("\033\\[[0-9;]*m");
        QString plainText = msg;
        plainText.remove(regex);

        // 统一加上时间戳
        QString timeStr = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
        
        // 简单处理：对于非宏生成的日志（如Qt内部警告），可能没有前缀，这里不做过度智能判断，直接记录
        QString fileMsg = QString("[%1] %2").arg(timeStr, plainText);

        QFile outFile(g_logFilePath);
        if (outFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
            QTextStream ts(&outFile);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            ts.setEncoding(QStringConverter::Utf8);
#else
            ts.setCodec("UTF-8");
#endif
            ts << fileMsg << "\n";
        }
    }
    
    // 对于 Fatal 错误，原来 default handler 会 abort，这里我们也许应该保留该行为?
    // Qt 文档建议：Fatal 消息处理后通常程序会终止。
    if (type == QtFatalMsg) {
        abort();
    }
}

void initLog(const QString& logDirName) {
    // 1. 确定日志目录
    QString logDirPath = QCoreApplication::applicationDirPath() + "/" + logDirName;

    // fprintf(stdout, "Log Directory: %s\n", qPrintable(logDirPath));
    PASS_("Log Directory: %s", qPrintable(logDirPath));

    QDir dir(logDirPath);
    if (!dir.exists()) {
        dir.mkpath(".");
    }

    // 2. 生成日志文件名 (按日期)
    QString dateStr = QDateTime::currentDateTime().toString("yyyy-MM-dd");
    g_logFilePath = logDirPath + "/Log_" + dateStr + ".txt";

    // 3. 安装消息处理器
    qInstallMessageHandler(customMessageHandler);
}
