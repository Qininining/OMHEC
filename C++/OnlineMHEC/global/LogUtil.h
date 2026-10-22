#pragma once
#include <QDebug>
#include <QString>

/**
 * @brief 初始化日志系统
 * 
 * 安装自定义消息处理器，将日志输出到文件和控制台。
 * 日志文件位于程序运行目录下的 log 文件夹中，按日期命名。
 * 建议在 main 函数起始处调用。
 * 
 * @param logDir 日志文件夹名称，默认为 "log"
 */
void initLog(const QString& logDir = "log");

// 日志输出宏定义 -----------------------------------------
// 使用 Qt 的日志系统替代原有的 printf
// 保留了原有的颜色定义和前缀格式
// 支持 printf 风格的格式化参数，例如: HIGHLIGHT_("Value: %d", 100);

// 紫色高亮信息
#define HIGHLIGHT_(...) qInfo("\033[35minfo - \033[35m" __VA_ARGS__)

// 黄色警告信息
#define WARNING_(...)   qWarning("\033[33mwarn - \033[0m" __VA_ARGS__)

// 绿色通过信息
#define PASS_(...)      qDebug("\033[32minfo - \033[0m" __VA_ARGS__)

// 红色错误信息
#define ERROR_(...)     qCritical("\033[31m err - \033[0m" __VA_ARGS__)

// 普通信息 (使用 \033[0m 开头以避免被 Handler 自动着色)
#define INFO_(...)      qInfo("\033[0minfo - " __VA_ARGS__)
