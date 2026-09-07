#include "mainwindow.h"

#include <QApplication>

#include "LogUtil.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    initLog(); // 初始化日志系统
    MainWindow w;
    w.show();
    return a.exec();
}
