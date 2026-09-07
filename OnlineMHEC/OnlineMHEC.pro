QT       += core gui statemachine charts concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    algorithm/FocusAnalyzer/FocusAnalyzer.cpp \
    algorithm/MHEC/MHEC.cpp \
    algorithm/TargetDectect/TargetDectect.cpp \
    drivers/AgeMotion/AgeMotionDriver.cpp \
    drivers/BaslerCamera/BaslerCameraControl.cpp \
    drivers/ParallelPlatform/ParallelPlatform.cpp \
    global/LogUtil.cpp \
    task/MARobot.cpp \
    task/actor/AgeMotionActor.cpp \
    task/task/MHEC/MHECTask.cpp \
    tools/MediaSaver.cpp \
    tools/modern_robotic_lib.cpp \
    ui/main.cpp \
    ui/mainwindow.cpp

HEADERS += \
    algorithm/FocusAnalyzer/FocusAnalyzer.h \
    algorithm/MHEC/MHEC.h \
    algorithm/TargetDectect/TargetDectect.h \
    drivers/AgeMotion/AgeMotionDriver.h \
    drivers/BaslerCamera/BaslerCameraControl.h \
    drivers/ParallelPlatform/ParallelPlatform.h \
    global/ErrorCode.h \
    global/LogUtil.h \
    global/RobotException.hpp \
    task/MARobot.h \
    task/actor/AgeMotionActor.h \
    task/task/MHEC/MHECTask.h \
    task/taskTemplate/AsyncTaskBridge.hpp \
    tools/MediaSaver.h \
    tools/modern_robotic_lib.h \
    ui/mainwindow.h

FORMS += \
    ui/mainwindow.ui

INCLUDEPATH += \
    $$PWD/ui \
    $$PWD/drivers/AgeMotion \
    $$PWD/drivers/BaslerCamera \
    $$PWD/drivers/ParallelPlatform \
    $$PWD/algorithm/FocusAnalyzer \
    $$PWD/algorithm/MHEC \
    algorithm/TargetDectect/ \
    $$PWD/tools \
    $$PWD/global \
    $$PWD/task \
    $$PWD/task/task/MHEC \
    $$PWD/task/taskTemplate \
    $$PWD/task/actor \


# *************************************** 3rdparty Libraries ****************************************
# # NanoDrive
# INCLUDEPATH += $$PWD/3rdparty/NanoDrive2.8.12/04_SDK/include
# DEPENDPATH += $$PWD/3rdparty/NanoDrive2.8.12/04_SDK/lib64
# unix:!macx|win32: LIBS += -L$$PWD/3rdparty/NanoDrive2.8.12/04_SDK/lib64/ -lNTControl
# # NarPod
# INCLUDEPATH += $$PWD/3rdparty/Hexpod
# DEPENDPATH += $$PWD/3rdparty/Hexpod/X64
# win32: LIBS += -L$$PWD/3rdparty/Hexpod/X64/ -lNarpodControl

# PI GCS2
# ParallelPlatform 采用 PI 官方函数指针声明并动态加载 DLL，不需要链接导入库。
INCLUDEPATH += $$PWD/3rdparty/PI/include
PI_DLL = $$PWD/3rdparty/PI/bin/x64/PI_GCS2_DLL_x64.dll
DEFINES += PI_GCS2_LOCAL_DLL_PATH=\\\"$$PI_DLL\\\"
pi_dll_path = $$shell_path($$PI_DLL)
CONFIG(debug, debug|release): pi_target_dir = $$shell_path($$OUT_PWD/debug)
else: pi_target_dir = $$shell_path($$OUT_PWD/release)
QMAKE_POST_LINK += $$QMAKE_COPY "$$pi_dll_path" "$$pi_target_dir" $$escape_expand(\n\t)

# opencv
INCLUDEPATH += $$PWD/3rdparty/opencv/include
DEPENDPATH += $$PWD/3rdparty/opencv/include
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/3rdparty/opencv/x64/vc16/lib/ -lopencv_world4100
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/3rdparty/opencv/x64/vc16/lib/ -lopencv_world4100d
# pylon
INCLUDEPATH += $$PWD/3rdparty/pylon/include
DEPENDPATH += $$PWD/3rdparty/pylon/lib/x64
win32: LIBS += -L$$PWD/3rdparty/pylon/lib/x64/ -lGCBase_MD_VC141_v3_1_Basler_pylon -lGenApi_MD_VC141_v3_1_Basler_pylon -lgxapi_v15 -lPylonBase_v9 -lPylonC_v9 -lPylonDataProcessing_v3 -lPylonGUI_v9 -lPylonUtility_v9 -luxapi_v14 -luxtopapi_v9
# eigen
INCLUDEPATH += $$PWD/3rdparty/eigen-3.4.0


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
