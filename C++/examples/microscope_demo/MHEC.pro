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
    ../../src/math.cpp \
    ../../src/estimators.cpp \
    ../../src/calibration.cpp \
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


# Hardware SDKs and Eigen are kept beside the project, as in the DMH examples.
# The supplied OpenCV and pylon import libraries require Windows x64 / MSVC.
!win32:error("MHEC requires Windows x64 and the device SDKs.")
win32:!msvc:error("Use a Qt MSVC x64 kit; the supplied SDK libraries are not MinGW libraries.")
msvc:QMAKE_CXXFLAGS += /utf-8

DEPENDENCE_DIR = $$PWD/dependence
CORE_DEPENDENCE_DIR = $$PWD/../../dependence
INCLUDEPATH += $$PWD/../../include
INCLUDEPATH += $$DEPENDENCE_DIR/PI/include \
               $$CORE_DEPENDENCE_DIR/opencv/include \
               $$DEPENDENCE_DIR/pylon/include \
               $$CORE_DEPENDENCE_DIR/eigen-3.4.0

win32:CONFIG(release, debug|release): LIBS += -L$$CORE_DEPENDENCE_DIR/opencv/x64/vc16/lib/ -lopencv_world4100
else:win32:CONFIG(debug, debug|release): LIBS += -L$$CORE_DEPENDENCE_DIR/opencv/x64/vc16/lib/ -lopencv_world4100d
win32: LIBS += -L$$DEPENDENCE_DIR/pylon/lib/x64/ -lGCBase_MD_VC141_v3_1_Basler_pylon -lGenApi_MD_VC141_v3_1_Basler_pylon -lgxapi_v15 -lPylonBase_v9 -lPylonC_v9 -lPylonDataProcessing_v3 -lPylonGUI_v9 -lPylonUtility_v9 -luxapi_v14 -luxtopapi_v9

# Both drivers locate their DLL beside the executable. Do not embed source paths.
PI_DLL = $$DEPENDENCE_DIR/PI/bin/x64/PI_GCS2_DLL_x64.dll
AGE_DLL = $$DEPENDENCE_DIR/AgeMotionForDriver/x64/AgeCOM.dll
CONFIG(debug, debug|release): device_target_dir = $$OUT_PWD/debug
else: device_target_dir = $$OUT_PWD/release
device_runtime_files = $$PI_DLL $$AGE_DLL
for(device_source, device_runtime_files) {
    !exists($$device_source):error("Missing device runtime: $$device_source")
    # COPY treats '+' as concatenation, so quote even paths without spaces.
    QMAKE_POST_LINK += $$QMAKE_COPY \"$$shell_path($$device_source)\" \"$$shell_path($$device_target_dir)\" $$escape_expand(\n\t)
}

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
