#pragma once

namespace MAR {

enum ErrorCode {
    OK = 0,
    SystemError = 100,
    CreateInstanceError = 101,
    DestroyInstanceError_nullptr,
    SerialError = 200,
    SerialInitError = 201,
    SerialCloseError = 202,
    SerialSendError = 203,
    SerialReceiveError,
    SerialReceiveError_LessThanExpected,
    SerialClearBufferError,
    NatorError = 300,
    NatorInitError = 301,
    NatorSetError,
    NatorGetError,
    NatorMoveError,
    RMDError = 400,
    RMDInitError = 401,
    RMDGetPIError = 402,
    RMDFormatError,
    RMDChecksumError,
    RMDMoveError,
    MARError = 500,
    MARMoveError,
    MARNatorNotInitialized,
    MARRMDMotorNotInitialized,
    MARCameraNotInitialized,
    MARBusy,
    MARGripFailed,
    MARInternalError,
    CameraError = 600,
    CameraInitError,
    CameraReadError
};
}
